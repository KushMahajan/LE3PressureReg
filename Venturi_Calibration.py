import threading
import numpy as np
import serial
import csv
import time
from datetime import datetime
import os.path
import sys, msvcrt

class VenturiCalibrator:
    def __init__(self, port="COM5", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.stop_thread = threading.Event()
        self.serial_lock = threading.Lock()
        self.test_data = []
        self.coefficients = None
        self.poly_order = 1  # Default to linear regression
        
        # Timestamps for flow duration
        self.flow_start_time = None
        self.flow_end_time = None
        
        # Flow test parameters
        self.initial_mass = 0.0
        self.final_mass = 0.0
        self.test_duration = 0
        self.actual_flow_rate = 0.0
        
        # Calibration results
        self.calibration_points = []  # [(theoretical_flow, actual_flow)]
        
        # Flag to control console output
        self.display_output = True
        
        # Create CSV file for data storage
        self.setup_csv_files()
        
    def setup_csv_files(self):
        # Create timestamp for file names
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Raw data file
        self.raw_data_file = f"venturi_raw_data_{timestamp}.csv"
        with open(self.raw_data_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'Timestamp', 'Raw_Tank', 'Raw_Venturi1', 'Raw_Venturi2', 
                'Tank_Pressure_PSI', 'Venturi1_Pressure_PSI', 'Venturi2_Pressure_PSI', 
                'Theoretical_Flow_LPM', 'Flow_Active'
            ])
        
        # Calibration results file
        self.calibration_file = f"venturi_calibration_{timestamp}.csv"
        with open(self.calibration_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'Test_Number', 'Duration_ms', 'Initial_Mass_g', 'Final_Mass_g', 
                'Mass_Change_g', 'Theoretical_Flow_Integral', 'Actual_Flow_Rate',
                'Theoretical_Avg_Flow'
            ])
            
        # Store polynomial coefficients separately
        self.coefficients_file = f"venturi_coefficients_{timestamp}.csv"

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            return False
            
    def disconnect_serial(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial connection closed")
            
    def start_data_collection(self):
        self.stop_thread.clear()
        self.test_data = []
        self.display_output = True
        self.serial_thread = threading.Thread(target=self._read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
    def stop_data_collection(self):
        self.stop_thread.set()
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join(timeout=2)
            
    def _read_serial_data(self):
        """Thread function to read data from serial port"""
        while not self.stop_thread.is_set():
            try:
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.is_open and self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        
                        # Process data lines
                        if line.startswith("DATA:"):
                            data_parts = line[5:].split(',')
                            if len(data_parts) >= 9:
                                # Parse data values
                                timestamp = int(data_parts[0])
                                raw_tank = float(data_parts[1])
                                raw_venturi1 = float(data_parts[2])
                                raw_venturi2 = float(data_parts[3])
                                tank_pressure = float(data_parts[4])
                                venturi1_pressure = float(data_parts[5])
                                venturi2_pressure = float(data_parts[6])
                                theoretical_flow = float(data_parts[7])
                                flow_active = int(data_parts[8])
                                
                                # Store the data
                                data_point = {
                                    'timestamp': timestamp,
                                    'raw_tank': raw_tank,
                                    'raw_venturi1': raw_venturi1,
                                    'raw_venturi2': raw_venturi2,
                                    'tank_pressure': tank_pressure,
                                    'venturi1_pressure': venturi1_pressure,
                                    'venturi2_pressure': venturi2_pressure,
                                    'theoretical_flow': theoretical_flow,
                                    'flow_active': flow_active
                                }
                                self.test_data.append(data_point)
                                
                                # Write to CSV
                                with open(self.raw_data_file, 'a', newline='') as file:
                                    writer = csv.writer(file)
                                    writer.writerow([
                                        timestamp, raw_tank, raw_venturi1, raw_venturi2,
                                        tank_pressure, venturi1_pressure, venturi2_pressure,
                                        theoretical_flow, flow_active
                                    ])
                                
                                # Print status to console (only if display_output is True)
                                if self.display_output:
                                    print(f"Flow: {theoretical_flow:.2f} LPM | Tank: {tank_pressure:.2f} PSI | "
                                          f"Venturi: {venturi1_pressure:.2f}/{venturi2_pressure:.2f} PSI | "
                                          f"Active: {flow_active}")
                        else:
                            # Print any other messages from Arduino (only if display_output is True)
                            if self.display_output:
                                print(f"Arduino: {line}")
            except Exception as e:
                if self.display_output:
                    print(f"Error reading from serial: {e}")
                time.sleep(0.1)
                continue
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
    
    def send_flow_command(self, duration_ms):
        """Send command to Arduino to start flow for the specified duration"""
        if self.serial_conn and self.serial_conn.is_open:
            with self.serial_lock:
                command = f"START:{duration_ms}\n"
                self.serial_conn.write(command.encode())
                self.flow_start_time = time.time()
                print(f"Flow command sent: {duration_ms} ms")
                return True
        else:
            print("Serial connection not available")
            return False
    
    def wait_for_flow_completion(self, duration_ms):
        """Wait for flow to complete plus a small buffer time"""
        wait_time = (duration_ms / 1000.0) + 0.5  # Add 500ms buffer
        print(f"Waiting for flow to complete ({wait_time:.1f} seconds)...")
        time.sleep(wait_time)
        self.flow_end_time = time.time()
        print("Flow complete.")
        
    def calculate_theoretical_flow_integral(self):
        """Calculate the integral of theoretical flow over the test duration"""
        flow_data = []
        timestamps = []
        
        # Extract flow data during active flow period
        for point in self.test_data:
            if point['flow_active'] == 1:
                flow_data.append(point['theoretical_flow'])
                timestamps.append(point['timestamp'])
        
        if not flow_data:
            print("No flow data collected during test")
            return 0.0
            
        # Calculate time intervals in seconds
        time_intervals = []
        for i in range(1, len(timestamps)):
            interval = (timestamps[i] - timestamps[i-1]) / 1000.0  # Convert ms to seconds
            time_intervals.append(interval)
            
        # Calculate flow integral (trapezoidal rule)
        flow_integral = 0.0
        for i in range(len(time_intervals)):
            flow_integral += 0.5 * (flow_data[i] + flow_data[i+1]) * time_intervals[i]
            
        # Convert from LPM*sec to L (divide by 60)
        flow_integral = flow_integral / 60.0
        
        # Convert from L to g (assuming water density = 1 g/mL)
        flow_integral = flow_integral * 1000.0
            
        return flow_integral
        
    def calculate_actual_flow(self):
        """Calculate actual flow based on mass change"""
        mass_change = self.final_mass - self.initial_mass  # in grams
        duration_sec = (self.flow_end_time - self.flow_start_time)  # in seconds
        
        # Flow rate in g/sec
        flow_rate_g_sec = mass_change / duration_sec
        
        # Convert to LPM (g/sec to mL/min, assuming 1g = 1mL for water)
        flow_rate_lpm = flow_rate_g_sec * 60 / 1000
        
        return flow_rate_lpm, mass_change
        
    def calculate_average_theoretical_flow(self):
        """Calculate average theoretical flow during the test"""
        flow_values = []
        for point in self.test_data:
            if point['flow_active'] == 1:
                flow_values.append(point['theoretical_flow'])
                
        if not flow_values:
            return 0.0
        
        return np.mean(flow_values)
        
    def perform_calibration(self):
        """Calculate calibration coefficients using polynomial regression"""
        if len(self.calibration_points) < 2:
            print("Need at least 2 data points for calibration")
            return False
            
        theoretical_flows = [point[0] for point in self.calibration_points]
        actual_flows = [point[1] for point in self.calibration_points]
        
        # Perform polynomial regression
        self.coefficients = np.polyfit(theoretical_flows, actual_flows, self.poly_order)
        
        # Calculate R-squared
        p = np.poly1d(self.coefficients)
        predicted = p(theoretical_flows)
        ss_total = np.sum((actual_flows - np.mean(actual_flows))**2)
        ss_residual = np.sum((actual_flows - predicted)**2)
        r_squared = 1 - (ss_residual / ss_total)
        
        print("\nCalibration Results:")
        print(f"Polynomial Order: {self.poly_order}")
        print(f"Coefficients: {self.coefficients}")
        print(f"R-squared: {r_squared:.4f}")
        
        # Save coefficients to file
        with open(self.coefficients_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Polynomial_Order', 'R_Squared'] + [f'Coefficient_{i}' for i in range(len(self.coefficients))])
            writer.writerow([self.poly_order, r_squared] + list(self.coefficients))
            
        return True
        
    def run_calibration_test(self, test_number):
        """Run a complete calibration test"""
        try:
            # Get test parameters from user
            while True:
                try:
                    duration_sec = float(input("\nEnter test duration in seconds: "))
                    if duration_sec <= 0:
                        print("Duration must be positive")
                        continue
                    break
                except ValueError:
                    print("Please enter a valid number")
            
            duration_ms = int(duration_sec * 1000)
            
            while True:
                try:
                    self.initial_mass = float(input("Enter initial bucket mass in grams: "))
                    if self.initial_mass < 0:
                        print("Mass cannot be negative")
                        continue
                    break
                except ValueError:
                    print("Please enter a valid number")
            
            ready = input("\nReady to start test? (y/n): ")
            if ready.lower() not in ['y', 'yes']:
                print("Test aborted")
                return False
                
            # Clear previous test data
            self.test_data = []
            
            # Start data collection
            self.start_data_collection()
            
            # Wait a moment for data collection to initialize
            time.sleep(1)
            
            # Send flow command to Arduino
            if not self.send_flow_command(duration_ms):
                self.stop_data_collection()
                return False
                
            # Wait for flow to complete
            self.wait_for_flow_completion(duration_ms)
            
            # Continue collecting data but stop displaying it to console
            self.display_output = False
            print("\n--- Flow test completed. Collecting final data ---")
            
            # Continue collecting data for a short period after flow stops
            time.sleep(1)
            
            # Clear any pending input
            
            try:
                # Only try to flush stdin if it's a terminal
                if sys.stdin.isatty():
                    while msvcrt.kbhit():
                        msvcrt.getch()
            except:
                pass
                
            # Get final mass from user (with display output stopped)
            while True:
                try:
                    self.final_mass = float(input("\nEnter final bucket mass in grams: "))
                    if self.final_mass < self.initial_mass:
                        print("Final mass should be greater than initial mass")
                        continue
                    break
                except ValueError:
                    print("Please enter a valid number")
            
            # Calculate theoretical flow integral
            theoretical_flow_integral = self.calculate_theoretical_flow_integral()
            print(f"Theoretical flow integral: {theoretical_flow_integral:.2f} g")
            
            # Calculate actual flow
            actual_flow_rate, mass_change = self.calculate_actual_flow()
            print(f"Actual mass change: {mass_change:.2f} g")
            print(f"Actual average flow rate: {actual_flow_rate:.2f} LPM")
            
            # Calculate average theoretical flow
            avg_theoretical_flow = self.calculate_average_theoretical_flow()
            print(f"Average theoretical flow: {avg_theoretical_flow:.2f} LPM")
            
            # Store calibration point
            self.calibration_points.append((avg_theoretical_flow, actual_flow_rate))
            
            # Save test results to CSV
            with open(self.calibration_file, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    test_number, duration_ms, self.initial_mass, self.final_mass,
                    mass_change, theoretical_flow_integral, actual_flow_rate,
                    avg_theoretical_flow
                ])
            
            # Stop data collection
            self.stop_data_collection()
            
            return True
        
        except Exception as e:
            print(f"Error during calibration test: {e}")
            self.stop_data_collection()
            return False

def main():
    print("\n=== Venturi Flow Calibration System ===")
    
    # Ask for serial port
    port = input("Enter serial port (default: COM5): ")
    if not port:
        port = "COM5"
    
    # Create calibrator instance
    calibrator = VenturiCalibrator(port=port)
    
    # Ask for polynomial order
    while True:
        try:
            poly_order = int(input("Enter polynomial order for calibration (1-3, default: 1): "))
            if 1 <= poly_order <= 3:
                calibrator.poly_order = poly_order
                break
            else:
                print("Order must be between 1 and 3")
        except ValueError:
            calibrator.poly_order = 1
            print("Using default order: 1 (linear)")
            break
    
    # Connect to Arduino
    if not calibrator.connect_serial():
        print("Failed to connect to Arduino. Exiting.")
        return
    
    # Wait for Arduino to initialize
    print("Waiting for Arduino to initialize...")
    time.sleep(2)
    
    try:
        test_number = 1
        continue_testing = True
        
        while continue_testing:
            print(f"\n--- Test #{test_number} ---")
            success = calibrator.run_calibration_test(test_number)
            
            if success:
                test_number += 1
                
                # Ask if user wants to continue with another test
                response = input("\nPerform another test? (y/n): ")
                continue_testing = response.lower() in ['y', 'yes']
            else:
                response = input("\nRetry this test? (y/n): ")
                continue_testing = response.lower() in ['y', 'yes']
        
        # Perform calibration if we have data points
        if len(calibrator.calibration_points) > 0:
            print("\nPerforming final calibration...")
            calibrator.perform_calibration()
            
            # Print calibration function
            if calibrator.coefficients is not None:
                print("\nCalibration Function:")
                formula = "actual_flow = "
                for i, coef in enumerate(calibrator.coefficients):
                    power = len(calibrator.coefficients) - i - 1
                    if power > 0:
                        formula += f"{coef:.6f} * theoretical_flow^{power} + "
                    else:
                        formula += f"{coef:.6f}"
                print(formula)
        
    except KeyboardInterrupt:
        print("\nCalibration process interrupted by user")
    finally:
        # Clean up
        calibrator.stop_data_collection()
        calibrator.disconnect_serial()
        print("\nCalibration process complete. Data saved to CSV files.")

if __name__ == "__main__":
    main()