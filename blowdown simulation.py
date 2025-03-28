import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.interpolate import interp1d
import random

class BlowdownSimulation:
    def __init__(self):
        # System parameters
        self.tank_volume = 5.5  # L
        self.initial_water_mass = 5.1  # kg
        self.current_water_mass = self.initial_water_mass
        self.max_flow_rate = 5  # kg/s
        self.upstream_pressure = 250  # psi
        self.initial_tank_pressure = self.upstream_pressure
        self.current_tank_pressure = self.initial_tank_pressure
        self.atmospheric_pressure = 14.7  # psi
        self.temperature = 298 # K
        self.ullage_volume = 0.5  # L
        self.noise_amplitude = 0.1

        # HJB Control parameters
        self.alpha = 0.15  # Pressure loss rate
        self.sigma = 0.5  # Random variation
        self.beta = 2  # Control penalty
        self.threshold_pressure = 200  # psi
        self.dt = 0.1  # seconds
        self.activation_time = 5  # seconds
        self.is_active = False
        
        # Solenoid valve parameters
        self.cv = 0.03
        self.solenoid_state = False
        self.pressurant_mass = 2.0  # kg
        self.pressurant_usage = 0.0  # kg
        
        # New stability parameters
        self.stability_window = 5  # samples
        self.pressure_history = []
        self.stability_threshold = 1.0  # psi
        self.threshold_adapt_rate = 5
        self.min_threshold = 200  # psi
        self.max_threshold = 280  # psi

        # Air properties
        self.air_R = 287.05  # Gas constant for air (J/kg·K)
        self.air_gamma = 1.4  # Specific heat ratio
        self.air_critical_pressure = 547.2  # psi
        self.air_Z = 1.0  # Initial compressibility factor
        
        # End-phase behavior parameters
        self.end_phase_threshold = 0.1  # L
        self.air_discharge_coefficient = 0.85
        self.in_end_phase = False
        
        # Venturi parameters
        self.venturi_diameter = 0.015  # m
        self.discharge_coefficient = 0.98
        self.beta = 0.5  # diameter ratio
        
        # Flow parameters
        self.pipe_roughness = 0.000015  # m
        self.pipe_diameter = 0.02  # m
        self.dynamic_viscosity = 8.9e-4  # Pa·s for water
        
        # Initial conditions
        self.initial_water_mass = 5.5  # kg
        self.current_water_mass = self.initial_water_mass
        self.initial_tank_pressure = self.upstream_pressure
        self.current_tank_pressure = self.initial_tank_pressure
        
        # Time parameters
        self.dt = 0.1
        self.activation_time = 5
        self.is_active = False
        
        # Data storage
        self.times = []
        self.tank_pressures = []
        self.flow_rates = []
        self.upstream_pressures = []
        self.reynolds_numbers = []
        self.solenoid_states = []  
        self.control_signals = []   
        self.value_functions = []

    def is_pressure_stable(self):
        if len(self.pressure_history) < self.stability_window:
            return False
        
        recent_pressures = self.pressure_history[-self.stability_window:]
        pressure_range = max(recent_pressures) - min(recent_pressures)
        return pressure_range < self.stability_threshold
    
    def calculate_pressure_loss(self):
        """Calculate natural pressure loss due to flow"""
        if not self.is_active:
            return 0
            
        # Basic pressure loss proportional to flow rate and current pressure
        base_loss = self.alpha * self.current_tank_pressure * self.dt
        
        # Add random variation
        noise = np.random.normal(0, self.sigma * self.dt)
        
        return base_loss + noise
    
    def calculate_pressure_increase(self):
        """Calculate pressure increase from control action"""
        if not self.solenoid_state:
            return 0
            
        control = self.control_signals[-1]
        delta_p = self.upstream_pressure - self.current_tank_pressure
        
        if delta_p <= 0:
            return 0
            
        # Pressure increase proportional to control signal and pressure difference
        increase = control * self.cv * np.sqrt(delta_p) * self.dt
        return increase
    
    def solve_hjb(self, current_pressure):
        # Update pressure history
        self.pressure_history.append(current_pressure)
        
        # Adaptive threshold adjustment
        if self.is_pressure_stable():
            # Calculate new threshold based on recent stable pressure
            recent_mean = sum(self.pressure_history[-self.stability_window:]) / self.stability_window
            threshold_error = self.threshold_pressure - recent_mean
            self.threshold_pressure += self.threshold_adapt_rate * threshold_error
            
            # Constrain threshold within bounds
            self.threshold_pressure = max(min(self.threshold_pressure, 
                                           self.max_threshold), 
                                       self.min_threshold)
        
        # Calculate control signal
        pressure_error = self.threshold_pressure - current_pressure
        value = 0.5 * pressure_error**2
        
        # Enhanced bang-bang control
        if abs(pressure_error) < self.stability_threshold:
            optimal_u = 0.5  # Partial activation for stability
        else:
            optimal_u = 1.0 if pressure_error > 0 else 0.0
            
        return optimal_u, value
    
        
    def predict_pressure_trajectory(self):
        """Use BSDE to predict pressure trajectory"""
        dt = self.control_dt
        steps = int(self.control_horizon / dt)
        
        # Initialize trajectories
        P = np.zeros(steps)
        P[0] = self.current_tank_pressure
        
        # Forward simulation with noise
        for i in range(1, steps):
            dW = np.random.normal(0, np.sqrt(dt))
            dP = (-self.alpha * P[i-1] * dt + 
                  self.sigma * dW)
            P[i] = P[i-1] + dP
            
        return P
    def update_control_system(self):
        """Update pressure control using HJB-based strategy"""
        # Predict future pressure trajectory
        pressure_trajectory = self.predict_pressure_trajectory()
        
        # Solve HJB equation for optimal control
        optimal_u, value = self.solve_hjb(self.current_tank_pressure)
        
        # Update solenoid state based on optimal control
        self.solenoid_state = optimal_u > 0.5
        
        # Store data
        self.solenoid_states.append(self.solenoid_state)
        self.control_signals.append(optimal_u)
        self.value_functions.append(value)
    def calculate_pressurant_flow(self):
        """Calculate pressurant flow with numerical stability"""
        if not self.solenoid_state:
            return 0.0
            
        P1 = self.upstream_pressure
        P2 = self.current_tank_pressure
        
        if P1 <= P2:
            return 0.0
            
        # Control-modulated flow with bounds checking
        control_signal = max(min(self.control_signals[-1], 1.0), 0.0)
        
        # Prevent negative or zero density
        rho = max(P1 * 6894.76 / (self.air_R * self.temperature), 1e-10)
        
        # Calculate flow rate with bounded pressure difference
        delta_P = max(P1 - P2, 0.0)
        flow_rate = self.cv * control_signal * np.sqrt(rho * delta_P)
        
        # Update pressurant usage with bounds checking
        self.pressurant_usage = min(self.pressurant_usage + flow_rate * self.dt,
                                  self.pressurant_mass)
        
        return flow_rate
    def calculate_reynolds_number(self, velocity):
        """Calculate Reynolds number for the flow"""
        density_water = 997  # kg/m³
        return (density_water * velocity * self.pipe_diameter) / self.dynamic_viscosity

    def calculate_friction_factor(self, reynolds):
        """Calculate Darcy friction factor using Colebrook-White equation"""
        if reynolds < 2300:
            return 64 / reynolds
        
        def colebrook(f):
            return (1 / np.sqrt(f)) + 2 * np.log10(
                (self.pipe_roughness / (3.7 * self.pipe_diameter)) + 
                (2.51 / (reynolds * np.sqrt(f)))
            )
        
        # Initial guess using Swamee-Jain equation
        f0 = 0.25 / (np.log10(self.pipe_roughness / (3.7 * self.pipe_diameter) + 
                             5.74 / reynolds**0.9))**2
        
        # Newton-Raphson iteration
        for _ in range(10):
            f1 = f0 - colebrook(f0) / (-2 * f0**(-1.5))
            if abs(f1 - f0) < 1e-6:
                return f1
            f0 = f1
        
        return f0

    def calculate_compressibility_factor(self, pressure, temperature):
        """Calculate air compressibility factor using simplified Redlich-Kwong"""
        Pr = pressure / self.air_critical_pressure
        Tr = temperature / 132.5  # 132.5 K is critical temperature of air
        
        # Simplified Redlich-Kwong for air
        a = 0.4278
        b = 0.0867
        Z = 1 + (b - 1/Tr) * Pr - (a/Tr**1.5) * Pr
        return max(Z, 0.4)

    def calculate_air_flow_rate(self):
        """Calculate flow rate during end phase (air discharge)"""
        P1 = self.current_tank_pressure * 6894.76  # psi to Pa
        P2 = self.atmospheric_pressure * 6894.76  # psi to Pa
        
        if P1 <= P2:
            return 0
        
        rho_air = (P1) / (self.air_R * self.temperature * self.air_Z)
        critical_ratio = (2 / (self.air_gamma + 1)) ** (self.air_gamma / (self.air_gamma - 1))
        pressure_ratio = P2 / P1
        
        area = np.pi * (self.venturi_diameter/2)**2
        
        if pressure_ratio > critical_ratio:
            # Subsonic flow
            flow_factor = np.sqrt((2 * self.air_gamma / (self.air_gamma - 1)) * 
                                ((pressure_ratio)**(2/self.air_gamma) - 
                                 (pressure_ratio)**((self.air_gamma + 1)/self.air_gamma)))
        else:
            # Choked flow
            flow_factor = np.sqrt(self.air_gamma * (2 / (self.air_gamma + 1))
                                **((self.air_gamma + 1)/(self.air_gamma - 1)))
        
        air_flow = self.air_discharge_coefficient * area * np.sqrt(2 * P1 * rho_air) * flow_factor
        
        if self.in_end_phase:
            # Fix exponential overflow
            mass_ratio = min(self.current_water_mass / max(self.end_phase_threshold, 1e-10), 100)
            spike_factor = 1.5 * np.exp(-5 * mass_ratio)
            air_flow *= (1 + min(spike_factor, 10))  # Limit maximum spike
        
        return air_flow

    def calculate_flow_rate(self):
        if not self.is_active:
            return 0

        if self.current_water_mass <= self.end_phase_threshold:
            self.in_end_phase = True
            return self.calculate_air_flow_rate()

        P1 = self.current_tank_pressure * 6894.76
        P2 = self.atmospheric_pressure * 6894.76

        if P1 <= P2:
            return 0
            
        rho = 997  # kg/m³
        area = np.pi * (self.venturi_diameter/2)**2

        # Modified pressure-flow relationship
        delta_P = P1 - P2
        velocity_theoretical = np.sqrt(2 * delta_P / rho)

        # Account for remaining water mass
        mass_factor = (self.current_water_mass / self.initial_water_mass)**0.5

        Re = self.calculate_reynolds_number(velocity_theoretical)
        f = self.calculate_friction_factor(Re)
        L = 0.5

        head_loss = f * L * velocity_theoretical**2 / (2 * self.pipe_diameter)
        velocity_actual = velocity_theoretical * np.sqrt(max(0, 1 - head_loss/velocity_theoretical**2))

        # Apply mass factor to flow rate
        flow_rate = self.discharge_coefficient * area * velocity_actual * rho * mass_factor

        self.reynolds_numbers.append(Re)
        return min(flow_rate, self.max_flow_rate)


    def update_pressure(self):
        if not self.is_active:
            return self.current_tank_pressure
            
        # Calculate pressure changes with stability consideration
        base_loss = self.alpha * self.current_tank_pressure * self.dt
        noise = np.random.normal(0, self.sigma * self.dt)
        
        # Add control influence
        control = self.control_signals[-1]
        delta_p = self.upstream_pressure - self.current_tank_pressure
        pressure_increase = control * self.cv * np.sqrt(max(delta_p, 0)) * self.dt
        
        # Update pressure with bounds
        new_pressure = max(
            self.atmospheric_pressure,
            min(
                self.current_tank_pressure - base_loss + pressure_increase + noise,
                self.upstream_pressure
            )
        )
        
        return new_pressure


    def get_upstream_pressure(self):
        noise = np.random.normal(0, self.noise_amplitude/3)
        return max(self.upstream_pressure + noise, self.atmospheric_pressure)

    def step(self, t):
        if t >= self.activation_time:
            self.is_active = True
        
        # Calculate control
        control, value = self.solve_hjb(self.current_tank_pressure)
        self.solenoid_state = control > 0.5
        
        # Calculate system dynamics
        flow_rate = self.calculate_flow_rate()
        pressure_loss = self.calculate_pressure_loss()
        pressure_increase = self.calculate_pressure_increase()
        
        # Update state
        self.current_water_mass = max(0, self.current_water_mass - flow_rate * self.dt)
        self.current_tank_pressure = max(
            self.atmospheric_pressure,
            self.current_tank_pressure - pressure_loss + pressure_increase
        )
        
        # Store data
        self.times.append(t)
        self.tank_pressures.append(self.current_tank_pressure)
        self.flow_rates.append(flow_rate)
        self.upstream_pressures.append(self.upstream_pressure)
        self.solenoid_states.append(self.solenoid_state)
        self.control_signals.append(control)
        self.value_functions.append(value)
        
        return (self.current_tank_pressure, flow_rate, self.upstream_pressure,
                self.solenoid_state, control, value)

# Set up the plotting
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12))
sim = BlowdownSimulation()

# Initialize lines
line1_tank, = ax1.plot([], [], label='Tank Pressure')
line1_upstream, = ax1.plot([], [], label='Upstream Pressure')
line1_threshold, = ax1.plot([], [], '--', label='Threshold Pressure')
line2, = ax2.plot([], [], label='Flow Rate')
line3_value, = ax3.plot([], [], label='Value Function')
line3_control, = ax3.plot([], [], '--', label='Control Signal')

# Set up plot parameters
ax1.set_xlim(0, 30)
ax1.set_ylim(0, 500)
ax1.set_ylabel('Pressure (psi)')
ax1.grid(True)
ax1.legend()

ax2.set_xlim(0, 30)
ax2.set_ylim(0, 100)
ax2.set_ylabel('Flow Rate (kg/s)')
ax2.grid(True)
ax2.legend()

ax3.set_xlim(0, 30)
ax3.set_ylim(0, 200)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Value / Control')
ax3.grid(True)
ax3.legend()

# Store solenoid indicator lines
solenoid_lines = []

def animate(frame):
    t = frame * sim.dt
    tank_pressure, flow_rate, upstream_pressure, solenoid_state, control, value = sim.step(t)
    
    # Update main data
    line1_tank.set_data(sim.times, sim.tank_pressures)
    line1_upstream.set_data(sim.times, [sim.upstream_pressure] * len(sim.times))
    line1_threshold.set_data(sim.times, [sim.threshold_pressure] * len(sim.times))
    line2.set_data(sim.times, sim.flow_rates)
    line3_value.set_data(sim.times, sim.value_functions)
    line3_control.set_data(sim.times, sim.control_signals)
    
    # Add solenoid indicators
    if len(sim.solenoid_states) > 1:
        if sim.solenoid_states[-1] != sim.solenoid_states[-2]:
            color = 'red' if sim.solenoid_states[-1] else 'black'
            for ax in [ax1, ax2, ax3]:
                solenoid_lines.append(ax.axvline(x=t, color=color, alpha=0.3))
    
    return (line1_tank, line1_upstream, line1_threshold, line2,
            line3_value, line3_control, *solenoid_lines)
# Create animation
anim = FuncAnimation(fig, animate, frames=300, interval=100, blit=True)
plt.tight_layout()
plt.show()