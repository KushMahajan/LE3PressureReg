/*
  Integrated Pressure Control System with Dynamic Threshold Optimization
  - Implements HJB and BSDE-based control methodology
  - Uses Venturi flow measurement
  - Dynamic threshold adaptation with stability monitoring
  - Separate upstream and downstream solenoid control
*/

// Pin Definitions
#define VALVE_UP_PIN 13    // Upstream solenoid open
//#define VALVE_UP_CLOSE_PIN 14   // Upstream solenoid close
#define VALVE_DOWN_PIN 12  // Downstream solenoid open
//#define VALVE_DOWN_CLOSE_PIN 16 // Downstream solenoid close
#define PT_UPSTREAM 25    // Upstream pressure
#define PT_TANK 26       // Tank pressure
#define PT_DOWNSTREAM 27  // Downstream pressure
#define PT_VENTURI_1 32  // Venturi inlet pressure
#define PT_VENTURI_2 35  // Venturi throat pressure

// Venturi constants
const float D1 = 0.375; // inches
const float D2 = 0.5;   // inches
const float A1 = PI * pow(D1/2, 2);
const float A2 = PI * pow(D2/2, 2);
const float RHO = 998.2; // Water density at 20°C in kg/m³
const float CONVERSION_FACTOR = 6894.76; // Convert PSI to Pascal

// System parameters from optimization paper
const float ALPHA = 0.1;        // Natural pressure loss rate
const float BETA = 0.5;         // Upstream control penalty
const float DELTA = 0.5;        // Downstream control penalty
const float GAMMA = 0.3;        // Oscillatory control weight
const float LAMBDA = 0.2;       // Instability penalty

// Control states
enum ControlState {
    NOMINAL_REGULATION,
    RECOVERY_DAMPING,
    HIGH_FREQUENCY_OSCILLATION
};

const char* stateStrings[] = {
    "NOMINAL",
    "RECOVERY",
    "OSCILLATION"
};


// Moving average filter
const int FILTER_WINDOW = 20;
float pressure_buffer[5][FILTER_WINDOW];
int buffer_index = 0;

// System state variables
ControlState current_state = NOMINAL_REGULATION;
float P_threshold_base = 50.0;
float P_threshold_down;
float P_threshold_current;
float P_fuel;
float dP_fuel_dt;
unsigned long last_update = 0;
const int UPDATE_INTERVAL = 50; // 50ms update interval

float P_threshold_up;  // Upstream pressure threshold
float P_threshold_up_base = P_threshold_base;  // Higher base threshold for upstream
float integral_error_up = 0;  // Separate integral error for upstream control
float last_pressure_error_up = 0;  // Separate last error for upstream control

const unsigned long deltaT = 1000;  // 0.1s in milliseconds

// Threshold optimization parameters
const float DT = UPDATE_INTERVAL / 1000.0;  // Convert to seconds
const float K_P = 0.8;  // Proportional gain
const float K_I = 0.2;  // Integral gain
const float K_D = 0.3;  // Derivative gain
const float MAX_THRESHOLD_CHANGE = 25;
const float STABILITY_EPSILON = 5.0;

// PID state variables
float last_pressure_error = 0;
float integral_error = 0;

// Calibration coefficients
const float PT_1_A = -1.14351422603069E-09;
const float PT_1_B = 6.07260454422966E-06;
const float PT_1_C = 0.0926898002815586;
const float PT_1_D = -45.4811452217083;

const float PT_2_A = -8.46148513e-09;
const float PT_2_B = 3.85591637e-05;
const float PT_2_C = 1.50430101e-01;
const float PT_2_D = -8.31516318e+01;

const float PT_3_A = -1.12886040e-08;
const float PT_3_B =  5.37687043e-05;
const float PT_3_C = 1.24582809e-01;
const float PT_3_D = -7.92816379e+01;

const float PT_4_A = -2.92357884e-09;
const float PT_4_B = 1.60928197e-05;
const float PT_4_C = 1.73172679e-01;
const float PT_4_D = -8.41620248e+01;

const float PT_5_A = -2.07890243e-09;
const float PT_5_B =  7.98159784e-06;
const float PT_5_C = 1.92631458e-01;
const float PT_5_D =  -7.84202084e+01;

// Function declarations
float calculatePressure(float raw_value, float PT_A, float PT_B, float PT_C, float PT_D);
float calculateVenturiFlow(float p1, float p2);
float updateMovingAverage(float new_value, float* buffer);
void calculatePressureDerivative(float current_pressure);
void updateDynamicThreshold();
float calculateInstability(float error, float error_derivative);
void determineControlState();
void applyOptimalControl();
void controlUpstreamSolenoid(float u_up);
void controlDownstreamSolenoid(float u_down);

float calculatePressure(float raw_value, float PT_A, float PT_B, float PT_C, float PT_D) {
    return (PT_A * pow(raw_value, 3)) +
           (PT_B * pow(raw_value, 2)) +
           (PT_C * raw_value) + PT_D;
}

float calculateVenturiFlow(float p1, float p2) {
    float dp = (p1 - p2) * CONVERSION_FACTOR;
    float term1 = 2 * dp / RHO;
    float term2 = 1 - pow(A2/A1, 2);
    return A2 * sqrt(term1 / term2) * 60000; // Convert to L/min
}

float updateMovingAverage(float new_value, float* buffer) {
    buffer[buffer_index] = new_value;
    float sum = 0;
    for(int i = 0; i < FILTER_WINDOW; i++) {
        sum += buffer[i];
    }
    return sum / FILTER_WINDOW;
}

void calculatePressureDerivative(float current_pressure) {
    static float last_pressure = current_pressure;
    static unsigned long last_time = millis();
    
    if (isnan(current_pressure)) {
        //Serial.println("NaN detected in current_pressure");
        return;
    }
    
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 2000.0;
    
    // Prevent division by zero or very small dt
    if (dt > 0.001) {
        float new_derivative = (current_pressure - last_pressure) / dt;
        if (!isnan(new_derivative)) {
            dP_fuel_dt = new_derivative;
        }
    }
    
    last_pressure = current_pressure;
    last_time = current_time;
}

/*void printDebugValues(float error, float error_derivative) {
    Serial.print("Debug - Error: ");
    Serial.print(error);
    Serial.print(" Error_deriv: ");
    Serial.print(error_derivative);
    Serial.print(" STABILITY_EPSILON: ");
    Serial.print(STABILITY_EPSILON);
    Serial.print(" MAX_THRESHOLD_CHANGE: ");
    Serial.println(MAX_THRESHOLD_CHANGE);
}
*/

float calculateInstability(float error, float error_derivative) {
    // Add bounds checking
    if (isnan(error) || isnan(error_derivative)) {
        Serial.println("NaN detected in input to calculateInstability");
        return 0.0;
    }
    
    if (STABILITY_EPSILON == 0 || MAX_THRESHOLD_CHANGE == 0) {
        Serial.println("Division by zero prevented in calculateInstability");
        return 0.0;
    }
    
    float term1 = pow(error/STABILITY_EPSILON, 2);
    float term2 = pow(error_derivative/MAX_THRESHOLD_CHANGE, 2);
    
    if (isnan(term1) || isnan(term2)) {
        Serial.println("NaN detected in calculateInstability calculation");
        return 0.0;
    }
    
    return sqrt(term1 + term2);
}

void updateDynamicThreshold() {
    // Calculate pressure error and rate of change
    float pressure_error = P_fuel - P_threshold_down;
    float target_adjustment = 0.0;
    
    // If system pressure is consistently away from threshold, gradually move threshold
    if (abs(pressure_error) > STABILITY_EPSILON) {
        // Calculate desired threshold movement based on current pressure
        target_adjustment = pressure_error * 0.05; // 30% movement toward actual pressure
        
        // Add derivative term to anticipate pressure trends
        target_adjustment += (dP_fuel_dt * DT * 0.35); // 20% weight on trend
        
        // Limit maximum adjustment per cycle
        target_adjustment = constrain(target_adjustment, -MAX_THRESHOLD_CHANGE * DT, MAX_THRESHOLD_CHANGE * DT);
        
        // Update threshold with smoothing
        P_threshold_down = P_threshold_down + target_adjustment;
        
        // Ensure threshold stays within acceptable bounds of base threshold
        P_threshold_down = constrain(P_threshold_down,
                                   P_threshold_base * 0.5,  // Allow more downward movement
                                   P_threshold_base * 1.7); // Allow more upward movement
    }
    
    // Update upstream threshold with wider bounds
    float pressure_error_up = P_fuel - P_threshold_up;
    if (abs(pressure_error_up) > STABILITY_EPSILON * 1.2) {
        float target_adjustment_up = pressure_error_up * 0.25; // 40% movement toward actual pressure
        target_adjustment_up += (dP_fuel_dt * DT * 0.10); // 25% weight on trend
        
        target_adjustment_up = constrain(target_adjustment_up, 
                                       -MAX_THRESHOLD_CHANGE * DT * 1.5,
                                       MAX_THRESHOLD_CHANGE * DT * 1.5);
        
        P_threshold_up = P_threshold_up + target_adjustment_up;
        P_threshold_up = constrain(P_threshold_up,
                                 P_threshold_up_base * 0.6,
                                 P_threshold_up_base * 1.5);
    }
    P_threshold_up_base = P_threshold_up;
    P_threshold_base = P_threshold_down;
    
    // Update current threshold
    P_threshold_current = P_threshold_down;
}

void determineControlState() {
    float instability = calculateInstability(P_fuel - P_threshold_current, dP_fuel_dt);
    
    if (instability < 0.8) {
        current_state = NOMINAL_REGULATION;
    } else if (dP_fuel_dt < -5.0 || instability > 0.8) {
        current_state = RECOVERY_DAMPING;
    } else if (abs(dP_fuel_dt) > 10.0 || instability > 1.5) {
        current_state = HIGH_FREQUENCY_OSCILLATION;
    }
}

// Update the applyOptimalControl function to use both thresholds
void applyOptimalControl() {
    // Calculate pressure errors for both thresholds
    float pressure_error_down = P_threshold_down - P_fuel;
    float pressure_error_up = P_threshold_up - P_fuel;
    
    // Calculate optimal control actions using both errors
    float u_up = -pressure_error_up / (2 * BETA);  // Use upstream error for upstream control
    float u_down = pressure_error_down / (2 * DELTA);  // Use downstream error for downstream control
    
    // Add oscillatory control term with modified behavior
    if (current_state == HIGH_FREQUENCY_OSCILLATION) {
        float oscillation_term = GAMMA * (dP_fuel_dt > 0 ? 1 : -1);
        
        // Apply different oscillation factors based on pressure region
        if (P_fuel > P_threshold_up) {
            u_up += oscillation_term * 1.2;  // Stronger oscillation for high pressure
            u_down += oscillation_term * 0.8;
        } else if (P_fuel < P_threshold_down) {
            u_up += oscillation_term * 0.8;
            u_down += oscillation_term * 1.2;  // Stronger oscillation for low pressure
        } else {
            u_up += oscillation_term;
            u_down += oscillation_term;
        }
    }
    
    // Apply state-based control strategy with enhanced logic
    switch (current_state) {
        case NOMINAL_REGULATION:
            static unsigned long closeStartTime = 0;

            if (P_fuel > P_threshold_up * 1.1) {  // 5% tolerance
                controlUpstreamSolenoid(0);        // Close upstream
                controlDownstreamSolenoid(1); // Normal downstream control
            } else if (P_fuel < P_threshold_down * 0.9) {  // 5% tolerance
                controlUpstreamSolenoid(1.0);     // Open upstream
                controlDownstreamSolenoid(0);     // Close downstream
            } else {
              if (millis() - closeStartTime >= deltaT) {
                    controlUpstreamSolenoid(u_up > 0 ? 1.0 : 0);
                    controlDownstreamSolenoid(u_down > 0 ? 1.0 : 0);
              } else {
                controlUpstreamSolenoid(u_up > 0 ? 1.0 : 0);
                controlDownstreamSolenoid(u_down > 0 ? 1.0 : 0);
              }
            }
            break;
            
        case RECOVERY_DAMPING:
            // Enhanced recovery control
            closeStartTime = 0;

            if (dP_fuel_dt < -10) {  // Fast pressure drop
                controlUpstreamSolenoid(1.0);  // Open upstream
                controlDownstreamSolenoid(0);  // Close downstream
                closeStartTime = millis();     // Store time when closed
            } else {
                // Check if the solenoid has been closed long enough
                if (millis() - closeStartTime >= deltaT) {
                    controlUpstreamSolenoid(u_up > 0 ? 1.0 : 0);
                    controlDownstreamSolenoid(u_down > 0 ? 1.0 : 0);
                } else {
                    controlDownstreamSolenoid(0);  // Keep it closed until deltaT expires
                }
            }
            break;
            
        case HIGH_FREQUENCY_OSCILLATION:
            // Implement bang-bang control
            if (P_fuel > P_threshold_up) {
                controlUpstreamSolenoid(0);
                controlDownstreamSolenoid(1.0);
            } else if (P_fuel < P_threshold_down) {
                controlUpstreamSolenoid(1.0);
                controlDownstreamSolenoid(0);
            } else {
              if (millis() - closeStartTime >= deltaT) {
                    controlUpstreamSolenoid(u_up > 0 ? 1.0 : 0);
                    controlDownstreamSolenoid(u_down > 0 ? 1.0 : 0);
              } else {
                controlUpstreamSolenoid(u_up > 0 ? 1.0 : 0);
                controlDownstreamSolenoid(u_down > 0 ? 1.0 : 0);
              }
            }
            break;
    }
}

bool upstream_solenoid_state = false;
bool downstream_solenoid_state = false;

void controlUpstreamSolenoid(float u_up) {
    upstream_solenoid_state = (u_up > 0);
    digitalWrite(VALVE_UP_PIN, upstream_solenoid_state);
}

// Modify controlDownstreamSolenoid to track state
void controlDownstreamSolenoid(float u_down) {
    downstream_solenoid_state = (u_down > 0);
    digitalWrite(VALVE_DOWN_PIN, downstream_solenoid_state);
}

void setup() {
    Serial.begin(115200);
    
    pinMode(VALVE_UP_PIN, OUTPUT);
    pinMode(VALVE_DOWN_PIN, OUTPUT);
    pinMode(PT_UPSTREAM, INPUT);
    pinMode(PT_TANK, INPUT);
    pinMode(PT_DOWNSTREAM, INPUT);
    pinMode(PT_VENTURI_1, INPUT);
    pinMode(PT_VENTURI_2, INPUT);
    
    // Initialize all valves to closed position
    digitalWrite(VALVE_UP_PIN, LOW);
    //digitalWrite(VALVE_UP_CLOSE_PIN, LOW);
    digitalWrite(VALVE_DOWN_PIN, LOW);
    //digitalWrite(VALVE_DOWN_CLOSE_PIN, LOW);
    
    // Initialize buffers and thresholds
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < FILTER_WINDOW; j++) {
            pressure_buffer[i][j] = 0;
        }
    }
    
    P_threshold_current = P_threshold_base;
    P_threshold_down = P_threshold_base;
}

void loop() {
    if (millis() - last_update >= UPDATE_INTERVAL) {
        // Read and filter all pressure values
        float p_upstream = calculatePressure(analogRead(PT_UPSTREAM), PT_1_A, PT_1_B, PT_1_C, PT_1_D);
        float p_tank = calculatePressure(analogRead(PT_TANK), PT_2_A, PT_2_B, PT_2_C, PT_2_D);
        float p_downstream = calculatePressure(analogRead(PT_DOWNSTREAM), PT_3_A, PT_3_B, PT_3_C, PT_3_D);
        float p_venturi1 = calculatePressure(analogRead(PT_VENTURI_1), PT_4_A, PT_4_B, PT_4_C, PT_4_D);
        float p_venturi2 = calculatePressure(analogRead(PT_VENTURI_2), PT_5_A, PT_5_B, PT_5_C, PT_5_D);
        
        // Apply moving average filter
        P_fuel = updateMovingAverage(p_tank, pressure_buffer[1]);
        float p_upstream_filtered = updateMovingAverage(p_upstream, pressure_buffer[0]);
        float p_downstream_filtered = updateMovingAverage(p_downstream, pressure_buffer[2]);
        float p_venturi1_filtered = updateMovingAverage(p_venturi1, pressure_buffer[3]);
        float p_venturi2_filtered = updateMovingAverage(p_venturi2, pressure_buffer[4]);
        
        // Calculate flow rate using Venturi
        float flow_rate = calculateVenturiFlow(p_venturi1_filtered, p_venturi2_filtered);
        
        // Update system state
        calculatePressureDerivative(P_fuel);
        updateDynamicThreshold();
        determineControlState();
        applyOptimalControl();
        
        // Update buffer index
        buffer_index = (buffer_index + 1) % FILTER_WINDOW;
        
        // Enhanced monitoring output
        Serial.print("Upstream Pressure: ");
        Serial.print(p_upstream_filtered);
        Serial.print(",Downstream Pressure: ");
        Serial.print(p_downstream_filtered);
        Serial.print(",P_fuel: ");
        Serial.print(P_fuel);
        Serial.print(",dP/dt: ");
        Serial.print(dP_fuel_dt);
        Serial.print(",Flow: ");
        Serial.print(flow_rate);
        Serial.print(",State: ");
        Serial.print(stateStrings[current_state]); // Print state name instead of number
        Serial.print(",Threshold: ");
        Serial.print(P_threshold_down);
        Serial.print(",Threshold_Up: ");
        Serial.print(P_threshold_up);
        Serial.print(",Instability: ");
        Serial.print(calculateInstability(P_fuel - P_threshold_current, dP_fuel_dt));
        Serial.print(",Control_Status: Up(");
        Serial.print(upstream_solenoid_state);
        Serial.print("), Down(");
        Serial.print(downstream_solenoid_state);
        Serial.println(")");
        
        last_update = millis();
    }
}