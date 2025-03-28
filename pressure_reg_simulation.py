import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

class PressureControlSystem:
    def __init__(self):
        # System constants
        self.upstream_pressure = 1000  # psi
        self.initial_pressure = 100  # psi
        self.tank_volume = 5.5  # L
        self.initial_ullage_volume = 0.4  # L
        self.initial_liquid_volume = self.tank_volume - self.initial_ullage_volume
        self.max_flow_rate = 1.2  # kg/s
        self.liquid_density = 1000  # kg/m^3
        
        # Dynamic variables
        self.current_pressure = self.initial_pressure
        self.current_upstream_pressure = self.upstream_pressure
        self.system_active = False
        self.current_time = 0.0
        self.activation_time = 5.0
        self.current_liquid_volume = self.initial_liquid_volume
        self.current_ullage_volume = self.initial_ullage_volume
        self.total_flow = 0.0
        self.valve_state = False
        self.system_empty = False  # New flag to track system emptying
        
        # Control parameters
        self.target_pressure = 100  # psi
        self.min_pressure = 90  # psi
        self.pressure_error_threshold = 10  # psi
        self.valve_on_times = []
        self.valve_off_times = []
        
        # Histories
        self.pressure_history = deque([self.current_pressure], maxlen=1500)
        self.time_history = deque([0.0], maxlen=1500)
        self.flow_rate_history = deque([0.0], maxlen=1500)
        self.upstream_pressure_history = deque([self.upstream_pressure], maxlen=1500)
        self.objective_loss_history = deque([0.0], maxlen=1500)
        self.target_history = deque([self.target_pressure], maxlen=1500)
        
    def update_upstream_pressure(self):
        if self.valve_state:
            noise = np.random.normal(0, 5)
            dynamic_drop = 20
        else:
            noise = np.random.normal(0, 2)
            dynamic_drop = 0
        self.current_upstream_pressure = self.upstream_pressure + noise - dynamic_drop
        
    def calculate_flow_rate(self):
        if not self.system_active or self.system_empty:
            return 0.0
        
        # Modified flow calculation with stronger dependency on liquid volume
        if self.current_liquid_volume <= 0.1:
            self.system_empty = True  # Set empty flag when liquid is depleted
            return 0.0
            
        base_flow = 0.8  # Nominal flow rate
        noise = np.random.normal(0, 0.02)
        
        # More aggressive flow reduction as liquid depletes
        depletion_factor = (self.current_liquid_volume / self.initial_liquid_volume) ** 1.5
        base_flow *= depletion_factor
            
        return max(0, min(base_flow + noise, self.max_flow_rate))
        
    def calculate_pressure_dynamics(self, dt):
        if not self.system_active:
            return self.current_pressure
            
        # Modified pressure calculation using proper gas law relationship
        # P₁V₁ = P₂V₂ relationship for ullage space
        base_pressure = (self.initial_pressure * self.initial_ullage_volume) / self.current_ullage_volume
        
        # Add pressurant effects when valve is open
        if self.valve_state and not self.system_empty:
            # Reduced pressure increase effect at larger volumes
            volume_factor = (self.initial_ullage_volume / self.current_ullage_volume) ** 0.5
            pressure_increase = (self.current_upstream_pressure - self.current_pressure) * 0.05 * volume_factor
            base_pressure += pressure_increase
        
        # Natural pressure decay due to heat loss and other factors
        pressure_decay = 0.01 * self.current_pressure * dt
        base_pressure -= pressure_decay
            
        return max(self.min_pressure, base_pressure)
        
    def update_target_pressure(self):
        if self.system_empty:
            self.target_pressure = self.min_pressure
            return
            
        pressure_error = abs(self.current_pressure - self.target_pressure)
        if pressure_error > self.pressure_error_threshold:
            self.target_pressure = max(self.min_pressure, 
                                     self.current_pressure + 20)
            
    def calculate_objective_loss(self):
        pressure_error = abs(self.current_pressure - self.target_pressure)
        pressurant_usage = len(self.valve_on_times) * 100
        time_factor = self.current_time * 0.1
        return pressure_error**2 + pressurant_usage + time_factor
        
    def update_valve_state(self):
        if self.system_empty:
            self.valve_state = False
            return
            
        old_state = self.valve_state
        
        if self.current_pressure < self.min_pressure:
            self.valve_state = True
        elif self.current_pressure > self.target_pressure:
            self.valve_state = False
            
        if self.valve_state != old_state:
            if self.valve_state:
                self.valve_on_times.append(self.current_time)
            else:
                self.valve_off_times.append(self.current_time)
                
    def update(self, dt):
        self.current_time += dt
        
        if self.current_time >= self.activation_time and not self.system_active:
            self.system_active = True
        
        if self.system_active:
            # Calculate flow before pressure update
            flow_rate = self.calculate_flow_rate()
            volume_outflow = flow_rate * dt
            
            # Update volumes
            self.current_liquid_volume = max(0, self.current_liquid_volume - volume_outflow)
            self.current_ullage_volume = self.tank_volume - self.current_liquid_volume
            self.total_flow += flow_rate * dt
            print(self.current_liquid_volume)
            # Update valve state and upstream pressure
            self.update_valve_state()
            self.update_upstream_pressure()
            
            # Update pressure
            self.current_pressure = self.calculate_pressure_dynamics(dt)
            
            # Update target pressure based on objective function
            self.update_target_pressure()
        else:
            flow_rate = 0
            
        # Calculate objective function loss
        objective_loss = self.calculate_objective_loss()
        
        # Store history
        self.pressure_history.append(self.current_pressure)
        self.time_history.append(self.current_time)
        self.flow_rate_history.append(flow_rate)
        self.upstream_pressure_history.append(self.current_upstream_pressure)
        self.objective_loss_history.append(objective_loss)
        self.target_history.append(self.target_pressure)
        
        return (self.current_pressure, flow_rate, self.current_upstream_pressure, objective_loss)

# The plotting code remains the same as in the original

# Plotting code
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
fig.tight_layout(pad=3.0)
system = PressureControlSystem()

# Initialize lines
line_pressure, = ax1.plot([], [], 'b-', label='Tank Pressure', linewidth=1.5)
line_target, = ax1.plot([], [], 'r--', label='Target Pressure', linewidth=1.5)
line_flow, = ax2.plot([], [], 'b-', label='Flow Rate', linewidth=1.5)
line_upstream, = ax3.plot([], [], 'g-', label='Upstream Pressure', linewidth=1.5)
line_loss, = ax3.plot([], [], 'r-', label='Objective Loss', linewidth=1.5)

def init():
    ax1.set_ylim(0, 600)
    ax1.set_xlim(0, 100)  # Shorter timeframe to match real data
    ax2.set_ylim(0, 1.0)
    ax2.set_xlim(0, 100)
    ax3.set_ylim(900, 1100)
    ax3.set_xlim(0, 100)
    return line_pressure, line_target, line_flow, line_upstream, line_loss

def update(frame):
    pressure, flow_rate, upstream_pressure, objective_loss = system.update(0.1)
    
    # Update data
    line_pressure.set_data(list(system.time_history), list(system.pressure_history))
    line_target.set_data(list(system.time_history), list(system.target_history))
    line_flow.set_data(list(system.time_history), list(system.flow_rate_history))
    line_upstream.set_data(list(system.time_history), list(system.upstream_pressure_history))
    line_loss.set_data(list(system.time_history), list(system.objective_loss_history))
    
    # Add valve state indicators
    for t in system.valve_on_times:
        if t <= system.current_time:
            ax1.axvline(x=t, color='r', alpha=0.3, linewidth=1)
            ax2.axvline(x=t, color='r', alpha=0.3, linewidth=1)
            ax3.axvline(x=t, color='r', alpha=0.3, linewidth=1)
    
    for t in system.valve_off_times:
        if t <= system.current_time:
            ax1.axvline(x=t, color='k', alpha=0.3, linewidth=1)
            ax2.axvline(x=t, color='k', alpha=0.3, linewidth=1)
            ax3.axvline(x=t, color='k', alpha=0.3, linewidth=1)
    
    return line_pressure, line_target, line_flow, line_upstream, line_loss

ani = FuncAnimation(fig, update, init_func=init, interval=50, blit=True)
plt.show()
