import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0



class Vehicle(Vehicle):
    def step(self, throttle, alpha):
        # ==================================
        #  Implement vehicle model here
        # ==================================
        dt = self.sample_time
        
        # 1) Integrate states forward (Euler) using previous derivatives
        self.x   = self.x   + self.v * dt
        self.v   = self.v   + self.a * dt
        self.w_e = self.w_e + self.w_e_dot * dt
        
        # Keep forward-motion assumption consistent
        if self.v < 0:
            self.v = 0.0
            
        # Engine torque: Te = throttle * (a0 + a1*w_e + a2*w_e^2)
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * (self.w_e ** 2))
        
        # Wheel speed: w_w = (GR) * w_e
        w_w = self.GR * self.w_e
        
        # Slip ratio: s = (w_w * r_e - v) / v   (handle v ~ 0 safely)
        if self.v > 1e-6:
            s = (w_w * self.r_e - self.v) / self.v
        else:
            # If basically stopped, define slip based on whether wheel is trying to spin
            s = 0.0
        
        # Tire force: Fx = c*s if |s|<1 else F_max*sign(s)
        if abs(s) < 1.0:
            F_x = self.c * s
        else:
            F_x = self.F_max * np.sign(s)
        
        # Load forces
        # Aero: F_aero = c_a * v^2
        F_aero = self.c_a * (self.v ** 2)
        
        # Rolling friction (simplified linear): R_x = c_r1 * v
        R_x = self.c_r1 * self.v
        
        # Grade: F_g = m*g*sin(alpha)
        F_g = self.m * self.g * np.sin(alpha)
        
        # Total load
        F_load = F_aero + R_x + F_g
        
        # 3) Combined engine + longitudinal dynamics
        # Engine dynamics: J_e * w_e_dot = T_e - (GR)*(r_e * F_load)
        self.w_e_dot = (T_e - (self.GR * (self.r_e * F_load))) / self.J_e

        # Longitudinal dynamics: m * x_ddot = F_x - F_load
        self.a = (F_x - F_load) / self.m


# Using the model, you can send constant throttle inputs to the vehicle in the cell below. You will observe that the velocity converges to a fixed value based on the throttle input due to the aerodynamic drag and tire force limit. A similar velocity profile can be seen by setting a negative incline angle $\alpha$. In this case, gravity accelerates the vehicle to a terminal velocity where it is balanced by the drag force.

sample_time = 0.01
time_end = 100
model = Vehicle()

t_data = np.arange(0,time_end,sample_time)
v_data = np.zeros_like(t_data)

# throttle percentage between 0 and 1
throttle = 0.2

# incline angle (in radians)
alpha = 0

for i in range(t_data.shape[0]):
    v_data[i] = model.v
    model.step(throttle, alpha)
    
plt.plot(t_data, v_data)
plt.show()


time_end = 20
t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)

# reset the states
model.reset()

# Ramp geometry (piecewise constant slope angles)
alpha1 = np.arctan(3.0/60.0)     # 0 -> 60 m rises 3 m
alpha2 = np.arctan(9.0/90.0)     # 60 -> 150 m rises additional 9 m (to 12 m total)

for i, t in enumerate(t_data):

    # --------------------------
    # Throttle profile x_theta(t)
    # --------------------------
    if t < 5.0:
        # ramp 0.2 -> 0.5 over 5s
        throttle = 0.2 + (0.5 - 0.2) * (t / 5.0)
    elif t < 15.0:
        # hold
        throttle = 0.5
    else:
        # ramp 0.5 -> 0 over 5s (15 to 20)
        throttle = 0.5 * (1.0 - (t - 15.0) / 5.0)
        throttle = max(throttle, 0.0)

    # --------------------------
    # Ramp angle profile alpha(x)
    # --------------------------
    x = model.x
    if x < 60.0:
        alpha = alpha1
    elif x < 150.0:
        alpha = alpha2
    else:
        alpha = 0.0

    # Step the model and log position
    model.step(throttle, alpha)
    x_data[i] = model.x


# Plot x vs t for visualization
plt.plot(t_data, x_data)
plt.show()



data = np.vstack([t_data, x_data]).T
np.savetxt('xdata.txt', data, delimiter=', ')




sample_time = 0.01
time_end = 30
model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)

# ==================================
#  Test various inputs here
# ==================================
for i in range(t_data.shape[0]):

    model.step(0,0)
    
plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()

