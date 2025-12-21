from notebook_grader import BicycleSolution, grade_bicycle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        
        self.L = 2
        self.lr = 1.2
        self.w_max = 1.22
        
        self.sample_time = 0.01
        
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0


# A sample time is required for numerical integration when propagating the kinematics through time. This is set to 10 milliseconds. We also have a reset function which sets all the state variables back to 0. 


class Bicycle(Bicycle):
    def step(self, v, w):

        dt = self.sample_time
        w = np.clip(w, -self.w_max, self.w_max)
        
        #Integrate the steering angle (delta)
        self.delta = self.delta + (w*dt)
        
        #Compute slip angle beta
        #beta = arctan((lr/L)*tan(delta))
        self.beta = np.arctan((self.lr/self.L)*np.tan(self.delta))
        
        #Bicycle kinematic calculations
        xc_dot = v*np.cos(self.theta+self.beta)
        yc_dot = v*np.sin(self.theta+self.beta)
        theta_dot = (v*np.cos(self.beta)*np.tan(self.delta))/self.L
        
        #Integrate states
        self.xc += xc_dot*dt
        self.yc += yc_dot*dt
        self.theta += theta_dot*dt
        
        pass


sample_time = 0.01
time_end = 20
model = Bicycle()
solution_model = BicycleSolution()

# set delta directly
model.delta = np.arctan(2/10)
solution_model.delta = np.arctan(2/10)

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
x_solution = np.zeros_like(t_data)
y_solution = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(np.pi, 0)
    
    x_solution[i] = solution_model.xc
    y_solution[i] = solution_model.yc
    solution_model.step(np.pi, 0)
    
    #model.beta = 0
    solution_model.beta=0
    
plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.plot(x_solution, y_solution,label='Solution Model')
plt.legend()
plt.show()


# The plot above shows the desired circle of 10m radius. The path is slightly offset which is caused by the sideslip effects due to $\beta$. By forcing $\beta = 0$ through uncommenting the last line in the loop, you can see that the offset disappears and the circle becomes centered at (0,10). 
# 
# However, in practice the steering angle cannot be directly set and must be changed through angular rate inputs $\omega$. The cell below corrects for this and sets angular rate inputs to generate the same circle trajectory. The speed $v$ is still maintained at $\pi$ m/s.


sample_time = 0.01
time_end = 20
model.reset()
solution_model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
x_solution = np.zeros_like(t_data)
y_solution = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    
    if model.delta < np.arctan(2/10):
        model.step(np.pi, model.w_max)
    else:
        model.step(np.pi, 0)
        
    x_solution[i] = solution_model.xc
    y_solution[i] = solution_model.yc
    
    if solution_model.delta < np.arctan(2/10):
        solution_model.step(np.pi, model.w_max)
    else:
        solution_model.step(np.pi, 0)    

plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.plot(x_solution, y_solution,label='Solution Model')
plt.legend()
plt.show()


# Here are some other example trajectories: a square path, a spiral path, and a wave path. Uncomment each section to view.

# In[6]:


sample_time = 0.01
time_end = 60
model.reset()
solution_model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
x_solution = np.zeros_like(t_data)
y_solution = np.zeros_like(t_data)

# maintain velocity at 4 m/s
v_data = np.zeros_like(t_data)
v_data[:] = 4 

w_data = np.zeros_like(t_data)

# ==================================
#  Square Path: set w at corners only
# ==================================
w_data[670:670+100] = 0.753
w_data[670+100:670+100*2] = -0.753
w_data[2210:2210+100] = 0.753
w_data[2210+100:2210+100*2] = -0.753
w_data[3670:3670+100] = 0.753
w_data[3670+100:3670+100*2] = -0.753
w_data[5220:5220+100] = 0.753
w_data[5220+100:5220+100*2] = -0.753

# ==================================
#  Spiral Path: high positive w, then small negative w
# ==================================
# w_data[:] = -1/100
# w_data[0:100] = 1

# ==================================
#  Wave Path: square wave w input
# ==================================
#w_data[:] = 0
#w_data[0:100] = 1
#w_data[100:300] = -1
#w_data[300:500] = 1
#w_data[500:5700] = np.tile(w_data[100:500], 13)
#w_data[5700:] = -1

# ==================================
#  Step through bicycle model
# ==================================
for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(v_data[i], w_data[i])

    x_solution[i] = solution_model.xc
    y_solution[i] = solution_model.yc
    solution_model.step(v_data[i], w_data[i])
    
plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.plot(x_solution, y_solution,label='Solution Model')
plt.legend()
plt.show()



sample_time = 0.01
time_end = 30
model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)

R = 8.0
T = time_end
dt = sample_time

# constant speed for 2 full circles in T seconds
v_const = 4.0 * np.pi * R / T

# --- solve to get delta_star such that curvature = 1/R in the full model ---
L = model.L
lr = model.lr
k_target = 1.0 / R

def curvature(delta):
    beta = np.arctan((lr / L) * np.tan(delta))
    return (np.cos(beta) * np.tan(delta)) / L  # kappa = theta_dot / v

lo, hi = 0.0, 0.8  # radians, safe bracket
for _ in range(60):
    mid = 0.5 * (lo + hi)
    if curvature(mid) < k_target:
        lo = mid
    else:
        hi = mid
delta_star = 0.5 * (lo + hi)

# --- time schedule for figure-8 ---
# 1/4 left circle, 1 full right circle, 3/4 left circle
t1 = T * (0.25 / 2.0)          # = T/8  = 3.75s when T=30
t2 = t1 + T * (1.0 / 2.0)      # +15s  = 18.75s when T=30

for i, t in enumerate(t_data):
    v = v_const

    # desired steering angle by segment
    if t < t1:
        delta_des = +delta_star      # left loop (CCW)
    elif t < t2:
        delta_des = -delta_star      # right loop (CW)
    else:
        delta_des = +delta_star      # finish left loop (CCW)

    # steering-rate command to drive delta -> delta_des, respecting w_max
    w = np.clip((delta_des - model.delta) / dt, -model.w_max, model.w_max)

    v_data[i] = v
    w_data[i] = w

    model.step(v, w)
    x_data[i] = model.xc
    y_data[i] = model.yc

plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()



grade_bicycle(t_data,v_data,w_data)


data = np.vstack([t_data, v_data, w_data]).T
np.savetxt('figure8.txt', data, delimiter=', ')


sample_time = 0.01
time_end = 30
model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)

# ==================================
#  Test various inputs here
# ==================================
for i in range(t_data.shape[0]):

    model.step(v_data[i], w_data[i])
    
plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()





