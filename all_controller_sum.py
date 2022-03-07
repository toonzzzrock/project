import numpy as np
import matplotlib.pyplot as plt
import math
import random
from gekko import GEKKO

#lane path y = ax^2+bx+c
y_eqa = [0.06,1,-1]
a,b,c = y_eqa

# function
vfunc = np.vectorize(lambda t: t.value[0])
f = lambda x: a * x**2 + b * x + c  

#initial state of vehical [x,y.psi]
init_state_val = [10,10,0]

#const
dt = 0.1
time = 30
speed = 10
l = 2
lr = 1
angle_limit = np.pi/3

#plot style
plt_sep_True = False
if plt_sep_True:
    plot_sep = [111,111,111]
else:
    plot_sep = [131,132,133] 

def cal_x_0(x_c, y_c):
    #newton-raphson method
    k = c - y_c
    x = x_c
    for i in range (30):
        t = a * x
        x = (t * x * (4 * t + 3 * b) - b * k + x_c) / (6 * t * (t + b) + b**2 + 2 * a * k + 1)
    return x

def lane_cal(x_c, y_c, psi):
    #error point in road
    x_r = cal_x_0(x_c, y_c)
    y_r = a * x_r**2 + b * x_r + c
    
    dy_dx = 2 * a * x_r + b
    
    # heading error
    epsi = math.atan(dy_dx) - psi
    
    #cross-track error vector size
    CTE = abs(x_r-x_c)*math.sqrt(1+1/dy_dx**2)
    
    #left-right
    car_to_point = math.cos(psi) * (y_r - y_c) - math.sin(psi) * (x_r - x_c)
        #car_to_point = math.asin((math.cos(psi)*(y_r-y_c)-math.sin(psi)*(x_r-x_c))/1000)
    CTE *= car_to_point / abs(car_to_point)
    
    return CTE, epsi, x_r, y_r

def moving(steel, speed, init, noise = True):
    if noise:
        steel += random.choice([1,-1]) * random.uniform(steel/8, steel/3)
    beta = math.atan(lr * math.tan(steel)/ l)
    x = init[0] + math.cos(init[2] + beta) * speed * dt
    y = init[1] + math.sin(init[2] + beta) * speed * dt
    psi = (init[2] + speed / l * math.cos(beta) * math.tan(steel) * dt) % (2*math.pi)
    
    return [x, y, psi]

def plot(init_state,x_r,y_r,subplot):
    plt.subplot(subplot)
    
    #lane
    x = np.linspace(min(min(init_state[:,0]),x_r), max(max(init_state[:,0]),x_r), 100)
    y = a*x**2 + b*x + c  
    plt.plot(x, y, color = 'b' )
    
    #path
    plt.plot(init_state[:i+2,0],init_state[:i+2,1], color = 'orange')
    
    #error
    plt.plot([init_state[i,0],x_r],[init_state[i,1],y_r], color = 'g')
    
class PID():
    def __init__(self, Kp, Ki, Kd):
        self.error_state_PID = 0
        self.integral_term = 0
        self.temp_error = 0
        self.init_state_PID = np.array([init_state_val for i in range(time)], float)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def controller(self):
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_PID[i, :3])
        self.error_state_PID += CTE**2 + epsi**2
        
        self.integral_term += CTE
        
        #PID_cal
        P = CTE * self.Kp
        I = self.integral_term * self.Ki * dt
        D = (CTE - self.temp_error) * self.Kd / dt
        steel_angle = P + I + D
        
        #angle limit
        if steel_angle > angle_limit:
            steel_angle = angle_limit
        elif steel_angle < -angle_limit:
            steel_angle = -angle_limit
            
        self.temp_error = CTE
        
        return steel_angle, x_r, y_r

class Stanley():
    def __init__(self, K):
        self.error_state_Stanley = 0
        self.init_state_Stanley = np.array([init_state_val for i in range(time)], float)
        self.K = K
        
    def controller(self):
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_Stanley[i, :3])
        self.error_state_Stanley += CTE**2 + epsi**2
        
        #Stanley_cal
        steel_angle = epsi + math.atan(self.K * CTE / speed)
        
        #angle_limit
        if steel_angle > angle_limit:
            steel_angle = angle_limit
        elif steel_angle < -angle_limit:
            steel_angle = -angle_limit
        
        return steel_angle, x_r, y_r
    
class MPC():
    def __init__(self, N):
        self.error_state_MPC = 0
        self.init_state_MPC = np.array([init_state_val for i in range(time)], float)
        self.N = N
        self.cte_weight = 1
        self.epsi_weight = 10
        self.subplot = plot_sep[2]
        
    def controller(self):
        
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_MPC[i, :3])
        self.error_state_MPC += CTE**2 + epsi**2
        
        #setup model
        m = GEKKO(remote=False)
        m.options.DIAGLEVEL = 0
        
        def m_lane_cal(x_c,psi,x_r):
            dy = 2 * a * x_r + b
            epsi = m.atan(dy) - psi
            cte_2 = (x_r - x_c)**2 * (1+1/dy**2)
            return cte_2, epsi
        
        #init stage
        x0 = self.init_state_MPC[i, :]
        
        # Variable
        # state array [x,y,psi]
        x_r = m.Array(m.Var,(self.N))
        x = m.Array(m.Var,(self.N+1, 3))
        # delta output state array [delta]
        u = m.Array(m.Var,(self.N))
        
        for r in range(self.N):
            x_r[r].value = 20
         
        #fix first stage
        [m.fix(x[0][idx], val = val)for idx, val in enumerate(x0)]
        
        # MPC controller
        for t in range(self.N):
            m.Equations([x[t + 1][0] == x[t][0] + speed * m.cos(x[t][2] + m.atan(lr / l * m.tan(u[t]))) * dt, #x
                        x[t + 1][1] == x[t][1] + speed * m.sin(x[t][2] + m.atan(lr / l * m.tan(u[t]))) * dt, #y 
                        x[t + 1][2] == x[t][2] + speed * m.cos(m.atan(lr / l * m.tan(u[t]))) * m.tan(u[t]) / l * dt #psi
                        #u[t] delta
                        ])
            
            u[t].upper = angle_limit
            u[t].lower = -angle_limit
            
            m.Equation(2 * a**2 * x_r[t]**3 + 3 * a * b * x_r[t] ** 2 + (b**2 + 2 * a * (c - x[t][1]) + 1) * x_r[t] + b * c - b * x[t][1] - x[t][0] == 0 )
        
        #error J cost function
        for ii in range(self.N):
            cte_2, epsi = m_lane_cal(x[ii,0], x[ii,2], x_r[ii])
            m.Obj(self.cte_weight * cte_2 + self.epsi_weight * epsi**2)
        
        #solve
        m.options.IMODE = 3
        m.solve()
        
        #change to value 
        stage = vfunc(x)
        x_r = vfunc(x_r)
        u = vfunc(u)
        #next stage
        self.init_state_MPC[i+1] = moving(u[0], speed, self.init_state_MPC[i])
        
        #MPC plot
        self.plot(stage,x_r)
        
    def plot(self, stage, x_r, cte_plot = True):
        plt.subplot(self.subplot)
        
        x = np.linspace(min([*self.init_state_MPC[:,0] , *stage[:,0]]), max([*self.init_state_MPC[:,0] , *stage[:,0]]), 100)
        y = f(x)  
        plt.plot(x, y, color = 'b')
        
        if cte_plot:
            y_r = f(x_r)
            [plt.plot([x_r[ii],stage[ii,0]], [y_r[ii],stage[ii,1]], color = 'g') for ii in range(self.N)]
            
        plt.plot(self.init_state_MPC[:i+2,0], self.init_state_MPC[:i+2,1], color = 'orange')#all state
        plt.scatter(stage[:,0], stage[:,1], s=3, c = 'k')#predict
        
        plt.legend(fontsize=10)
        plt.legend(['lane', 'real state', 'predicted'])
        ax = plt.gca()
        leg = ax.get_legend()
        leg.legendHandles[1].set_color('orange')
        leg.legendHandles[2].set_color('black')
        plt.title('MPC')
        
def controller_All():
    #figure
    plt.figure(figsize=(1, 3), dpi=200)
    plt.rcParams.update({'font.size': 5})
    
    #PID
    steel_angle, x_r, y_r = PID_control.controller()
    PID_control.init_state_PID[i+1] = moving(steel_angle, speed, PID_control.init_state_PID[i])
    
    #PID_plot
    plot(PID_control.init_state_PID, x_r, y_r, plot_sep[0])
    plt.title('PID')
    
    #Stanley
    steel_angle, x_r, y_r = Stanley_control.controller()
    Stanley_control.init_state_Stanley[i+1] = moving(steel_angle, speed, Stanley_control.init_state_Stanley[i])
    
    #Stanley_plot
    plot(Stanley_control.init_state_Stanley, x_r, y_r, plot_sep[1])
    plt.title('Stanley')

    #MPC
    MPC_control.controller()
    
    #plot style
    if plt_sep_True:
        plt.rcParams.update({'font.size': 8})
        p = lambda lis, name : plt.text(lis[i+1][0], lis[i+1][1], name)
        p(PID_control.init_state_PID,'PID')
        p(Stanley_control.init_state_Stanley,'Stanley')
        p(MPC_control.init_state_MPC,'MPC')
        plt.title('All')
        
    #plot adjustment
    left  = 0.7  # the left side of the subplots of the figure
    right = 5    # the right side of the subplots of the figure
    bottom = 0.1   # the bottom of the subplots of the figure
    top = 0.9      # the top of the subplots of the figure
    wspace = 0.4   # the amount of width reserved for blank space between subplots
    hspace = 0.2   # the amount of height reserved for white space between subplots
    plt.subplots_adjust(left=left, bottom=bottom, right=right, top=top, wspace=wspace, hspace=hspace)
    plt.show()    
        
#Gain
PID_K = [1, 0.2, 0.1]
Stanley_K = 1.1
MPC_N = 20
#controller setup
PID_control = PID(*PID_K)
Stanley_control = Stanley(Stanley_K)
MPC_control = MPC(MPC_N)
#run
for i in range(time-1):
    controller_All()
print(f'error report\nPID_control : {PID_control.error_state_PID/i}\nStanley_control : {Stanley_control.error_state_Stanley/i}\nMPC_control {MPC_control.error_state_MPC/i}\n')


'''
# Stanley controller

list_K = np.linspace(1, 10, 300)
for K in list_K:
    try:
        init_state = np.array([[0,0,0,0] for i in range(time)], float)    
        error_state = 0
        controller(K)
        error_state = error_state/time
        
        x = np.linspace(min(init_state[:,0]), max(init_state[:,0]), 100)
        y = a*x**2 + b*x + c  
        plt.title(f'min {K} with:{error_state}')
        plt.plot(init_state[:,0],init_state[:,1])
        plt.plot(x, y)
        plt.legend(['state','lane'])
        plt.axis('scaled')
        plt.show()
        
    except ValueError:
        print('error')
        def f(x_list):
            y_list = []
            for x in x_list:
                y_list.append(2*a**2*x**3+3*a*b*x**2+(b**2+2*a*(c-x_c)+1)*x+b*c-b*y_c-x_c)
            return y_list
        x_c,y_c,_,_ = init_state[i]
        k = c-y_c
        x = x_c
        list_x = []
        for ii in range (100):
            t = a*x
            x = (t*x*(4*t + 3*b) - b*k + x_c)/(6*t*(t + b) + b**2 + 2*a*k + 1)
            
            if ii%5:
                list_x.append(x)
                plt.plot(list_x, f(list_x), '.r')
                plt.text(x, f([x]), str(ii))
                x_plot = np.linspace(min(list_x), max(list_x), 200)
                plt.plot(x_plot,f(x_plot))
                plt.show()
                plt.pause(0.2)'''
        