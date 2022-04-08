import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time
from gekko import GEKKO
import evolution_Stanley as Stanley_evo
import evolution as PID_evo
import pickle

# processing before run first step
#lane path y = ax^2+bx+c
y_eqa = [0.06,1,-1]
a,b,c = y_eqa
epsi_weight = 3
# function
vfunc = np.vectorize(lambda t: t.value[0])
f = lambda x: a * x**2 + b * x + c  

#initial state of vehical [x,y.psi]
init_state_val = [20,25,0]

#const
dt = 0.1
time_step = 75
speed = 3
l = 1
lr = 0.5
angle_limit = np.pi/3

#plot style
plt_sep_True = False

def cal_x_0(x_c, y_c):
    #newton-raphson method
    k = c - y_c
    x = x_c
    for i in range (6):
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

def moving(steel, speed, init, dt_real, noise = False, bias = True):
    if bias:
        dt_real+= 0.2
    if noise:
        steel += random.choice([1,-1]) * random.uniform(steel/8, steel/6)
        
    beta = math.atan(lr * math.tan(steel)/ l)
    x = init[0] + math.cos(init[2] + beta) * speed * dt_real
    y = init[1] + math.sin(init[2] + beta) * speed * dt_real
    psi = (init[2] + speed / l * math.cos(beta) * math.tan(steel) * dt_real) % (2*math.pi)
    
    return [x, y, psi]

def plot(init_state,x_r,y_r, ax, color):
    
    #lane
    x = np.linspace(min(min(init_state[:,0]),x_r), max(max(init_state[:,0]),x_r), 100)
    y = a*x**2 + b*x + c  
    ax.plot(x, y, color = 'b' )
    
    #path
    ax.plot(init_state[:i+2,0],init_state[:i+2,1], color = color)
    
    #error
    ax.plot([init_state[i,0],x_r],[init_state[i,1],y_r], color = 'g')
    
class PID():
    def __init__(self, Kp, Ki, Kd):
        self.error_state_PID = np.zeros(time_step)
        self.integral_term = 0
        self.temp_error = 0
        self.init_state_PID = np.array([init_state_val for i in range(time_step)], float)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.PID_dt_real = []
    def controller(self):
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_PID[i, :3])
        self.error_state_PID[i] = CTE**2 + epsi_weight * epsi**2
        
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
        self.error_state_Stanley = np.zeros(time_step)
        self.init_state_Stanley = np.array([init_state_val for i in range(time_step)], float)
        self.K = K
        self.Stanley_dt_real = []
    def controller(self):
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_Stanley[i, :3])
        self.error_state_Stanley[i] = CTE**2 + epsi_weight * epsi**2
        
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
        self.error_state_MPC = np.zeros(time_step)
        self.init_state_MPC = np.array([init_state_val for i in range(time_step)], float)
        self.N = N
        self.cte_weight = 1
        self.epsi_weight = epsi_weight
        self.MPC_dt = 0.3
        self.MPC_dt_real = []
    def controller(self):
        
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_MPC[i, :3])
        self.error_state_MPC[i] = CTE**2 + epsi_weight * epsi**2
        
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
            x_r[r].value = 50
         
        #fix first stage
        [m.fix(x[0][idx], val = val)for idx, val in enumerate(x0)]
       
        # MPC controller
        for t in range(self.N):
            m.Equations([x[t + 1][0] == x[t][0] + speed * m.cos(x[t][2] + m.atan(lr / l * m.tan(u[t]))) * self.MPC_dt, #x
                        x[t + 1][1] == x[t][1] + speed * m.sin(x[t][2] + m.atan(lr / l * m.tan(u[t]))) * self.MPC_dt, #y 
                        x[t + 1][2] == x[t][2] + speed * m.cos(m.atan(lr / l * m.tan(u[t]))) * m.tan(u[t]) / l * self.MPC_dt #psi
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
        m.solve(disp=False)
        #change to value 
        self.stage = vfunc(x)
        self.x_r = vfunc(x_r)
        self.u = vfunc(u)
        m.cleanup()
       
        
    def plot(self, stage, x_r, ax, cte_plot = True):
        
        x = np.linspace(min([*self.init_state_MPC[:,0] , *stage[:,0]]), max([*self.init_state_MPC[:,0] , *stage[:,0]]), 100)
        y = f(x)  
        ax.plot(x, y, color = 'b')
        
        if cte_plot:
            y_r = f(x_r)
            [ax.plot([x_r[ii],stage[ii,0]], [y_r[ii],stage[ii,1]], color = 'g') for ii in range(self.N)]
            
        ax.plot(self.init_state_MPC[:i+2,0], self.init_state_MPC[:i+2,1], color = 'orange')#all state
        ax.scatter(stage[:,0], stage[:,1], s=3, c = 'k')#predict
        
        
        
def controller_All():
    #figure
    
    if plt_sep_True:
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        plot_sep = [ax1, ax2, ax3]
    else:
        fig, ax = plt.subplots(1, 1)
        plot_sep = [ax for i in range(3)]
    plt.rcParams.update({'font.size': 5})
    
    start = time.time()
    #PID
    PID_steel_angle, PID_x_r, PID_y_r = PID_control.controller()
    end = time.time()
    PID_control.PID_dt_real.append(end - start)
    
    start = time.time()
    #Stanley
    Stanley_steel_angle, x_r, y_r = Stanley_control.controller()
    end = time.time()
    Stanley_control.Stanley_dt_real.append(end - start)
    
    start = time.time()
    #MPC
    MPC_control.controller()
    end = time.time()
    MPC_control.MPC_dt_real.append(end - start)
    
    #moving
    PID_control.init_state_PID[i+1] = moving(PID_steel_angle, speed, PID_control.init_state_PID[i], PID_control.PID_dt_real[-1])
    Stanley_control.init_state_Stanley[i+1] = moving(Stanley_steel_angle, speed, Stanley_control.init_state_Stanley[i], Stanley_control.Stanley_dt_real[-1])
    MPC_control.init_state_MPC[i+1] = moving(MPC_control.u[0], speed, MPC_control.init_state_MPC[i], MPC_control.MPC_dt_real[-1])
    
    #PID_plot
    plot(PID_control.init_state_PID, PID_x_r, PID_y_r, plot_sep[0], 'r')
    plt.title('PID')
    
    #Stanley_plot
    plot(Stanley_control.init_state_Stanley, x_r, y_r, plot_sep[1], 'm')
    plt.title('Stanley')
    
    #MPC plot
    MPC_control.plot(MPC_control.stage, MPC_control.x_r, plot_sep[2])
        
    #plot style
    if not plt_sep_True:
        plt.rcParams.update({'font.size': 8})
        p = lambda lis, name : plt.text(lis[i+1][0], lis[i+1][1], name)
        p(PID_control.init_state_PID,'PID')
        p(Stanley_control.init_state_Stanley,'Stanley')
        p(MPC_control.init_state_MPC,'MPC')
        plt.title('All')
        
        leg = plt.legend(['lane', 'PID', 'CTE', 'Stanley', 'MPC'])
        colors=['b', 'r', 'green', 'm', 'orange']
        [j.set_color(colors[i]) for i, j in enumerate(leg.legendHandles)]
            
        
    #plot adjustment
    left  = 0.3  # the left side of the subplots of the figure
    right = 1   # the right side of the subplots of the figure
    bottom = 0.1   # the bottom of the subplots of the figure
    top = 0.9      # the top of the subplots of the figure
    wspace = 0.4   # the amount of width reserved for blank space between subplots
    hspace = 0.2   # the amount of height reserved for white space between subplots
    plt.subplots_adjust(left=left, bottom=bottom, right=right, top=top, wspace=wspace, hspace=hspace)
    #plt.axes().set_facecolor("#3A3B3C")
    #plt.axes().set_aspect('equal')
    plt.show()

def get_gain():
    PID_K = PID_evo.gain(speed)
    Stanley_K = Stanley_evo.gain(speed)
    K_dict = {0:PID_K, 1:Stanley_K}
    pickle_out = open("gain.pickle","wb")
    pickle.dump(K_dict, pickle_out)
    pickle_out.close()
    return PID_K, Stanley_K

def main():            
    #Gain
    global PID_control, Stanley_control, MPC_control, i
    
    try:
        inp = 'n'
        if inp == 'Y' or inp == 'y':
            PID_K, Stanley_K = get_gain()
        else:
            pickle_in = open("gain.pickle","rb")
            K_dict = pickle.load(pickle_in)
            pickle_in.close()
            PID_K = K_dict[0]
            Stanley_K = K_dict[1]
    
    except:
        PID_K, Stanley_K = get_gain()
    
    finally:
        PID_K = [0.5929007106210498, 0.009036181600267633, 0.3714275114980786]
        
        Stanley_K = 1.89
        print("PID gain: ", PID_K, "\nStanley gain: ", Stanley_K)
        MPC_N = 5
        #controller setup
        PID_control = PID(*PID_K)
        Stanley_control = Stanley(Stanley_K)
        MPC_control = MPC(MPC_N)
        
        #run
        
        for i in range(time_step-1):
            controller_All()
            
        plt.plot(PID_control.error_state_PID)
        plt.plot(Stanley_control.error_state_Stanley)
        plt.plot(MPC_control.error_state_MPC)
        
        plt.legend(['PID', 'Stanley', 'MPC'])
        plt.show()
        
        plt.plot(PID_control.PID_dt_real)
        plt.plot(Stanley_control.Stanley_dt_real)
        plt.plot(MPC_control.MPC_dt_real)
        
        plt.legend(['PID', 'Stanley', 'MPC'])
        plt.show()
        
        plt.plot(PID_control.PID_dt_real)
        plt.plot(Stanley_control.Stanley_dt_real)
        
        plt.legend(['PID', 'Stanley'])
        plt.show()
        
        print(f'error report\nPID_control : {sum(PID_control.error_state_PID)/time_step}\nStanley_control : {sum(Stanley_control.error_state_Stanley)/time_step}\nMPC_control {sum(MPC_control.error_state_MPC)/time_step}\n')

if __name__ == "__main__":
    main()
