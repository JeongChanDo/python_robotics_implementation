import math
import numpy as np
import matplotlib.pyplot as plt
#system noise Cov EKF simulation

Q = np.diag([
    0.1, #variance of location on x-axis
    0.1, #variance of localtion on y-axis
    np.deg2rad(1.0), #variance of yaw angle
    1.0 # variance of velocity
])**2 #predict state covariance

#measurement noise cov  Observation x,y position covariance
R = np.diag([1.0, 1.0])**2


#Simulation parameter
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)])**2
GPS_NOISE = np.diag([0.5,0.5])**2

#time tick[s]
DT = 0.1
#simlation time[s]
SIM_TIME = 50.0

show_animation = True

def calc_input():
    v = 1.0 #[m/s]
    yawrate = 0.1 #[rad/s]
    u = np.array([[v],[yawrate]])
    return u

def observation(xTrue,xd,u):
    xTrue = motion_model(xTrue, u)

    #measurment noise = add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE@np.random.randn(2,1)
    #control noise = add noise to input
    ud = u + INPUT_NOISE@np.random.randn(2,1)

    #Dead Reckoning = Dead Recoking + control input
    xd = motion_model(xd,ud)

    return xTrue, z, xd, ud

#system modeling
def motion_model(x,u):

    #current val
    F = np.array([[1.0, 0, 0, 0],
        [0, 1.0, 0, 0],
        [0, 0, 1.0, 0],
        [0, 0, 0, 0]
    ])

    #discrete time differantial value
    B = np.array([[DT * math.cos(x[2,0]),0],
        [DT*math.sin(x[2,0]),0],
        [0.0, DT],
        [1.0, 0.0]
    ])

    x = F@x + B@u
    return x

#observation modeling
def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    
    z = H @ x
    '''
    print("h : "+str(H.shape))
    print("x :"+str(x.shape))
    print("z :"+str(z.shape))
'''
    return z

def jacobF(x, u):
    """
    Jacobian of Motion model
    motion model = A*x + Q
    x_{t+1} = x_t+v*dt*cos(yaw) 
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t + omega*dt
    v_{t+1} = v{t}

    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2,0]
    v = u[0,0]
    jF = np.array([
        [1.0, 0.0, -DT*v*math.sin(yaw), DT*math.cos(yaw)], #dx/dx, dx/dy, dx/dyaw, dx/dv
        [0.0, 1.0, DT*v*math.cos(yaw), DT*math.sin(yaw)], #dy/dx, dy/dy, dy/dyaw, dy/dv
        [0.0, 0.0, 1.0, 0.0], #dyaw/dx, dyaw/dy, dyaw/dyaw, dyaw/dv
        [0.0, 0.0, 0.0, 1.0] #...
    ])

    return jF

def jacobH(x):
    #Jacobian of Observation Model
    jH = np.array([
        [1,
         0, 0, 0],
        [0, 1, 0, 0]
    ])
    return jH

def ekf_estimation(xEst,Pest, z, u):
    u = calc_input()

    #1) prediction step
    #predicted x
    xPred = motion_model(xEst,u)
    jF = jacobF(xPred,u)

    #Predicted Error covariance
    PPred = jF@Pest@jF.T +Q

    #2) update step
    jH = jacobH(xPred)

    zPred = observation_model(xPred)
    #differential of real mearsurement and predicted measurement
    y = z-zPred
    #Estimated Error Covariance
    S = jH@PPred@jH.T +R
    #calculate Kalman Gain = Pp*H.T/(H*Pp*H.T)
    K = PPred@jH.T@np.linalg.inv(S)



    #Estimated state xEst = xPred + K*(z-H*xPred)
    xEst = xPred + K@y
    #Estimated Error Covariance = Pp - K*H*Pp
    PEst = (np.eye(len(xEst))- K@jH)@PPred
    return xEst, PEst

def plot_covariance_ellipse(xEst, PEst):
    #Estimated Error Cov of x,y 
    Pxy = PEst[0:2, 0:2]
    # eigen value and vector of cov x,y
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind =1
        smallind = 0

    t = np.arange(0,2 *math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a*math.cos(it) for it in t]
    y = [b*math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind,1],eigvec[bigind,0])
    R = np.array([[math.cos(angle), math.sin(angle)],
        [-math.sin(angle), math.cos(angle)]])
    fx = R@(np.array([x, y]))
    px = np.array(fx[0,:] +xEst[0,0] ).flatten()
    py = np.array(fx[1, :]+xEst[1,0]).flatten()
    plt.plot(px,py,"--r")

def main():
    print(__file__+"  start!!")

    time = 0.0

    #State Vector [x y yaw v]'
    xEst = np.zeros((4,1))
    xTrue = np.zeros((4,1))
    PEst = np.eye(4)

    #Dead reckoning
    xDR = np.zeros((4,1))

    #history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2,1))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        #input xTrue, DeadReckoning, control
        #output : true Pose, noised measuremnt, DeadReckoning, noised control
        xTrue, z, xDR, ud = observation(xTrue, xDR,u)

        #calculate current estimated state/error cov from ref state/error cov using noised measuremnt and input
        xEst, PEst = ekf_estimation(xEst,PEst,z ,ud)

        #store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz,z))

        if show_animation:
            plt.plot(hz[0,:],hz[1,:],".g")
            plt.plot(hxTrue[0,:].flatten(),
                hxTrue[1,:].flatten(),"-b")
            plt.plot(hxDR[0,:].flatten(),
                hxDR[1,:].flatten(),"-k")
            plt.plot(hxEst[0,:].flatten(),
                hxEst[1,:].flatten(),"-r")
            #plot_covariance_ellipse(xEst,PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)
    

if __name__ =="__main__":
    main()