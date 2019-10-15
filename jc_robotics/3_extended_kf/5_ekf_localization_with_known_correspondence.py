import numpy as np
import random
import matplotlib.pyplot as plt
import math

#######
def jacobian_F(x,u):
    v = u[0,0]
    jF = np.array([
        [1, 0, v * DT *(-1)* np.sin(x[2,0]), DT*np.cos(x[2,0])],
        [0, 1, v * DT * np.cos(x[2,0]), DT*np.sin(x[2,0])],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    return jF

def jacobian_H(x):
    C = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    jH = C

    return jH

def motion_model(x, u):
    yaw = x[2,0]

    A = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 0]
    ])

    B = np.array([
        [np.cos(yaw) * DT, 0],
        [np.sin(yaw) * DT, 0],
        [0, DT],
        [1, 0]
    ])

    x = A@x + B@u

    return x

def observation_model(x):

    C = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = C@x

    return z
#
def ekf_localization(xEst, pEst, z, u):



    xPred = motion_model(xEst,u)

    zPred = observation_model(xPred)

    y = z - zPred

    jF = jacobian_F(xPred,u )
    pPred = jF@pEst@jF.T + R
    jH = jacobian_H(xPred)


    S = jH@pPred@jH.T+Q

    k = pPred@jH.T@np.linalg.inv(S)

    xEst = xPred + k@y

    pEst = (np.eye(4) - k@jH)@pPred

    return xEst, pEst

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def observation(x,u, RFID):
    xTrue = motion_model(x,u)


    z = np.zeros((0,3))
    for i in range(len(RFID)):
        dx = RFID[i,0] - xTrue[0,0]
        dy = RFID[i,1] - xTrue[1,0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx)-xTrue[2,0])
        s = i


        if d<= MAX_RANGE:
            dn = d + np.random.randn() *Q[0,0]
            anglen = angle + np.random.randn() * Q[1,1]
            zi = np.array([dn,anglen,i])
            z = np.vstack((z,zi))


    return xTrue, z


def calc_input():
    v = 0.5
    yaw_rate = 0

    u = np.array([
        [v],
        [yaw_rate]
    ])

    return u
pos_x = 0
pos_y = 0
yaw = 0
v = 0

MAX_RANGE = 100
"""
xInit = np.array([
    [pos_x],
    [pos_y],
    [yaw],
    [v]
])
"""
STATE_SIZE = 4
xTrue = np.zeros((STATE_SIZE,1))
xEst = np.zeros((STATE_SIZE,1))
pEst = np.eye(STATE_SIZE)



GPS_noise = np.diag([0.5, 0.5])
input_noise = np.diag([1, 1])

DT = 0.2

#motion noise cov
R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1])**2
#measurement noise cov
Q = np.array([
    [1 , 0],
    [0, 1]
])**2




SIM_Time = 5
time = 0

histTrue = xTrue
histEst = xEst
show_visualization = True

RFID = np.array([
    [1,0.5]
])



while SIM_Time >= time:
    time +=DT
    u = calc_input()

    xTrue, z = observation(xTrue, u, RFID)
    xEst, pEst = ekf_localization(xEst,pEst,z, u)

    print(z)
    
    histTrue = np.hstack((histTrue,xTrue))
    histEst = np.hstack((histEst,xEst))

    if show_visualization:
        #print(xEst)
        #plt.plot(xEst[0],xEst[1],'.g',label="EstPose")

        #plt.plot(z[0],z[1],'.b',label="measurement")
        plt.plot(histEst[0],histEst[1],'.g',label="EstPose")
        plt.plot(histTrue[0],histTrue[1],'r--',label="TruePose")
        plt.axis('equal')
        plt.pause(0.01)


    print("pEst : ", pEst)

    if SIM_Time >= time:
        plt.clf()



plt.pause(5)
