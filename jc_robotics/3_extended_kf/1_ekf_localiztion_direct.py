import numpy as np
import random
import matplotlib.pyplot as plt

#####
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

def observation(x,u):
    xTrue = motion_model(x,u)
    z = observation_model(xTrue) + GPS_noise@np.random.randn(2,1)

    return xTrue, z

pos_x = 0
pos_y = 0
yaw = 0
v = 0


xInit = np.array([
    [pos_x],
    [pos_y],
    [yaw],
    [v]
])

pInit = np.eye(4)

v = 1
yaw_rate = 0.1

u = np.array([
    [v],
    [yaw_rate]
])

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

xTrue = xInit


xEst = xInit
pEst = pInit


SIM_Time = 50
time = 0

histTrue = xTrue
histEst = xEst
histz = np.zeros((2,1))


while SIM_Time >= time:
    time +=DT

    xTrue, z = observation(xTrue, u)

    xEst, pEst = ekf_localization(xEst,pEst,z, u)
    #print("xTrue : " ,xTrue)
    print("xEst : ")
    print(xEst[:2,:])
    print("-------------------")
    #print("pEst : ", pEst)

    histTrue = np.hstack((histTrue,xTrue))
    histEst = np.hstack((histEst,xEst))
    histz = np.hstack((histz,z))
