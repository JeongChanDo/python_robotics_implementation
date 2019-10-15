import numpy as np
import random
import matplotlib.pyplot as plt
import math


STATE_SIZE = 4 #[x, y, yaw, v]
LM_SIZE = 3 #[x,y,s]
MAX_RANGE = 100
DT = 0.2
SIM_TIME = 5
time = 0
show_visualization = True

#motion noise cov
R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1])**2
#measurement noise cov
Q = np.array([
    [1 , 0],
    [0, 1]
])**2



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
    v = 1
    yaw_rate = 0.1

    u = np.array([
        [v],
        [yaw_rate]
    ])

    return u


def plot_line(x,RFID):
    line_x = np.linspace(x[0,0], RFID[0,0],30)
    line_y = np.linspace(x[1,0], RFID[0,1],30)

    plt.plot(line_x,line_y,"--y")
    


def main():

    time = 0

    xTrue = np.zeros((STATE_SIZE,1))
    xEst = np.zeros((STATE_SIZE,1))
    pEst = np.eye(STATE_SIZE)


    histTrue = xTrue
    histEst = xEst
    show_visualization = True

    RFID = np.array([
        [0,10]
    ])



    while SIM_TIME >= time:
        time +=DT
        u = calc_input()

        xTrue, z = observation(xTrue, u, RFID)
        xEst, pEst = ekf_localization(xEst,pEst,z, u)

        
        plot_line(xTrue,RFID)
        histTrue = np.hstack((histTrue,xTrue))
        histEst = np.hstack((histEst,xEst))

        if show_visualization:
            #print(xEst)
            #plt.plot(xEst[0],xEst[1],'.g',label="EstPose")

            #plt.plot(z[0],z[1],'.b',label="measurement")
            plt.plot(RFID[0,0],RFID[0,1],'.b')
            plt.plot(histEst[0],histEst[1],'.g',label="EstPose")
            plt.plot(histTrue[0],histTrue[1],'r--',label="TruePose")
            plt.axis('equal')
            plt.pause(0.01)


        if SIM_TIME >= time:
            plt.clf()



    plt.pause(5)


if __name__ == "__main__":
    main()