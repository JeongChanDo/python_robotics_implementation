import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import random


def KalmanFilter(xEst, pEst, u, A, B, C, R, Q, show_animation=True):


    
    xPred = A@xEst + B*u
    pPred = A@pEst@A.T + R
    z = A@xEst+B*u + Q@np.random.randn(2)

    S = C*pPred*C + Q
    print("pPred : ",pPred)
    print("C : ",C)

    K = pPred*C@np.linalg.inv(S)
    print("k : ",K)
    xEst = xPred + K@(z - C*xPred)
    pEst = (np.eye(2)- K*C)@pPred
    
    if show_animation ==True:
        #x = np.arange(0,xEst[0]+10,0.1)
        x = np.arange(5,30,0.1)
        plot_xPred = stats.norm.pdf(x, xPred[0],pPred[0,0])
        plot_xEst = stats.norm.pdf(x, xEst[0],pEst[0,0])
        plot_z = stats.norm.pdf(x,z[0],Q[0,0])
        print(z)
        plt.plot(x,plot_z,"g")
        
        plt.annotate("p(x|x,u) = N~({:.1f},{:.3f})".format(xPred[0],pPred[0,0]),xy=(xPred[0],max(plot_xPred)))
        plt.plot(x,plot_xPred,".r")
        plt.plot(x,plot_xEst,"b")


        plt.annotate("p(x|z,u) = N~({:.1f},{:.3f})".format(xEst[0],pEst[0,0]),xy=(xEst[0],max(plot_xEst)))
        plt.grid(True)
        plt.pause(0.2)
        plt.clf()

    
    return xEst, pEst




dt = 1


A = np.eye(2)
A[1,1] = 0
B = np.array([dt, 1])
C = 1


#system noise cov
R = np.array([
    [2, 0], # x noise variance
    [0 , 1] #valocity noise variance
])

#measurement noise cov
Q = np.array([
    [0.5, 0], #x measurement noise variance
    [0, 0]
])

xEst = np.array([5, 0])
pEst = np.eye(2)

time = 0
dt = 1
u = 1
SIM_TIME = 20


while SIM_TIME >= time:
    time += dt
    xEst, pEst = KalmanFilter(xEst, pEst,u,A,B,C,R,Q,show_animation=True)
