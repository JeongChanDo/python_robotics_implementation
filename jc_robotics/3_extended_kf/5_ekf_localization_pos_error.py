import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

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

def observation(x,u):
    xTrue = motion_model(x,u)
    z = observation_model(xTrue) + GPS_noise@np.random.randn(2,1)

    return xTrue, z

def multivariate_gaussian(pos,mu,sigma):
    """
    return the multivariate gaussian distribution on array pos.
    """

    n = mu.shape[0]
    sigma_det = np.linalg.det(sigma)
    sigma_inv = np.linalg.inv(sigma)
    N = np.sqrt((2*np.pi)**n*sigma_det)

    #this einsum call calculates (x-mu)^.sigma-1.(x-mu)
    fac = np.einsum('...k,kl,...l->...',pos-mu,sigma_inv,pos-mu)
    return np.exp(-fac/2)/N


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


SIM_Time = 60
time = 0

histTrue = xTrue
histEst = xEst
histz = np.zeros((2,1))
show_visualization = True


N = 100

X = np.linspace(-12,12,N)
Y = np.linspace(0,24,N)
X, Y = np.meshgrid(X,Y)

pos = np.empty(X.shape+(2,))
pos[:,:,0] = X
pos[:,:,1] = Y

fig = plt.figure()

while SIM_Time >= time:
    time +=DT

    xTrue, z = observation(xTrue, u)

    xEst, pEst = ekf_localization(xEst,pEst,z, u)
 
    histTrue = np.hstack((histTrue,xTrue))
    histEst = np.hstack((histEst,xEst))
    histz = np.hstack((histz,z))

    mu = np.array([xEst[0],xEst[1]])
    mu = mu.reshape((2,))
    sigma = np.array([
        [pEst[0,0],pEst[0,1]],
        [pEst[1,0],pEst[1,1]]
    ])

    Z = multivariate_gaussian(pos,mu,sigma)


    if show_visualization:
        ax = fig.gca(projection='3d')

        ax.plot(histEst[0],histEst[1],'b',zs=0.5, linewidth=5,label="EstPose Trajectory")
        ax.plot(histTrue[0],histTrue[1],'--r',zs=0.5, linewidth=2,label="TruePose")
        #ax.plot_surface(X,Y,Z,rstride=3,cstride=3,linewidth=1, antialiased = True, cmap=cm.viridis)
        ax.plot_wireframe(X, Y, Z, rstride=7, cstride=7,label="EstPose")

        cset = ax.contourf(X, Y, Z, zdir='z', offset= -0.5,cmap=cm.Greys)
        ax.set_zlim(-0.5,1)

        ax.legend()

        plt.pause(0.1)

        plt.cla()


pos_x = np.arange(0,SIM_Time+DT,DT)
print("pos_x.shape : " ,pos_x.shape)
print(pos_x)
pos_error = histTrue - histEst
print("pos_error.shape : ", pos_error.shape)

fig2, axs = plt.subplots(3)
axs[0].set_title("pos x error")
axs[0].plot(pos_x,pos_error[0,:])
axs[1].set_title("pos y error")
axs[1].plot(pos_x,pos_error[1,:])
axs[2].set_title("orientation error")
axs[2].plot(pos_x,pos_error[2,:])

plt.show()