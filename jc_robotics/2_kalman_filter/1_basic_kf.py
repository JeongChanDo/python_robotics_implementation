import numpy as np
import random
import matplotlib.pyplot as plt


class robot1D:

    def __init__(self):
        self.x = 0
        self.p = np.eye(2)

        self.motion_noise =0
        self.measurement_noise = 0
        self.control = 0 #[m/s]

    def set_control(self,control=0):
        self.control =control
    
    def set_noise(self,motion_noise=0,measurement_noise=0):
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise

    def set_inital_pose_cov(self,pose=0):
        self.x=pose

    def move(self):
        self.x = self.x + self.control+ random.gauss(0,self.motion_noise)
    
    def calc_input():
        return self.control

    def measurement(self):
        z = landmark - self.x + random.gauss(0, self.measurement_noise)
        return z


    def observation(self, xTrue, xDR, u):
        z = observation_model(xTrue) + measurement_noise@np.random.randn(1)




        
"""
def kalman_filter(r, A, B, C):
    
    x = np.array([r.x,r.measurement()]).transpose()
    xtrue = r.x + r.control
    z = r.measurement()
    #print(x)
    #print(A)
    xp = A.dot(x) + B.dot(r.control)+ random.gauss(0,r.motion_noise)
    #print(B.dot(r.control))

    #print("B.r.control  : ",B*r.control)
    #print("random.gauss(0,r.motion_noise) : ",random.gauss(0,r.motion_noise))
    pp = A*r.p*A+r.motion_noise
    k = pp*C/(C*pp*C+r.measurement_noise)
    #print("xtrue : ",xtrue)
    #print("xp : ", xp)
    #print("k : ",k)
    #print("z : ",z)
    #print("C : " ,C)
    xest = xp + k*(z-C*xp)
    pest = (1- k*C)*pp
    #print("xest : " ,xest.shape)
    #print("pest : " ,pest)


    r.x = xest
    r.p = pest

    return r, xtrue, xp, pp, xest, pest
"""

def observation():

def kalman_filter(xEst ):
    xEst = motion_model(xEst)
    

landmark = 10
dt = 0.1
SIM_TIME = 5
animation = True


def main():
    time = 0
    r = robot1D()
    r.set_control(1)
    r.set_noise(motion_noise=0.1,measurement_noise=0.1)
    r.set_inital_pose_cov(pose=0)
    """

    A = 1
    B = 0.1
    C = 1
    """
    A = np.eye(2)
    B = np.zeros((2,2))
    B[0,0] =robot.control *dt
    C = np.eye(2)
    C[0,0] = -1


    xTrue = 0
    xEst = 0
    pEst = 0
    xDR = 0


    hxEst = xEst
    hxTrue = xTrue
    hxDR = xDR
    hz = 0

    while SIM_TIME >=time:
        time += dt
        u = r.calc_input()

        robot, xtrue, xp, pp, xest, pest = kalman_filter(robot,A,B,C)
        true_hist.append(xtrue)
        pred_hist.append([xp,pp])
        esti_hist.append([xest,pest])
        """
        if animation:
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
        """
        time +=dt

    print(true_hist)
    print(pred_hist)
    print(esti_hist)


    

if __name__=="__main__":
    main()