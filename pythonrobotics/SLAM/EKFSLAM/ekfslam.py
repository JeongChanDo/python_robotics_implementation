import math
import numpy as np
import matplotlib.pyplot as plt



# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2

#  Simulation parameter
Qsim = np.diag([0.2, np.deg2rad(1.0)])**2
Rsim = np.diag([1.0, np.deg2rad(10.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 30.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

show_animation = True



def ekf_slam(xEst, PEst, u, z):
    #Predict
    S = STATE_SIZE
    #input : xEst[0:S](= x, y, yaw), control input
    #output : predicted state( only robot state, not landmark)
    # initialize xEst shape (3,1)  becuase no landmark state only robot.
    # but after loop if lm are founded, shape will be up 
    print("before loop len : " +str(len(xEst)))
    print("shape : "+str(xEst.shape))
    xEst[0:S] = motion_model(xEst[0:S],u)

    #input :state, control
    #output : G(motion model using Jacobian matrix shape 3,3), Fx differential of motion shape 3,3
    #using for calculating error covariance. fuction motion_model return 2,2 shape. but jacob_motion return 3,3 shape
    G, Fx = jacob_motion(xEst[0:S], u)

    #predicted error covariance = A * P * A.T + Q = system model * prev error cov * system model + system noise
    #predicted error cov = motion model 
    PEst[0:S, 0:S] = G.T*PEst[0:S, 0:S]* G + Fx.T*Cx *Fx

    #initilize P
    initP = np.eye(2)

    #update
    for iz in range(len(z[:,0])): #for each observation
        #xEst = estimated state of robot and lm, Pest = error cov, z =(distance,angle, rfid ID)
        minid = search_correspond_LM_ID(xEst, PEst, z[iz,0:2])
        nLM = calc_n_LM(xEst)

        if minid == nLM:
            #Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_LM_Pos(xEst, z[iz,:])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug

        lm = get_LM_Pos_from_state(xEst,minid)
        #y = measuremnt - perdicted measurement
        #S = H @ Pp @ H.T + R 
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], minid)

        #calculate kalman gain = Pp H.T /( H Pp H + R)[= S]
        K = (PEst@H.T)@np.linalg.inv(S)
        #Estimate state = Predicted State + Kalman gain*(measuremnt - measurement model*predicted state) [==y]
        
        print("before len : " +str(len(xEst)))
        print("shape : "+str(xEst.shape))
        xEst = xEst + (K@y)

        print("after len : " +str(len(xEst)))
        print("shape : "+str(xEst.shape))
        #estimated error covariance = predicted error cov - kalman gain * H * predicted error cov
        # calcualte estimated error covariance
        PEst = (np.eye(len(xEst)) -(K@H)) @PEst

    xEst[2] = pi_2_pi(xEst[2])
    return xEst, PEst


#calculate number of landmark
def calc_n_LM(x):
    #len(x) =  3 , state size = 3 ==> n = 0
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n

#input :state, control
#output : jacob matrix 
def jacob_motion(x, u):

    #np.eye(STATE_SIZE) = (3,3)
    #np.zeros(STATE_SIZE,LM_SIZE*clac_n_LM) = np.zeros(3, 2* 0)
    #Fx = (3,3) diag
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_LM(x)))))

    #calculate diffential state
    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]])

    #Identical matrix + differential state = shape (3,3)
    G = np.eye(STATE_SIZE) + Fx.T * jF * Fx

    return G, Fx



def calc_innovation(lm, xEst, PEst, z, LMid):
    #diffence between selected landmark and robot pose
    delta = lm - xEst[0:2]
    # q= sum's of (lm-xet)^2 -> with sqrt distance from landmark to robot
    # q is calculated for getting Mean Square Error. will be sqrt
    q = (delta.T @ delta)[0, 0]
    # angle between landmark and robot
    zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]

    #zp = [distance, angle]. predicted measurement
    zp = np.array([[math.sqrt(q), pi_2_pi(zangle)]])
    #y is calculated for estimating state as estX= predX + K(measurement - predicted measurement) 
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])

    #H : measurement model
    H = jacobH(q, delta, xEst, LMid + 1)

    # S will be used to calculate kalman gain
    # K = Pp*H/(H*Pp*H.T + R)
    S = H @ PEst @ H.T + Cx[0:2, 0:2]

    return y, S, H




def jacobH(q, delta, x, i):
    #q : distance from lm and robot not sqrt
    #sq : distance
    sq = math.sqrt(q)

    #G = distance * not sqrt distance (2,5)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

    # G = G/not sqrt distance = sq
    G = G / q

    nLM = calc_n_LM(x)
    
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))


    F = np.vstack((F1, F2))

    H = G @ F

    return H



#input : state (robot, LM0, LM1, LM2, ...)
# output : lm (LM0, LM1 , ...)
def get_LM_Pos_from_state(x, ind):

    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def calc_LM_Pos(x, z):
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    return zp

def search_correspond_LM_ID(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """
    # number of landmark
    # input : xEst, output : number of landmark.
    # intialize step. there is no ladmark nLM is 0. after than landmark will be found. 
    nLM = calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):
        #get only landmark state
        lm = get_LM_Pos_from_state(xAug, i)
        #input landmark state vector, Preicted state vector, Predicted error cov, noised measurement(distance, angle, )
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        mdist.append(y.T @ np.linalg.inv(S) @ y)

    mdist.append(M_DIST_TH)  # new landmark

    minid = mdist.index(min(mdist))

    return minid




#control input vector
def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u



#input : true/dead reckoning state, control input, rfid pos
#output : prediction of noised true/dead reckoning, noised measurment(distance and angle)/control input 
def observation(xTrue, xd, u, RFID):

    #true prediction
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        #rfid distance
        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        #rfid distance and angle
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        
        #noised distance and angle to measurement
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.array([dn, anglen, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * Rsim[0, 0],
        u[1, 0] + np.random.randn() * Rsim[1, 1]]]).T

    #dead reckoning prediction state
    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

#input : state, control
#output : predicted state
def motion_model(x, u):

    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    return x





def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))
    xTrue = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)

    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        x_state = xEst[0:STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")

            # plot landmark
            for i in range(calc_n_LM(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")

            plt.plot(hxTrue[0, :],
                     hxTrue[1, :], "-b")
            plt.plot(hxDR[0, :],
                     hxDR[1, :], "-k")
            plt.plot(hxEst[0, :],
                     hxEst[1, :], "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()