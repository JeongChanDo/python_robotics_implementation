import numpy as np
import os
import cv2

def test():
    print("wow")

def imread(name):
    path = os.getcwd()+"\\vision\\"
    name = path+name
    img = cv2.imread(name)
    return img

def getOrbFeature(img):
    
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Initiate STAR detector
    orb = cv2.ORB_create()


    # find the keypoints with ORB
    kp = orb.detect(img_gray,None)

    # compute the descriptors with ORB
    kp, des = orb.compute(img, kp)

    return kp, des

def getFeatureMatches(des1,des2):
    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params,search_params)

    des1 = np.float32(des1)
    des2 = np.float32(des2)

    matches = flann.knnMatch(des1,des2,k=2)
    return matches

def getFundamentaMatrix(kp1,kp2,matches):
    good = []
    pts1 = []
    pts2 = []

    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.8*n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)
    return F, mask