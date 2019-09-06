import numpy as np
import os
import cv2
import utils as ut

img1 = ut.imread("01.jpg")
img2 = ut.imread("02.jpg")

img1 = cv2.resize(img1, dsize=(640, 480), interpolation=cv2.INTER_AREA)
img2 = cv2.resize(img2, dsize=(640, 480), interpolation=cv2.INTER_AREA)

kp1, des1 = ut.getOrbFeature(img1)
kp2, des2 = ut.getOrbFeature(img2)

pts1 = cv2.KeyPoint_convert(kp1)
pts2 = cv2.KeyPoint_convert(kp2)

matches = ut.getFeatureMatches(des1,des2)
F = ut.getFundamentaMatrix(kp1,kp2,matches)


E, mask = cv2.findEssentialMat(pts1, pts2, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=3.0)
points, R, t, mask = cv2.recoverPose(E, pts1, pts2)


print("recover pose mask:",np.sum(mask!=0))
print("R:",R,"t:",t.T)

# Normalize for Esential Matrix calaculation
'''
M_r = np.hstack((R, t))
M_l = np.hstack((np.eye(3, 3), np.zeros((3, 1))))

P_l = np.dot(K_l,  M_l)
P_r = np.dot(K_r,  M_r)
point_4d_hom = cv2.triangulatePoints(P_l, P_r, np.expand_dims(pts_l, axis=1), np.expand_dims(pts_r, axis=1))
point_4d = point_4d_hom / np.tile(point_4d_hom[-1, :], (4, 1))
point_3d = point_4d[:3, :].T
'''
