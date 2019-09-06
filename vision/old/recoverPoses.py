import numpy as np
import os
import cv2
import utils as ut

img1 = ut.imread("pose_test/1.jpg")
img2 = ut.imread("pose_test/4.jpg")

img1 = cv2.resize(img1, dsize=(640, 480), interpolation=cv2.INTER_AREA)
img2 = cv2.resize(img2, dsize=(640, 480), interpolation=cv2.INTER_AREA)

kp1, des1 = ut.getOrbFeature(img1)
kp2, des2 = ut.getOrbFeature(img2)

pts1 = cv2.KeyPoint_convert(kp1)
pts2 = cv2.KeyPoint_convert(kp2)

E, mask = cv2.findEssentialMat(pts1, pts2, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=3.0)
points, R, t, mask = cv2.recoverPose(E, pts1, pts2)


print("recover pose mask:",np.sum(mask!=0))
print("R:",R,"t:",t.T)

R_pos = 1
T_pos = 1

R_pos = R_pos*R
T_pos = T_pos + t.T*R

pos = np.dot(t.T,R)
print(pos)
'''
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1,des2)
matches = sorted(matches, key = lambda x:x.distance)


# Draw first 10 matches.
img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)

cv2.imshow("img",img3)
cv2.waitKey(100)
cv2.destroyAllWindows()
'''