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


bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1,des2)
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 10 matches.
img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)

cv2.imshow("img",img3)
cv2.waitKey(0)
cv2.destroyAllWindows()
