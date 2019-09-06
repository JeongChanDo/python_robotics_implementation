import numpy as np
import os
import cv2

path = os.getcwd()+"\\vision\\"
print(path)
name = path +"000000.png"
img = cv2.imread(name)

img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# Initiate STAR detector
orb = cv2.ORB_create()


# find the keypoints with ORB
kp = orb.detect(img_gray,None)

# compute the descriptors with ORB
kp, des = orb.compute(img, kp)
img2=None

# draw only keypoints location,not size and orientation
img2 = cv2.drawKeypoints(img,kp,img2,color=(0,0,255), flags=0)

cv2.imshow('orb',img2)
cv2.waitKey(0)
cv2.destroyAllWindows()
