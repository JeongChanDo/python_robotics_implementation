import cv2
import sys

cap = cv2.VideoCapture(0)

if cap.isOpen():
    print("open")
else:
	sys.exit()

while 1:
	ret, fram = cap.read()
	
	if ret:
		gray = cv2.cvtColor(fram, cv2.COLOR_BGR2GRAY)
		cv2.imshow('video', gray)
		k == cv2.waitKey(1) & 0xFF
		if k == 27:
			break
	else:
		print('error')

cap.release()
cv2.destroyAllWindows()