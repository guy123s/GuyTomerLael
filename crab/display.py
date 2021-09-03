'''
enables vision support and cancellation
'''

import cv2
import numpy as np
from djitellopy import Tello


#dictionary of all contours
contours = {}
#array of edges of polygon
approx = []
#scale of the text
scale = 2
#camera
#cap = cv2.VideoCapture(0)
print("press q to attempt exit")

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()
cap = tello.get_frame_read()


img=frame_read.frame

#cv2.imwrite("picture.png", img)


# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

#calculate angle
'''def angle(pt1,pt2,pt0):
    dx1 = pt1[0][0] - pt0[0][0]
    dy1 = pt1[0][1] - pt0[0][1]
    dx2 = pt2[0][0] - pt0[0][0]
    dy2 = pt2[0][1] - pt0[0][1]
    return float((dx1*dx2 + dy1*dy2))/math.sqrt(float((dx1*dx1 + dy1*dy1))*(dx2*dx2 + dy2*dy2) + 1e-10)'''

def print_out(frame, canny, out):
    out.write(frame)
    cv2.imshow('frame',frame)
    cv2.imshow('canny',canny)
    if cv2.waitKey(1) == 1048689: #if q is pressed
                return

def time_waster():
    x=0
    for i in range(100000):
        x+=1
    

print("tello battary is: ", tello.get_battery())
for i in range(99999999):
    while(True):
        #Capture frame-by-frame
        frame = frame_read.frame
        if True:
            #Display the resulting frame
            print_out(frame, frame, out)
            

#When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# originally called boo.py
