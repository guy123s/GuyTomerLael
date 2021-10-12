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
def angle(pt1,pt2,pt0):
    dx1 = pt1[0][0] - pt0[0][0]
    dy1 = pt1[0][1] - pt0[0][1]
    dx2 = pt2[0][0] - pt0[0][0]
    dy2 = pt2[0][1] - pt0[0][1]
    return float((dx1*dx2 + dy1*dy2))/math.sqrt(float((dx1*dx1 + dy1*dy1))*(dx2*dx2 + dy2*dy2) + 1e-10)
counter=0;
old = -1
old2=0
area_avg=0;
num_avg=0;
#tello.takeoff()
for i in range(99999999):
    max_area=0;
    counter+=1
    #Capture frame-by-frame
    frame = frame_read.frame
    if True:
        #grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #Canny
        canny = cv2.Canny(frame,80,240,3)

        #contours
        contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        objects = 0
        for i in range(0,len(contours)):
            #count big enought contours
            if (abs(cv2.contourArea(contours[i]))>100):
                objects+=1
            if (abs(cv2.contourArea(contours[i]))>max_area):
                max_area=abs(cv2.contourArea(contours[i]))
            
            if (counter%30==0):
                '''if(num_avg < 30):
                    tello.rotate_clockwise(90)
                else:
                    tello.move_forward(20);'''
                num_avg=0
                area_avg=0
                old = max_area
                old2=objects
            cv2.putText(frame, f'area = {old} objects = {old2}', (10,450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
        area_avg+=max_area
        num_avg+=objects
    

        #Display the resulting frame
        out.write(frame)
        cv2.imshow('frame',frame)
        cv2.imshow('canny',canny)
        if cv2.waitKey(1) == 1048689: #if q is pressed
            break

#When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# initially called Python_last_resort
