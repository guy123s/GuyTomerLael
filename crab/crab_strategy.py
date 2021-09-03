'''
This file implements the crab strategy!
'''

import cv2
import numpy as np
from djitellopy import Tello
import time
import keyboard


GRID_SIZE=5000
LOW_OBJECT = 30
HIGH_OBJECT = 70

#dictionary of all contours
contours = {}
#array of edges of polygon
approx = []
#scale of the text
scale = 2
#camera
#cap = cv2.VideoCapture(0)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

direction = "north"
x=GRID_SIZE//2
y=GRID_SIZE//2

grid=[[0 for i in range(GRID_SIZE)] for j in range(GRID_SIZE)]
dirs = {"north":0, "east":1, "south":2, "west":3}
revs = {0:"north", 1:"east", 2:"south", 3:"west"}
dirs2 = {"up":0, "right":1, "down":2, "left":3}

def danger_level(num):
    if (num<= (LOW_OBJECT/20)):
        return "High"
    if (num > (HIGH_OBJECT/20)):
        return "Low"
    else:
        return "Medium"


def want_to_continue():
    if keyboard.read_key()=="q":
        return False
    else:
        return True

def change_pos(dest):
    global y
    global x
    dest = (dirs2[dest]+dirs[direction])%4
    if dest == 0:
        y += 1
    if dest == 1:
        x += 1
    if dest == 2:
        y -= 1
    if dest == 3:
        x -= 1
    return grid[x][y]
        

def print_out(frame, canny, out):
    out.write(frame)
    cv2.imshow('frame',frame)
    #cv2.imshow('canny',canny)
    if cv2.waitKey(1) == 1048689: #if q is pressed
                return

def not_wall(cap):
    max_area=0;
    objects = 0;
    #Capture frame-by-frame
    for i in range (20):
        new_objects=0
        frame = cap.frame
        #grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #Canny
        canny = cv2.Canny(frame,80,240,3)

        #contours
        contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for i in range(0,len(contours)):
            #count big enought contours
            if (abs(cv2.contourArea(contours[i]))>100):
                objects+=1
            if (abs(cv2.contourArea(contours[i]))>max_area):
                max_area=abs(cv2.contourArea(contours[i]))
        #cv2.putText(frame, f'objects = {objects}', (10,450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
        #cv2.putText(frame, f'Probability of wall is {danger_level(new_objects)}, and {objects}', (10,450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
        print_out(frame, canny, out)
        #objects+=new_objects
    print("objects = ", objects)
    if (objects<LOW_OBJECT):
        return False
    else:
        return True
        
        

def besto_crabbo(pictures_array):
    max_objects = 0
    index = 0
    print("array length is" , len(pictures_array))
    for i in range (len(pictures_array)):
        objects=0
        gray = cv2.cvtColor(pictures_array[i], cv2.COLOR_BGR2GRAY)
        #Canny
        canny = cv2.Canny(pictures_array[i],80,240,3)

        #contours
        contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        objects = 0
        for i in range(0,len(contours)):
            #count big enought contours
            if (abs(cv2.contourArea(contours[i]))>100):
                objects+=1
        if objects>max_objects:
            max_objects = objects
            index = len(pictures_array) - i
            print("yam yam! new winner!")
        print("crab score is: ", objects)
        
    pictures_array = []
    return index

def main():
    try:
        #connect to the tello
        tello = Tello()
        tello.connect()
        global direction
        print("tello battery % is: ",tello.get_battery())
        #gets images
        tello.streamon()
        cap = tello.get_frame_read()

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

        tello.takeoff()
        grid[x][y]=1
        
        #approximate what the best initial direction is
        
        while(True):
            while(not_wall(cap)):
                change_pos("up");
                tello.move_forward(80);
                time.sleep(3)
                grid[x][y]=1;
                print("not wall")
        #if we think we find a wall, start crab walk
            tello.rotate_counter_clockwise(90)
            new_dir = (dirs[direction]+3)%4
            direction = revs[new_dir]
            crab_len=1
            crab_array=[]
            while(change_pos("left")==1):
                crab_len+=1
                tello.move_left(80)
                frame = cap.frame
                crab_array.append(frame)
                print_out(frame, frame, out)
            change_pos("right")
        #find the best place to go to from crab walk
            i=besto_crabbo(crab_array)
            print("i is", i)
            tello.move_right(80*i)
            '''for t in range(i):
                print("strting moving ", i, "to the right")
                tello.move_right(80)
                time.sleep(0.4)'''
    except KeyboardInterrupt:
        print("You pressed ctrl + c, so you want to quite")
        cv2.destroyAllWindows()
        tello.land()
        tello.streamoff()
    except:
        print("Error, landing now")
        cv2.destroyAllWindows()
        tello.land()
        tello.streamoff()
if __name__ == "__main__":
    main()

# initially called test1.py
