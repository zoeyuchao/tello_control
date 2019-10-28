#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import random
import numpy as np

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# if you can not find cv2 in your python, you can try this. usually happen when you use conda.
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import tello_base as tello

y_max_th = 200
y_min_th = 170

img = None
tello_state='mid:-1;x:100;y:100;z:-170;mpry:1,180,1;pitch:0;roll:0;yaw:-19;'
tello_state_lock = threading.Lock()    
img_lock = threading.Lock()    

# send command to tello
class control_handler: 
    def __init__(self, control_pub):
        self.control_pub = control_pub
    
    def forward(self, cm):
        command = "forward "+(str(cm))
        self.control_pub.publish(command)
    
    def back(self, cm):
        command = "back "+(str(cm))
        self.control_pub.publish(command)
    
    def up(self, cm):
        command = "up "+(str(cm))
        self.control_pub.publish(command)
    
    def down(self, cm):
        command = "down "+(str(cm))
        self.control_pub.publish(command)
    
    def right(self, cm):
        command = "right "+(str(cm))
        self.control_pub.publish(command)
    
    def left(self, cm):
        command = "left "+(str(cm))
        self.control_pub.publish(command)

    def cw(self, cm):
        command = "cw "+(str(cm))
        self.control_pub.publish(command)


    def ccw(self, cm):
        command = "ccw "+(str(cm))
        self.control_pub.publish(command)

    def takeoff(self):
        command = "takeoff"
        self.control_pub.publish(command)
        print ("ready")

    def land(self):
        command = "land"
        self.control_pub.publish(command)

    def stop(self):
        command = "stop"
        self.control_pub.publish(command)

#subscribe tello_state and tello_image
class info_updater():   
    def __init__(self):
        rospy.Subscriber("tello_state", String, self.update_state)
        rospy.Subscriber("tello_image", Image, self.update_img)
        con_thread = threading.Thread(target = rospy.spin)
        con_thread.start()

    def update_state(self,data):
        global tello_state, tello_state_lock
        tello_state_lock.acquire() #thread locker
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)

    def update_img(self,data):
        global img, img_lock
        img_lock.acquire()#thread locker
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding = "passthrough")
        img_lock.release()
        # print(img)


# put string into dict, easy to find
def parse_state():
    global tello_state, tello_state_lock
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    print (statestr)
    dict={}
    for item in statestr:
        if 'mid:' in item:
            mid = int(item.split(':')[-1])
            dict['mid'] = mid
        elif 'x:' in item:
            x = int(item.split(':')[-1])
            dict['x'] = x
        elif 'z:' in item:
            z = int(item.split(':')[-1])
            dict['z'] = z
        elif 'mpry:' in item:
            mpry = item.split(':')[-1]
            mpry = mpry.split(',')
            dict['mpry'] = [int(mpry[0]),int(mpry[1]),int(mpry[2])]
        # y can be recognized as mpry, so put y first
        elif 'y:' in item:
            y = int(item.split(':')[-1])
            dict['y'] = y
        elif 'pitch:' in item:
            pitch = int(item.split(':')[-1])
            dict['pitch'] = pitch
        elif 'roll:' in item:
            roll = int(item.split(':')[-1])
            dict['roll'] = roll
        elif 'yaw:' in item:
            yaw = int(item.split(':')[-1])
            dict['yaw'] = yaw
    tello_state_lock.release()
    return dict

def showimg():
    global img, img_lock
    img_lock.acquire()
    cv2.imshow("tello_image", img)
    cv2.waitKey(30)
    img_lock.release()

# mini task: take off and fly to the center of the blanket.
class task_handle():
    class taskstages():
        finding_location  = 0 # find locating blanket 
        order_location  = 1 # find the center of locating blanket and adjust tello 
        finished = 6 # task done signal

    def __init__(self , ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.now_stage = self.taskstages.finding_location

    def main(self): # main function: examine whether tello finish the task
        while not (self.now_stage == self.taskstages.finished):
            if(self.now_stage == self.taskstages.finding_location):
                self.finding_location()
            elif(self.now_stage == self.taskstages.order_location):
                self.order_location()
        print("Task Done!")
        exit(0)
    
    def finding_location(self): # find locating blanket (the higher, the easier)
        assert (self.now_stage == self.taskstages.finding_location)
        while not ( parse_state()['mid'] > 0 ): # if no locating blanket is found:
            distance = random.randint(20,30) # randomly select distance
            print (distance)
            self.ctrl.up(distance) # tello up
            time.sleep(4) # wait for command finished
            showimg()
        self.now_stage = self.taskstages.order_location

    def order_location(self):# adjust tello to the center of locating blanket
        assert (self.now_stage == self.taskstages.order_location)
        state_conf = 0
        self.States_Dict = parse_state()
        while not ( self.States_Dict['yaw'] <= 8 and self.States_Dict['yaw'] >= -8 and self.States_Dict['x'] <= 120 and self.States_Dict['x'] >= 80 and  self.States_Dict['y'] <= 120 and self.States_Dict['y'] >= 80 and abs(self.States_Dict['z']) <= 150 and abs(self.States_Dict['z']) >= 190):
            if ( abs(self.States_Dict['z']) > 190 or abs(self.States_Dict['z']) < 150 ):
                if (self.States_Dict['z'] > 150):
                    self.ctrl.up(20)     
                    time.sleep(4)
                elif (self.States_Dict['z'] < 190):
                    self.ctrl.down(20) 
                    time.sleep(4)
            elif ( self.States_Dict['yaw'] < -8 or self.States_Dict['yaw'] > 8 ):
                if (self.States_Dict['yaw'] < -8):
                    self.ctrl.cw(10)
                    time.sleep(4)
                elif(self.States_Dict['yaw'] > 8):
                    self.ctrl.ccw(10)
                    time.sleep(4)
            elif ( self.States_Dict['x'] < 80 or self.States_Dict['x'] > 120 ):
                if (self.States_Dict['x'] < 80):
                    self.ctrl.right(20)
                    time.sleep(4)
                elif(self.States_Dict['x'] > 120):
                    self.ctrl.left(20)
                    time.sleep(4)
            elif ( self.States_Dict['y'] < 80 or self.States_Dict['y'] > 120 ):
                if (self.States_Dict['y'] < 80):
                    self.ctrl.back(20)
                    time.sleep(4)
                elif(self.States_Dict['y'] > 120):
                    self.ctrl.forward(20)
                    time.sleep(4)
            else:
                time.sleep(2)
                self.ctrl.stop()
                state_conf += 1
            self.States_Dict = parse_state()
            showimg()
            if self.States_Dict['mid'] < 0 :
                self.now_stage = self.taskstages.finding_location
                break
        self.now_stage = self.taskstages.finished     


if __name__ == '__main__':
    rospy.init_node('tello_control', anonymous=True)

    control_pub = rospy.Publisher('command', String, queue_size=1)
    ctrl = control_handler(control_pub)
    infouper = info_updater()
    tasker = task_handle(ctrl)

    time.sleep(5.2)
    ctrl.takeoff( )
    time.sleep(4)
    ctrl.up(60)
    time.sleep(4)

    tasker.main()

    # ctrl.land()

    

