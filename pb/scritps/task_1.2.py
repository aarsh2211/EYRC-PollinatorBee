#!/usr/bin/env python
from plutodrone.msg import *
from pid_tune.msg import *
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

import rospy
import time

class WayPoint():

    def __init__(self):

        rospy.init_node('WayPoint Navigation', disable_signals = True)

        self.ros_bridge = cv_bridge.CvBridge()
        self.largest_area=0;
        self.largest_contour_index=0
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)


        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
        #Publishers for plotting error with time graphs to aid in PID tuning
        self.Red = rospy.Publisher('/red', Int32, queue_size=10)
        self.Blue = rospy.Publisher('/blue', Int32, queue_size=10)
        self.Green = rospy.Publisher('/green', Int32, queue_size=10)

        #subscribers for yaw and roll pitch Altitude
        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
        rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)


        self.cmd = PlutoMsg()

        # Position to hold.
        self.wp_x = -5.63
        self.wp_y = -5.63
        self.wp_z = 30.0
        self.currentyaw = 0

        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.cmd.plutoIndex = 0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        #PID constants for Roll
        self.kp_roll = 0.0
        self.ki_roll = 0.0
        self.kd_roll = 0.0

        #PID constants for Pitch
        self.kp_pitch = 0.0
        self.ki_pitch = 0.0
        self.kd_pitch = 0.0

        #PID constants for Yaw
        self.kp_yaw = 0.0
        self.ki_yaw = 0.0
        self.kd_yaw = 0.0

        #PID constants for Throttle
        self.kp_throt = 0.0
        self.ki_throt = 0.0
        self.kd_throt = 0.0

        # Correction values after PID is computed
        self.correct_roll = 0.0
        self.correct_pitch = 0.0
        self.correct_yaw = 0.0
        self.correct_throt = 0.0

        # Loop time for PID computation. You are free to experiment with this
        self.last_time = 0.0
        self.loop_time = 0.032

        # Pid calculation paramerters for Pitch
        self.error_pitch = 0.0
        self.P_value_pitch = 0.0
        self.D_value_pitch = 0.0
        self.I_value_pitch = 0.0
        self.DerivatorP = 0.0
        self.IntegratorP = 0.0
        # Pid calculation parameters for Throttle
        self.error_throt = 0.0
        self.P_value_throt = 0.0
        self.D_value_throt = 0.0
        self.I_value_throt = 0.0
        self.DerivatorT = 0.0
        self.IntegratorT = 0.0
        # Pid calculation parameters for roll
        self.error_roll = 0.0
        self.P_value_roll = 0.0
        self.D_value_roll = 0.0
        self.I_value_roll = 0.0
        self.DerivatorR = 0.0
        self.IntegratorR = 0.0
        # Pid calculation parameters for roll
        self.error_yaw = 0.0
        self.P_value_yaw = 0.0
        self.D_value_yaw = 0.0
        self.I_value_yaw = 0.0
        self.DerivatorY = 0.0
        self.IntegratorY = 0.0
        # Variables for number of patches
        self.red = 0
        self.blue = 0
        self.green = 0

        # Integrator and flags for Navigation and landing
        self.i = 0
        self.flag = 0




        rospy.sleep(.1)


    def arm(self):
        self.cmd.rcAUX4 = 1500
        self.cmd.rcThrottle = 1000
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    #Function to aid in smooth landing
    def land(self):
        self.wp_z+=0.01
        if (self.drone_z>42):
            self.disarm()

    #Function to check if drone has reached last set point
    def at_origin(self):
        if (abs(self.drone_x) < 0.2 and abs(self.drone_y) < 0.2 and self.drone_z < 31.5 and self.drone_z>28.5):
            return True
        else:
            return False

    # WayPoint Navigation is dones here
    def isThere(self):

        coordinate = [ [5.57,-5.63], [5.55, 5.54], [-5.6,5.54],[0.0, 0.0, 30]]

        if (self.at_origin()):
            print("landing...")
            self.flag=1

        if (self.i>3 and self.flag):       #abs(self.drone_x)<0.2 and abs(self.drone_y)<0.2 and
            self.land()


        if(abs(self.drone_x - self.wp_x) < 0.1 and self.i<=3):
            if(abs(self.drone_y - self.wp_y)<0.1):
                if(self.drone_z>28.5 and self.drone_z<31.5):
                    print "Visiting Point: ", (coordinate[self.i][0], coordinate[self.i][1],30)
                    self.wp_x = coordinate[self.i][0];
                    self.wp_y = coordinate[self.i][1];
                    self.i = self.i+1



    def position_hold(self):

        rospy.sleep(2)

        print "disarm"
        self.disarm()
        rospy.sleep(.2)
        print "arm"
        self.arm()
        rospy.sleep(.1)

        while True:


            self.findcolor()




    def calc_pid(self):
        self.seconds = time.time()
        current_time = self.seconds - self.last_time
        if(current_time >= self.loop_time):
            self.pid_roll()
            self.pid_pitch()
            self.pid_throt()
            self.pid_yaw()
            self.last_time = self.seconds


    def pid_yaw(self):
        #Compute Yaw PID is computed here
                                                    #PID coefficiets are put in this after finding out the correct values for pid_tune package provided
        self.error_yaw =  1 - self.currentyaw       #To get the starting orientation of YAW that is 1 or 0
        self.P_value_yaw = 50 * self.error_yaw      #Replace 43 with kp_yaw and so on to use PID tune
        self.D_value_yaw = 6 * ( self.error_yaw - self.DerivatorY)
        self.DerivatorY = self.error_yaw

        self.IntegratorY = self.IntegratorY + self.error_yaw

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_yaw = self.IntegratorY * 3

        self.correct_yaw = (self.P_value_yaw + self.I_value_yaw/1000 + self.D_value_yaw)/100
        #print ("Yaw: ",self.correct_yaw)


    def pid_roll(self):
        #Roll pid is computed here

        self.error_roll = self.wp_y - self.drone_y
        self.P_value_roll = 1036 * self.error_roll       #Replace 43 with kp_roll and so on to use PID tune package
        self.D_value_roll = 1036 * ( self.error_roll - self.DerivatorR)
        self.DerivatorR = self.error_roll

        self.IntegratorR = self.IntegratorR + self.error_roll

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_roll = self.IntegratorR * 0

        self.correct_roll = (self.P_value_roll - self.I_value_roll/1000 + self.D_value_roll)/100
        #print ("Roll", self.correct_roll)


    def pid_pitch(self):
        #Pitch pid is computed here

        self.error_pitch = self.wp_x - self.drone_x
        self.P_value_pitch = 1036 * self.error_pitch
        self.D_value_pitch = 1036 * ( self.error_pitch - self.DerivatorP)
        self.DerivatorP = self.error_pitch

        self.IntegratorP = self.IntegratorP + self.error_pitch

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_pitch = self.IntegratorP * 0

        self.correct_pitch = (self.P_value_pitch + self.I_value_pitch/1000 + self.D_value_pitch)/100
        #print ("Pitch", self.correct_pitch)


    def pid_throt(self):
        #throttle pid is computed here

        self.error_throt = self.wp_z - self.drone_z
        self.P_value_throt = 1485 * self.error_throt
        self.D_value_throt = 36220 * ( self.error_throt - self.DerivatorT)
        self.DerivatorT = self.error_throt

        self.IntegratorT = self.IntegratorT + self.error_throt

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_throt = self.IntegratorT * 3

        self.correct_throt = (self.P_value_throt + self.I_value_throt/1000 + self.D_value_throt)/100


        #print ("Throttle", self.kp_throt)


    def limit(self, input_value, max_value, min_value):

        #Use this function to limit the maximum and minimum values you send to your drone

        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value



    def get_pose(self,pose):

        #This is the subscriber function to get the whycon poses
        #The x, y and z values are stored within the drone_x, drone_y and the drone_z variables

        self.drone_x = pose.poses[0].position.x
        self.drone_y = pose.poses[0].position.y
        self.drone_z = pose.poses[0].position.z

    def get_yaw(self,yaw):

        #This is the subscriber function to get the whycon poses
        #The x, y and z values are stored within the drone_x, drone_y and the drone_z variables

        self.currentyaw = yaw.data


    def image_callback(self,msg):

        #converting whycon image into a numpy image used by opencv
        self.image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def findcolor(self):
        blurred = cv2.GaussianBlur(self.image, (11, 11), 0)
        threshgreen = cv2.inRange(blurred, (0,225,0), (200,255,200))
        threshblue = cv2.inRange(blurred, (230,0,0), (255,220,220))
        threshred = cv2.inRange(blurred, (0,0,220), (220,220,255))


        #thresh = cv2.threshold(output, 165, 255, cv2.THRESH_BINARY)[1]


    #    threshred = cv2.erode(threshred,None, iterations=2)
        threshred = cv2.dilate(threshred, None, iterations=4)

        threshblue = cv2.erode(threshblue,None, iterations=2)
        threshblue = cv2.dilate(threshblue, None, iterations=4)

        #threshgreen = cv2.erode(threshgreen,None, iterations=2)
        threshgreen = cv2.dilate(threshgreen, None, iterations=4)

        im2, contoursred, hierarchy = cv2.findContours(threshred,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        im2, contoursblue, hierarchy = cv2.findContours(threshblue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        im2, contoursgreen, hierarchy = cv2.findContours(threshgreen,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if(contoursred):
            larcnt = contoursred[0]
            for cnt in contoursred:
                area = cv2.contourArea(cnt)
                if (area>self.largest_area):
                    self.largest_area=area
                    larcnt = cnt

            x,y,w,h = cv2.boundingRect(larcnt)
            cv2.rectangle(self.image,(x,y),(x+w,y+h),(0,0,255),2)
            self.largest_area=0;

        if(contoursblue):
            larcnt = contoursblue[0]
            for cnt in contoursblue:
                area = cv2.contourArea(cnt)
                if (area>self.largest_area):
                    self.largest_area=area
                    larcnt = cnt

            x,y,w,h = cv2.boundingRect(larcnt)
            cv2.rectangle(self.image,(x,y),(x+w,y+h),(255,0,0),2)
        self.largest_area=0;
        if(contoursgreen):

            larcnt = contoursgreen[0]
            for cnt in contoursgreen:
                area = cv2.contourArea(cnt)
                if (area>self.largest_area):
                    self.largest_area=area
                    larcnt = cnt

            x,y,w,h = cv2.boundingRect(larcnt)
            cv2.rectangle(self.image,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.imshow("LED count", self.image)
        cv2.imshow("Thresh", threshgreen)
        cv2.waitKey(1)

if __name__ == '__main__':

    while not rospy.is_shutdown():
        temp = WayPoint()
        temp.position_hold()
        rospy.spin()
