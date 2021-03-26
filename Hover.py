#!/usr/bin/env python

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import rospy
import time

coordinate = [ [5.57,-5.63], [5.55, 5.54], [-5.6,5.54]]
i = 0

class DroneFly():
    """docstring for DroneFly"""
    def __init__(self):

        rospy.init_node('pluto_fly', disable_signals = True)

        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
        self.pluto_roll = rospy.Publisher('/error_roll', Float32, queue_size=10)
        self.pluto_pitch = rospy.Publisher('/error_pitch', Float32, queue_size=10)
        self.pluto_throt = rospy.Publisher('/error_throt', Float32, queue_size=10)

        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
        rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

        # To tune the drone during runtime
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)

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
        # Pid calculation parameters for Throttle
        self.error_roll = 0.0
        self.P_value_roll = 0.0
        self.D_value_roll = 0.0
        self.I_value_roll = 0.0
        self.DerivatorR = 0.0
        self.IntegratorR = 0.0
        # Pid calculation parameters for Throttle
        self.error_yaw = 0.0
        self.P_value_yaw = 0.0
        self.D_value_yaw = 0.0
        self.I_value_yaw = 0.0
        self.DerivatorY = 0.0
        self.IntegratorY = 0.0

        rospy.sleep(.1)

    def isThere(self, i):
        if(abs(self.drone_x)>abs(self.wp_x)-0.2 and abs(self.drone_x)<abs(self.wp_x)+0.2):
            if(abs(self.drone_y)>abs(self.wp_y)-0.2 and abs(self.drone_y)<abs(self.wp_y)+0.2):
                self.wp_x = coordinate[i][0];
                self.wp_y = coordinate[i][1];
                i = i+1


    def arm(self):
        self.cmd.rcAUX4 = 1500
        self.cmd.rcThrottle = 1000
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)


    def position_hold(self):

        rospy.sleep(2)

        print "disarm"
        self.disarm()
        rospy.sleep(.2)
        print "arm"
        self.arm()
        rospy.sleep(.1)

        while True:

            self.calc_pid()

            # Check your X and Y axis. You MAY have to change the + and the -.
            # We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
            pitch_value = int(1500 - self.correct_pitch)
            self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)

            roll_value = int(1500 - self.correct_roll)
            self.cmd.rcRoll = self.limit(roll_value, 1600,1400)

            throt_value = int(1500 - self.correct_throt)
            self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

            yaw_value = int(1500 + self.correct_yaw)
            self.cmd.rcYaw = self.limit(yaw_value, 1650,1450)

            self.pluto_cmd.publish(self.cmd)
            self.pluto_throt.publish(self.error_throt)
            self.pluto_roll.publish(self.error_roll)
            self.pluto_pitch.publish(self.error_pitch)
            isThere()

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
        #PID coefficiets are put in this after finding out the correct values
        self.error_yaw =  1 - self.currentyaw
        self.P_value_yaw = 43 * self.error_yaw
        self.D_value_yaw = 5 * ( self.error_yaw - self.DerivatorY)
        self.DerivatorY = self.error_yaw

        self.IntegratorY = self.IntegratorY + self.error_yaw

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_yaw = self.IntegratorY * 0

        self.correct_yaw = (self.P_value_yaw + self.I_value_yaw/1000 + self.D_value_yaw)/100
        print ("Yaw: ",self.correct_yaw)


    def pid_roll(self):
        self.error_roll = self.wp_y - self.drone_y
        self.P_value_roll = 171 * self.error_roll
        self.D_value_roll = 203 * ( self.error_roll - self.DerivatorR)
        self.DerivatorR = self.error_roll

        self.IntegratorR = self.IntegratorR + self.error_roll

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_roll = self.IntegratorR * 60

        self.correct_roll = (self.P_value_roll + self.I_value_roll/1000 + self.D_value_roll)/100
        print ("Roll", self.correct_roll)
        #Compute Roll PID here

    def pid_pitch(self):
        self.error_pitch = self.wp_x - self.drone_x
        self.P_value_pitch = 171 * self.error_pitch
        self.D_value_pitch = 203 * ( self.error_pitch - self.DerivatorP)
        self.DerivatorP = self.error_pitch

        self.IntegratorP = self.IntegratorP + self.error_pitch

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_pitch = self.IntegratorP * 3

        self.correct_pitch = (self.P_value_pitch + self.I_value_pitch/1000 + self.D_value_pitch)/100
        print ("Pitch", self.correct_pitch)
        #Compute Pitch PID here

    def pid_throt(self):
        self.error_throt = self.wp_z - self.drone_z
        self.P_value_throt = 256 * self.error_throt
        self.D_value_throt = 2100 * ( self.error_throt - self.DerivatorT)
        self.DerivatorT = self.error_throt

        self.IntegratorT = self.IntegratorT + self.error_throt

         # if self.Integrator > 500:
         #     self.Integrator = self.Integrator_max
         # elif self.Integrator < -500:
         #     self.Integrator = self.Integrator_min

        self.I_value_throt = self.IntegratorT * 0

        self.correct_throt = (self.P_value_throt + self.I_value_throt/1000 + self.D_value_throt)/100


        print ("Throttle", self.kp_throt)
        #Compute Throttle PID here

    def limit(self, input_value, max_value, min_value):

        #Use this function to limit the maximum and minimum values you send to your drone

        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value

    #You can use this function to publish different information for your plots
    # def publish_plot_data(self):


    def set_pid_alt(self,pid_val):

        #This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

        self.kp_throt = pid_val.Kp
        self.ki_throt = pid_val.Ki
        self.kd_throt = pid_val.Kd

    def set_pid_roll(self,pid_val):

        #This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

        self.kp_roll = pid_val.Kp
        self.ki_roll = pid_val.Ki
        self.kd_roll = pid_val.Kd

    def set_pid_pitch(self,pid_val):

        #This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

        self.kp_pitch = pid_val.Kp
        self.ki_pitch = pid_val.Ki
        self.kd_pitch = pid_val.Kd

    def set_pid_yaw(self,pid_val):

        #This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

        self.kp_yaw = pid_val.Kp
        self.ki_yaw = pid_val.Ki
        self.kd_yaw = pid_val.Kd

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


if __name__ == '__main__':
    while not rospy.is_shutdown():
        temp = DroneFly()
        temp.position_hold()
        rospy.spin()
