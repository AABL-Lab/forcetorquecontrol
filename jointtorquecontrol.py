#!/usr/bin/env python3
import sys

from armpy.gen2_teleop import Gen2Teleop
import rospy
import kinova_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time

from collections import deque

class JointTorquesController:
    def __init__(self, timer_period = 2):
        self.joint_torque_listener =rospy.Subscriber("/j2s7s300_driver/out/joint_torques", kinova_msgs.msg.JointAngles, self.jointtorques_callback)
        print("started joint torque listener")
        self._arm_listener = rospy.Subscriber("/j2s7s300_driver/out/joint_state", sensor_msgs.msg.JointState, self.joint_state_callback)
        print("started armstate listener")
        self.jtcounter = 0
        self.counter = 0
        self.starttime = rospy.Time.now()
        self._controller_timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)
        self.j1threshold = .5 # how much force on j1 before it moves
        self.jtaverager = kinova_msgs.msg.JointAngles(0,0,0,0,0,0,0)
        self.teleop = Gen2Teleop(ns="/j2s7s300_driver")
        # initialize the twist message we will use to move the arm
        self.twist = geometry_msgs.msg.Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0

        # this one will be static and be our estop
        self.stop = geometry_msgs.msg.Twist()
        self.stop.linear.x = 0
        self.stop.linear.y = 0
        self.stop.linear.z = 0
        self.stop.angular.x = 0
        self.stop.angular.y = 0
        self.stop.angular.z = 0
        

    def jointtorques_callback(self, jointtorquedata):
        # right-append the new joint torque data to the queue
        self._jointtorquedata = jointtorquedata
         

        if self.jtcounter < 10:
            self.jtcounter = self.jtcounter + 1
            self.jtaverager.joint1 = self.jtaverager.joint1 +jointtorquedata.joint1
            self.jtaverager.joint2 = self.jtaverager.joint2 + jointtorquedata.joint2
            self.jtaverager.joint3 = self.jtaverager.joint3 + jointtorquedata.joint3
            self.jtaverager.joint4 = self.jtaverager.joint4 + jointtorquedata.joint4
            self.jtaverager.joint5 = self.jtaverager.joint5 + jointtorquedata.joint5
            self.jtaverager.joint6 = self.jtaverager.joint6 + jointtorquedata.joint6
            self.jtaverager.joint7 = self.jtaverager.joint7 + jointtorquedata.joint7
            
            
           # print(self.jtaverager)
            

        else:
            #print("the joint torques are", jointtorquedata)
            self.jtaverager.joint1 = self.jtaverager.joint1/10
            self.jtaverager.joint2 = self.jtaverager.joint2/10
            self.jtaverager.joint3 = self.jtaverager.joint3/10
            self.jtaverager.joint4 = self.jtaverager.joint4/10
            self.jtaverager.joint5 = self.jtaverager.joint5/10
            self.jtaverager.joint6 = self.jtaverager.joint6/10
            self.jtaverager.joint7 = self.jtaverager.joint7/10
            
        #print("the average over the last 100 messages is ", self.jtaverager)
            if self._jointtorquedata.joint1 > self.jtaverager.joint1 + self.j1threshold:
                difference = (self._jointtorquedata.joint1-self.jtaverager.joint1)
                print("we would move in +x by ", difference)
                self.twist.linear.x = .2*difference
                self.teleop.set_velocity(self.twist)

            elif self._jointtorquedata.joint1 < self.jtaverager.joint1-self.j1threshold:
                difference = (self._jointtorquedata.joint1-self.jtaverager.joint1)
                print("we would move in -x by", difference)
                self.twist.linear.x = .2*difference
                self.teleop.set_velocity(self.twist)

                
            self.jtcounter = 0
            
    def timer_callback(self, event):
        #print(f"The Time since start is {(event.current_real-self.starttime).to_sec():.03f}")
        pass



        # this is the logic to compare the arm torque data and
        # set the velocity of the arm appropriately.

        # because we are moving short distances with a small loop,
        # we will use J1 torques as if they were x+- commands
        # (should also be y depending on angle)
        # and J2 torques as if they were z+- commands
        # this is a total hack and should get replaced with some real kinematic solution, but we will use the F-T sensor for that.
    def joint_state_callback(self, data): 
        if self.counter < 10:
            self.counter = self.counter + 1
        else:
            #print(data.name)
            self.counter = 0
        self._jointdata = data
    
if __name__ == '__main__':
    rospy.init_node('forcetorquecontrol')
    # make an instance of the class, which will also run init
    # and start the subscribers

    this_controller = JointTorquesController() # instantiate the controller using built-in joint torques

    rospy.spin()

