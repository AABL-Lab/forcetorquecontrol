#!/usr/bin/env python3
import sys

from armpy.gen2_teleop import Gen2Teleop
import rospy
import kinova_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time
import traceback
from collections import deque
from std_msgs.msg import Int32

global history
history = [[],[],[],[],[],[]]
average = [0,0,0,0,0,0]

class ForceTorqueController:
    def __init__(self, timer_period = .2): # starts when you make the class
#        self._bota_listener = rospy.Subscriber("/bus0/bota_ftsensor/ft_sensor_readings/wrench", geometry_msgs.msg.WrenchStamped, self.bota_callback)

        # this listens to the data that has been zeroed based on
        # sensor position with the "gravity compensation" package
        self.counter = 0
        self.botacounter = 0
        self.starttime = rospy.Time.now()
        self._controller_timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        
         
        # initialize the twist message we will use to move the arm
        self.twist = geometry_msgs.msg.Twist()
        self.twist.linear.x = 0
        print("Twist x", self.twist.linear.x)
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0

        # this one will be static and be our estop

        self.estop = geometry_msgs.msg.Twist()
        self.estop.linear.x = 0
        self.estop.linear.y = 0
        self.estop.linear.z = 0
        self.estop.angular.x = 0
        self.estop.angular.y = 0
        self.estop.angular.z = 0


        self.teleop = Gen2Teleop(ns="/j2s7s300_driver")


        self._bota_listener = rospy.Subscriber("/ft_sensor/ft_compensated", geometry_msgs.msg.WrenchStamped, self.bota_callback)
        
        print("started bota listener")
        self._arm_listener = rospy.Subscriber("/j2s7s300_driver/out/joint_state", sensor_msgs.msg.JointState, self.joint_state_callback)
        print("started armstate listener")
        
    def bota_callback(self, msg):
        #print("inside the bota callback")
        #geometry_msgs.msg._Wrench.Wrench
        #Vector3  force
        #float64 x
        #float64 y
        #float64 z
        #Vector3  torque
        #float64 x
        #float64 y
        #float64 z
        self._botadata = msg # sets up the time

        history[0].append(msg.wrench.force.x)
        history[1].append(msg.wrench.force.y)
        history[2].append(msg.wrench.force.z)
        history[3].append(msg.wrench.torque.x)
        history[4].append(msg.wrench.torque.y)
        history[5].append(msg.wrench.torque.z)
        # print(history)
        for i, value in enumerate(history):
            if len(history[i]) > 30:
                history[i] = history[i][-30:]
                average[i] = round(sum(history[i]) / float(len(history[i])), 1)
        self._botadata.wrench.force.x=average[0]
        self._botadata.wrench.force.y=average[1]
        self._botadata.wrench.force.z=average[2]
        self._botadata.wrench.torque.x=average[3]
        self._botadata.wrench.torque.y=average[4]
        self._botadata.wrench.torque.z=average[5]

    def joint_state_callback(self, data):
        # the data comes in as sensor_msgs/JointState.msg
        # which is
        #string[] name
        #float64[] position
        #float64[] velocity
        #float64[] effort

#        if self.counter < 10:
#            self.counter = self.counter + 1
#        else:
            #print(data.name)
#            self.counter = 0
        self._jointdata = data


    def timer_callback(self, event):
        print("The Time since start is ", event.current_real-self.starttime)
        # this is the logic to compare the bota data and
        # set the velocity of the arm appropriately.
        try:
            print("BotaData is now", self._botadata) # averaged data
            force = [0,0,0,0,0,0]
            # now we do something
            force[0]=self._botadata.wrench.force.x
            force[1]=self._botadata.wrench.force.y
            force[2]=self._botadata.wrench.force.z
            force[3]=self._botadata.wrench.torque.x
            force[4]=self._botadata.wrench.torque.y
            force[5]=self._botadata.wrench.torque.z

            print("force is", force)

            print("which way to go? \n x=0, y=1, z=2")
            try:
                counter = int(input())
            except:
                counter = s
            if counter == 0:
                self.twist.angular.x = 1
                self.twist.linear.y = 0
                self.twist.linear.z = 0
                self.twist.angular.x =0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.twist.angular.z = 0
            elif counter == 1:
                self.twist.angular.x = 0
                self.twist.linear.y = 1
                self.twist.linear.z = 0
                self.twist.angular.x =0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.twist.angular.z = 0
            elif counter == 2:
                self.twist.angular.x = 0
                self.twist.linear.y = 0
                self.twist.linear.z = 1
                self.twist.angular.x =0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.twist.angular.z = 0
            elif counter == 3:
                self.twist.angular.x = 1
                self.twist.linear.y = 0
                self.twist.linear.z = 0
                self.twist.angular.x =0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.twist.angular.z = 0
            elif counter == 4:
                self.twist.angular.x = 0
                self.twist.linear.y = 0
                self.twist.linear.z = 0
                self.twist.angular.x =1
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.twist.angular.z = 0
            else:
                self.twist.angular.x = 0
                self.twist.linear.y = 0
                self.twist.linear.z = 0
                self.twist.angular.x =0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.twist.angular.z = 0

            print("Twist will be", self.twist)

            # sets the current twist/robot velocity to be equal to twist
            # this will make the robot move!
            self.teleop.set_velocity(self.twist)
            print("moved the robot")
        
        except Exception as e:
            print("exception is", e)
            print(traceback.format_exc())
            time.sleep(20)
            print("Waiting 20 seconds for init")
            #self.teleop.set_velocity(self.twist)
            

#                self.teleop.set_velocity(self.estop)

        
if __name__ == '__main__':
    rospy.init_node('forcetorquecontrol')
    # make an instance of the class, which will also run init
    # and start the subscribers
    this_ft_controller = ForceTorqueController()
    #this_controller = JointTorquesController() # instantiate the controller using built-in joint torques


    rospy.spin()

