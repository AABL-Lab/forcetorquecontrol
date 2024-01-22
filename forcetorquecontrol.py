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
    

class ForceTorqueController:
    def __init__(self, timer_period = 2): # starts when you make the class
        self._bota_listener = rospy.Subscriber("bus0/ft_sensor0/ft_sensor_readings/wrench", geometry_msgs.msg.WrenchStamped, self.bota_callback)
        print("started bota listener")
        self._arm_listener = rospy.Subscriber("/j2s7s300_driver/out/joint_state", sensor_msgs.msg.JointState, self.joint_state_callback)
        print("started armstate listener")
        self.counter = 0
        self.botacounter = 0
        self.starttime = rospy.Time.now()
        self._controller_timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)
        
    def bota_callback(self, botadata):
        #geometry_msgs.msg._Wrench.Wrench
        #Vector3  force
        #float64 x
        #float64 y
        #float64 z
        #Vector3  torque
        #float64 x
        #float64 y
        #float64 z
        self._botadata = botadata
      #  if self._botadata > something:
        if self.botacounter < 100:
            self.botacounter = self.botacounter + 1
        else:
            #print("the Bota time is", botadata.header)
            #print(botadata.wrench)
            self.botacounter = 0


    def joint_state_callback(self, data):
        # the data comes in as sensor_msgs/JointState.msg
        # which is
        #string[] name
        #float64[] position
        #float64[] velocity
        #float64[] effort

        if self.counter < 10:
            self.counter = self.counter + 1
        else:
            #print(data.name)
            self.counter = 0
        self._jointdata = data
    
    def timer_callback(self, event):
        print("The Time since start is {(event.current_real-self.starttime).to_sec():.03f}")
        # this is the logic to compare the bota data and
        # set the velocity of the arm appropriately.
        xneutral = 3.17
        xdelta = 2
        yneutral = 2.7
        ydelta = 2
        zneutral = 184
        zdelta = 10
        if self._botadata.wrench.force.x > xneutral+xdelta:
            print("move x plus ", self._botadata.wrench.force.x)
        elif self._botadata.wrench.force.x < xneutral-xdelta:
            print("move x minus ", self._botadata.wrench.force.x)
        elif self._botadata.wrench.force.y > yneutral+ydelta:
            print("move y plus ", self._botadata.wrench.force.y)
        elif self._botadata.wrench.force.y < yneutral-ydelta:
            print("move y minus ", self._botadata.wrench.force.y)
        elif self._botadata.wrench.force.z > zneutral + zdelta:
            print("move z plus ", self._botadata.wrench.force.z)
        elif self._botadata.wrench.force.z < zneutral - zdelta:
            print("move z-minus ", self._botadata.wrench.force.z)
        else:
            print(".")
    
        
  ######################        

    

###############
    #armpy.set_velocity(twist)


    # Get the current state of the robot (position, velocity)

    # Get the current force-torques from the sensor

    # if the force in any direction exceeds the threshold,
    # set the robot velocity in that direction to a value
    # proportional to the force in that direction

#while stopnow == False:
#while rospy.is_shutdown
# twist is a 6DoF (linear x,y,z, angular x,y,z) **velocity**
# geometry_msgs.msg Twist
# representing the linear and angular velocity of the controller

#Gen2Teleop.set_velocity(twist) # sets the current twist to be equal to twist
        
if __name__ == '__main__':
    rospy.init_node('forcetorquecontrol')
    # make an instance of the class, which will also run init
    # and start the subscribers
    # this_ft_controller = ForceTorqueController()
    this_controller = JointTorquesController() # instantiate the controller using built-in joint torques



    # teleop = Gen2Teleop(ns="/j2s7s300_driver")
    # twist = geometry_msgs.msg.Twist()
    # twist.linear.x = .5
    # twist.linear.y = 0
    # twist.linear.z = 0
    # twist.angular.x = 0
    # twist.angular.y = 0
    # twist.angular.z = 0

    

    # stop = geometry_msgs.msg.Twist()
    # stop.linear.x = 0
    # stop.linear.y = 0
    # stop.linear.z = 0
    # stop.angular.x = 0
    # stop.angular.y = 0
    # stop.angular.z = 0

    # teleop.set_velocity(twist)
    # print("setting velocity to 1x")

    
    # time.sleep(1)

    # teleop.set_velocity(stop)
    # print("stopping")

   # this is the last thing, no code after this
    rospy.spin()

