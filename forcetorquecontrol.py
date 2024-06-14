#!/usr/bin/env python3
import sys
import math
from armpy.gen2_teleop import Gen2Teleop
import rospy
import kinova_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time
import traceback
from collections import deque
from std_msgs.msg import Int32
import armpy
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker

#FIXME - talk to Reuben
global history
history = [[],[],[],[],[],[]]
average = [0,0,0,0,0,0]
## also may not need?


class ForceTorqueController:
    def __init__(self, timer_period = .1): # starts when you make the class
        # timer_period is in seconds

        self.teleop = Gen2Teleop(ns="/j2s7s300_driver", home_arm=False)
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        
        arm = armpy.arm.Arm()
        arm.set_velocity(.5)
        startposition = [4.721493795519453,4.448460661610131,-0.016183561810626166,1.5199463284150871,3.0829157579242956,4.517873824894174,1.57]
        arm.move_to_joint_pose(startposition)


        #        self._bota_listener = rospy.Subscriber("/bus0/bota_ftsensor/ft_sensor_readings/wrench", geometry_msgs.msg.WrenchStamped, self.bota_callback)
        self._rviz_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        self.twistpublisher = rospy.Publisher("convertedTwistStamped", geometry_msgs.msg.TwistStamped, queue_size = 2)
        
        # this listens to the data that has been zeroed based on
        # sensor position with the "gravity compensation" package
        self._bota_listener = rospy.Subscriber("/ft_sensor/ft_compensated", geometry_msgs.msg.WrenchStamped, self.bota_callback)
        #print("started bota listener")
        self._arm_listener = rospy.Subscriber("/j2s7s300_driver/out/joint_state", sensor_msgs.msg.JointState, self.joint_state_callback)
        #print("started armstate listener")
        self.counter = 0
        self.botacounter = 0
        self.starttime = rospy.Time.now()
        self._controller_timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.Subscriber("/j2s7s300_driver/out/tool_pose", geometry_msgs.msg.PoseStamped, self.check_pose)

        self.pose = geometry_msgs.msg.PoseStamped()
        
         
        # initialize the twist message we will use to move the arm
        self.twist = geometry_msgs.msg.Twist()
        self.twist.linear.x = 0
        #print("Twist x", self.twist.linear.x)
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


        
    def bota_callback(self, msg):
        ##print("inside the bota callback")
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
 
#        history[0].append(msg.wrench.force.x)
#        history[1].append(msg.wrench.force.y)
#        history[2].append(msg.wrench.force.z)
#        history[3].append(msg.wrench.torque.x)
#        history[4].append(msg.wrench.torque.y)
#        history[5].append(msg.wrench.torque.z)
        # # #print(history)
        # for i, value in enumerate(history):
        #     if len(history[i]) > 30:
        #         history[i] = history[i][-30:]
        #         average[i] = round(sum(history[i]) / float(len(history[i])), 1)
        # self._botadata.wrench.force.x=average[0]
        # self._botadata.wrench.force.y=average[1]
        # self._botadata.wrench.force.z=average[2]
        # self._botadata.wrench.torque.x=average[3]
        # self._botadata.wrench.torque.y=average[4]
        # self._botadata.wrench.torque.z=average[5]
        ##print("Botadata from inside callback", self._botadata)
        # botamarker = Marker()
        # botamarker.type = 0
        # botamarker.ns = "botamarker"
        # botamarker.header=self._botadata.header
        # botamarker.id = 2
        # botamarker.scale.x = .1#math.sqrt((self._botadata.wrench.force.x)**2+(self._botadata.wrench.force.y)**2+(self._botadata.wrench.force.z)**2)
        # botamarker.scale.y = .1
        # botamarker.scale.z = .1
        # # at the origin of the frame
        # point0 = geometry_msgs.msg.Point(0,0,0)
        # point1 = geometry_msgs.msg.Point(self._botadata.wrench.force.x,
        #                                  self._botadata.wrench.force.y,
        #                                  self._botadata.wrench.force.z)
        
        # botamarker.points = [point0, point1]
        # botamarker.color.r = 1
        # botamarker.color.g = 0
        # botamarker.color.b = 1
        # botamarker.color.a = 1
        # #
        # self._rviz_publisher.publish(botamarker)

        
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
            ##print(data.name)
#            self.counter = 0
        self._jointdata = data

    def scaleforce(self, force, scalingfactor):
        scaledforce = [scalingfactor*x for x in force]
        #print("scaledforce is ", scaledforce)
        return scaledforce

    def timer_callback(self, event):
        #print("duration", event.last_duration)
        #print("The Time since start is ", event.current_real-self.starttime)
        # this is the logic to compare the bota data and
        # set the velocity of the arm appropriately.
        try:
            ##print("BotaData is now", self._botadata) # averaged data
            force = [0,0,0,0,0,0]

            # now we do something

            force[0]=self._botadata.wrench.force.x
            force[1]=self._botadata.wrench.force.y
            force[2]=self._botadata.wrench.force.z
            force[3]=self._botadata.wrench.torque.x
            force[4]=self._botadata.wrench.torque.y
            force[5]=self._botadata.wrench.torque.z

            #print("force is", force)
            force = self.deadband(force, 2)
            scaledforce = self.scaleforce(force, .3)

            # this creates a desired vector to move along
            # in the frame of reference of the F/T sensor
            # based on the force inputs
            
            outputvectorstamped = self.force2Vector(scaledforce, self._botadata.header)
            ##print("output vector in original frame", outputvectorstamped)
            torques = self.force2Vector([0,0,0], self._botadata.header)
            #torques = self.force2Vector([scaledforce[3],scaledforce[4], scaledforce[5]], self._botadata.header) 



            # This converts it to the base frame, which is needed for
            # Kinova velocity control (the Twist command we will make)
            # and returns the output vector in the base frame

            outputvectorstamped_base = self.tfBuffer.transform(outputvectorstamped,"j2s7s300_link_base",timeout=rospy.Duration(1))
            ##print("just after frame transform", outputvectorstamped_base)
            # This converts the base-frame output vector
            # into Twist translation commands
            converted_twist = self.Vector2Twist(outputvectorstamped_base.header, outputvectorstamped_base.vector, torques.vector) #TwistStamped
            converted_twist.header.stamp=rospy.get_rostime()

            # publish the converted twist header to ROS
            self.twistpublisher.publish(converted_twist)
            
#             ##print("The velocity twist command I would send is", converted_twist)
#             convertedMarker = Marker()
#             convertedMarker.type = Marker.ARROW
#             convertedMarker.header=converted_twist.header
#             convertedMarker.id = 1
#             convertedMarker.ns = "base vector"
#             convertedMarker.scale.x = .1 #math.sqrt((converted_twist.twist.linear.x)**2+(converted_twist.twist.linear.y)**2+(converted_twist.twist.linear.z)**2)
#             convertedMarker.scale.y = .1
#             convertedMarker.scale.z = .1
#             # at the origin of the frame
# #            convertedMarker.pose.position.x = 0
# #            convertedMarker.pose.position.y = 0
# #            convertedMarker.pose.position.z = 0
#             convertedMarker.color.r = 0
#             convertedMarker.color.g = 1
#             convertedMarker.color.b = 1
#             convertedMarker.color.a = 1
#             #
#             point0 = geometry_msgs.msg.Point(0,0,0)
#             point1 = geometry_msgs.msg.Point(converted_twist.twist.linear.x,
#                                              converted_twist.twist.linear.y,
#                                              converted_twist.twist.linear.z)
        
#             convertedMarker.points = [point0, point1]
# #            convertedMarker.pose.orientation.x = converted_twist.twist.linear.x
# #            convertedMarker.pose.orientation.y = converted_twist.twist.linear.y
# #            convertedMarker.pose.orientation.z = converted_twist.twist.linear.z
#             self._rviz_publisher.publish(convertedMarker)

              # Check if the robot is too close to the table
            #collision_checker_node = StateValidity()
            #collision_checker_node.start_collision_checker()


            # this is the part that will make the robot move
            self.teleop.set_velocity(converted_twist.twist)
        

        
            
        except Exception as e:
            #print("exception is", e)
            #print(traceback.format_exc())
            time.sleep(20)
            #print("Waiting 20 seconds for init")
            self.teleop.set_velocity(self.estop)
            pass
        
    def check_pose(self, data):
        self.pose = data

    def deadband(self, force, threshold):
        newforce = []
        for i in range(len(force)):
            if abs(force[i]) < threshold:
                newforce.append(0)
            else:
                newforce.append(force[i])
        #print("newforce is", newforce)
        return newforce

    def force2Vector(self, force, header):
        # force is [x,y,z,rot-x,rot-y,rot-z]
        # but can also be just [x,y,z] or just [rot-x,rot-y, rot-z]
        outputVectorStamped = geometry_msgs.msg.Vector3Stamped()
        ##print("ovs is", outputVectorStamped)
        outputVectorStamped.header = header
        outputVectorStamped.vector.x = force[0]
        outputVectorStamped.vector.y = force[1]
        outputVectorStamped.vector.z = force[2]
        #print("outputvectorstamped is ", outputVectorStamped)
        return outputVectorStamped # a StampedVector3 message

    def Vector2Twist(self, header, linearVector, angularVector):
        # assumes you've split the vector into pieces, puts it together
        # use this after converting between frames, Twist can not be transformed
        twistout = geometry_msgs.msg.TwistStamped()

        twistout.header = header
        twistout.twist.linear = linearVector
        twistout.twist.angular = angularVector
        return twistout # a TwistStamped message
            

    def transform_twist(self,input_twist, from_frame, to_frame):
        # This does not work at all
        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        output_twist = geometry_msgs.msg.Twist()

        try:
 
            
            return output_twist
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        
if __name__ == '__main__':
    rospy.init_node('forcetorquecontrol')
    # make an instance of the class, which will also run init
    # and start the subscribers
    this_ft_controller = ForceTorqueController()
    #this_controller = JointTorquesController() # instantiate the controller using built-in joint torques


    rospy.spin()

