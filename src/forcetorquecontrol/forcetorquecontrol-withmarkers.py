#!/usr/bin/env python3
import sys
import math

from simple_pid import PID
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
from kinova_msgs.msg import PoseVelocity
from kinova_msgs.srv import Start, Stop, HomeArm

DEFAULT_ROBOT_NS = "/j2s7s300_driver"
CARTESIAN_VEL_TOPIC = "/in/cartesian_velocity"
START_SERVICE = "/in/start"
STOP_SERVICE = "/in/stop"
HOME_ARM_SERVICE = "/in/home_arm"

class ForceTorqueController:
    def __init__(self, threshold = 2.0, scalingfactor = .3, K_P=1.0, K_D=1.0, arm_velocity=.5, controltype="PD"):
        # timer_period is in seconds
        # arm_velocity is 0-1
        if controltype=="PD":
            print("Controller type is ",controltype, "with K_P ", K_P, "and K_D ", K_D)
        ns = rospy.resolve_name(DEFAULT_ROBOT_NS)

        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        self.threshold = threshold
        self.scalingfactor = scalingfactor
        self.K_P = K_P
        self.K_D = K_D
        self.timer_period = .01 # must be 100Hz for Kinova arm
        self._command = None
        arm = armpy.arm.Arm()
        arm.set_velocity(arm_velocity)
        self._started = False
        self._controller_timer = None
        self.controltype=controltype
        self.robot_base_link = "j2s7s300_link_base"
        #self.robot_base_link = "base_link"
        self.print = False #this is for debugging, True if force > threshold in any direction
        self._rviz_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        self.twistpublisher = rospy.Publisher("convertedTwistStamped", geometry_msgs.msg.TwistStamped, queue_size = 2)
        
        # this listens to the data that has been zeroed based on
        # sensor position with the "gravity compensation" package
        self._bota_listener = rospy.Subscriber("/ft_sensor/ft_compensated", geometry_msgs.msg.WrenchStamped, self.bota_callback)
        #print("started bota listener")
        self._arm_listener = rospy.Subscriber("/j2s7s300_driver/out/joint_state", sensor_msgs.msg.JointState, self.joint_state_callback)
        # this will publish cartesian velocities to the Kinova arm
        self._cart_vel_pub = rospy.Publisher(ns + CARTESIAN_VEL_TOPIC, PoseVelocity, queue_size=1)
       #print("started armstate listener")
        self.counter = 0
        self.botacounter = 0
        self.starttime = rospy.Time.now()

        # used for collision checking
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

    def set_velocity(self, thistwist):
        """
        Attributes
        ----------
        twist : geometry_msgs.msg Twist
            The linear and angular velocity of controller

        Can take both Twist and TwistStamped
        """

        
        if not self._started:
            self.start()
            self._started = True
            
        try:  # for TwistStamped   
            self._command = PoseVelocity(
                twist_linear_x = thistwist.twist.linear.x,
                twist_linear_y = thistwist.twist.linear.y,
                twist_linear_z = thistwist.twist.linear.z,
                twist_angular_x = thistwist.twist.angular.x,
                twist_angular_y = thistwist.twist.angular.y,
                twist_angular_z = thistwist.twist.angular.z
            )
        except AttributeError: # for Twist
            self._command = PoseVelocity(
                twist_linear_x = thistwist.linear.x,
                twist_linear_y = thistwist.linear.y,
                twist_linear_z = thistwist.linear.z,
                twist_angular_x = thistwist.angular.x,
                twist_angular_y = thistwist.angular.y,
                twist_angular_z = thistwist.angular.z
            )

        
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


        ##########################################################
        #This section sets up a marker showing the
        # vector of the force-torque data
        ########################################################
        botamarker = Marker()
        botamarker.type = 0
        botamarker.ns = "botamarker"
        botamarker.header=self._botadata.header
        botamarker.id = 2
        botamarker.scale.x = .1#math.sqrt((self._botadata.wrench.force.x)**2+(self._botadata.wrench.force.y)**2+(self._botadata.wrench.force.z)**2)
        botamarker.scale.y = .1
        botamarker.scale.z = .1
        # at the origin of the frame
        point0 = geometry_msgs.msg.Point(0,0,0)
        point1 = geometry_msgs.msg.Point(self._botadata.wrench.force.x,
                                          self._botadata.wrench.force.y,
                                          self._botadata.wrench.force.z)
        
        botamarker.points = [point0, point1] #purple
        botamarker.color.r = 1
        botamarker.color.g = 0
        botamarker.color.b = 1
        botamarker.color.a = 1
        # #
        self._rviz_publisher.publish(botamarker)
        ############################################################
        
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

    def force_controller(self, force):
        if self.controltype=="P":
            scaledforce = [self.scalingfactor*x for x in force]
        elif self.controltype=="PD":
            pidx = PID(self.K_P,1, self.K_D)
            pidy = PID(self.K_P, 1, self.K_D)
            pidz = PID(self.K_P, 1, self.K_D)
            scaledforce = [pidx(force[0]), pidy(force[1]), pidz(force[2])]


        else:
            print(self.controltype, "is not a supported controller")
        return scaledforce

    def timer_callback(self, event):
        #rospy.loginfo("callback start")
        #if event.last_duration:
        #    print("duration", round(event.last_duration, 4))
        #print("The Time since start is ", event.current_real-self.starttime)
        # this is the logic to compare the bota data and
        # set the velocity of the arm appropriately.
        try:

            force = [0,0,0,0,0,0]


            force[0]=self._botadata.wrench.force.x
            force[1]=self._botadata.wrench.force.y
            force[2]=self._botadata.wrench.force.z
            force[3]=self._botadata.wrench.torque.x
            force[4]=self._botadata.wrench.torque.y
            force[5]=self._botadata.wrench.torque.z
            if self.print==True:
                rospy.loginfo("force is %s", force)
            force = self.deadband_fxn(force, self.threshold)
            #rospy.loginfo("deadband done")
            
            scaledforce = self.force_controller(force) # control params are class variables
            if self.print==True:
                print("========================================timercallbackstart======")
      
            if self.print==True:
                rospy.loginfo("force scaled is %s", scaledforce)
            # this creates a desired vector to move along
            # in the frame of reference of the F/T sensor
            # based on the force inputs
            
            outputvectorstamped = self.force2Vector(scaledforce, self._botadata.header)
            #print("output vector in original frame", outputvectorstamped)
            torques = self.force2Vector([0,0,0], self._botadata.header)
            # currently ignoring torque, comment line above and
            #uncomment line below to use torque
            #torques = self.force2Vector([scaledforce[3],scaledforce[4], scaledforce[5]], self._botadata.header) 

            # This converts it to the base frame, which is needed for
            # Kinova velocity control (the Twist command we will make)
            # and returns the output vector in the base frame
            #rospy.loginfo("just before frame transform")
            try:
                if self.print==True:
                    print("Transform from", outputvectorstamped.header.frame_id, "to", self.robot_base_link)
#                transform = self.tfBuffer.lookup_transform_core(outputvectorstamped.header.frame_id, self.robot_base_link, rospy.Time())
                transform = self.tfBuffer.lookup_transform_core(self.robot_base_link, outputvectorstamped.header.frame_id,  rospy.Time())
            except tf2_ros.LookupException:
                rospy.logwarn("can not look up transform")
                return
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(f"timing error {e}")
                return

            do_transform = self.tfBuffer.registration.get(type(outputvectorstamped))
            outputvectorstamped_base = do_transform(outputvectorstamped, transform)
            if self.print==True:
                print(outputvectorstamped_base)
            
            #rospy.loginfo("just after frame transform")
            
            #This converts the base-frame output vector
            # into Twist translation commands
            converted_twist = self.Vector2Twist(outputvectorstamped_base.header, outputvectorstamped_base.vector, torques.vector) #TwistStamped
            converted_twist.header.stamp=rospy.get_rostime()
            converted_twist.header.frame_id="j2s7s300_link_base"

            # publish the converted twist header to ROS
            self.set_velocity(converted_twist) # updates self._command, turns twist into PoseVelocity
            #self._cart_vel_pub.publish(self._command)  # publishes command for Kinova to pick up
            if self.print==True:
                print("command sent", self._command)
            self.print=False
            #print("The velocity twist command I would send is", converted_twist, force)
            #######################################################
            ## This makes a marker of the converted data that is
            ## being sent as a velocity command
            ######################################################
            convertedMarker = Marker()
            convertedMarker.type = Marker.ARROW
            convertedMarker.header=converted_twist.header
            convertedMarker.id = 1
            convertedMarker.ns = "base vector"
            convertedMarker.scale.x = .1 #math.sqrt((converted_twist.twist.linear.x)**2+(converted_twist.twist.linear.y)**2+(converted_twist.twist.linear.z)**2)
            convertedMarker.scale.y = .1
            convertedMarker.scale.z = .1
             # at the origin of the frame
            convertedMarker.pose.position.x = 0
            convertedMarker.pose.position.y = 0
            convertedMarker.pose.position.z = 0
            convertedMarker.color.r = 0 #teal
            convertedMarker.color.g = 1
            convertedMarker.color.b = 1
            convertedMarker.color.a = 1
             #
            point0 = geometry_msgs.msg.Point(0,0,0)
            point1 = geometry_msgs.msg.Point(converted_twist.twist.linear.x,
                                              converted_twist.twist.linear.y,
                                              converted_twist.twist.linear.z)
        
            convertedMarker.points = [point0, point1]
            convertedMarker.pose.orientation.x = converted_twist.twist.linear.x
            convertedMarker.pose.orientation.y = converted_twist.twist.linear.y
            convertedMarker.pose.orientation.z = converted_twist.twist.linear.z
            self._rviz_publisher.publish(convertedMarker)
            #####################################################
            ## End of the marker block
            ####################################################
            
              # Check if the robot is too close to the table
            #collision_checker_node = StateValidity()
            #collision_checker_node.start_collision_checker()

            #rospy.loginfo("end of the timer callback")
            
        except AttributeError as e:
            print("exception is", e)
            print(traceback.format_exc())
            time.sleep(5)
            print("Waiting 5 seconds for init")
            self.set_velocity(self.estop)
            pass
        
    def check_pose(self, data):
        # this is for collision checking, does nothing right now
        self.pose = data

    def deadband_fxn(self, force, threshold):
        self.commandfound=False
        newforce = []
        for i in range(len(force)):
            if abs(force[i]) < threshold:
                newforce.append(0)
            else:
                newforce.append(force[i])
                self.print=True
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

    def stop(self):
        self._controller_timer.shutdown()
        print("Controller timer shut down sent")
        self.twistpublisher.unregister()

    def start(self):
        if self._controller_timer!=None:
            self._controller_timer.shutdown()
    
        self._controller_timer = rospy.Timer(rospy.Duration(self.timer_period), self.timer_callback)
        

if __name__ == '__main__':
    rospy.init_node('forcetorquecontrol')
    # make an instance of the class, which will also run init
    # and start the subscribers
    this_ft_controller = ForceTorqueController(controltype="PD",
                                               K_P=-4.0, K_D=-5.0, threshold=3.0)
    #this_ft_controller = ForceTorqueController(controltype="P",
    #                                           K_P=-400.0)

    this_ft_controller.start()
    #this_controller = JointTorquesController() # instantiate the controller using built-in joint torques
    rospy.spin()
    this_ft_controller.stop()


    
