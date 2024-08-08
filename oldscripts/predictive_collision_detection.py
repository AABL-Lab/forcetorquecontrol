#!/usr/bin/env python3
# modified from https://answers.ros.org/question/203633/collision-detection-in-python/

import rospy
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class StateValidity():
    def __init__(self):
        # subscribe to joint joint states
        rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')
        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name = ["j2s7s300_joint_1","j2s7s300_joint_2",
                                    "j2s7s300_joint_3","j2s7s300_joint_4",
                                    "j2s7s300_joint_5","j2s7s300_joint_6",
                                    "j2s7s300_joint_7"]
        self.rs.joint_state.position = [0.0, 0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_states_received = False


    def checkCollision(self):
        '''
        check if robot is in collision
        '''
        if self.getStateValidity().valid:
            rospy.loginfo('robot not in collision, all ok!')
        else:
            rospy.logwarn('robot in collision')


    def jointStatesCB(self, msg):
        '''
        update robot state and check for collisions
        '''
        print("joint states received")
        self.rs.joint_state.position = [msg.position[0], msg.position[1],msg.position[2],msg.position[3],msg.position[4],msg.position[5],msg.position[6]]
        self.joint_states_received = True
        self.checkCollision()

    def getStateValidity(self, group_name='acrobat', constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result


#    def start_collision_checker(self):
#        while not self.joint_states_received:
#            rospy.sleep(0.1)
#        rospy.loginfo('joint states received! continue')
#        self.checkCollision()
#        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('collision_checker_node', anonymous=False)
    collision_checker_node = StateValidity()
#    collision_checker_node.start_collision_checker()
