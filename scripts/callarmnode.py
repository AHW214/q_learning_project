#!/usr/bin/env python3

# Node Robot in file movearm.py
# Robot.puckup_db() - orient toward the dumbbell in front of the robot, within .5 meters of robot, move forward, and raise overhead
# Robot.putdown_db() - put down the dumbbell where robot is, back away slightly
#
# to operate, need: roscore, roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
#                   roslaunch turtlebot3_manipulation_moveit_config move_group.launch
#                   rosrun q_learning_project movearm.py


import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from q_learning_project.msg import RobotMoveDBToBlock

class CallArm(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('CallArm')

        self.arm_pub = rospy.Publisher("/q_learning/cmd_arm", RobotMoveDBToBlock, queue_size=10)
        rospy.Subscriber("q_learning/res_arm", RobotMoveDBToBlock, self.command_received)
        
    def command_received(self,data):
        self.response = data.robot_db

    def run(self):
        rate = rospy.Rate(1)
        self.response = ""
        arm_command = RobotMoveDBToBlock()
        arm_command.robot_db = "up"
        self.arm_pub.publish(arm_command)
        while (self.response != "done" ):
            rate.sleep()

        arm_command.robot_db = "down"
        self.response =""
        self.arm_pub.publish(arm_command)
        while (self.response != "done" ):
            rate.sleep()
            
        rospy.spin()

if __name__=="__main__":

    node=CallArm()
    node.run()
