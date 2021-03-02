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
from q_learning_project.msg import ArmCommand ArmResponse

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('movearm')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        #Declare node as a subscriber to the scan topic and
        # set self.process_scan as the function to be used for callback
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber("/q_learning/cmd_arm", ArmCommand, self.command_received)
        
        #Get a publisher to the cmd_vel topic
        self.twist_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size = 10)
        self.resp_pub = rospy.Publisher("/q_learning/res_arm", ArmResponse, queue_Size=10)
        
        #Create a default twist msg (all values 0)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear = lin, angular = ang)
        self.turning = False
        self.moving = False
        
    def open_gripper(self):
        gripper_joint_goal = self.move_group_gripper.get_current_joint_values()
        gripper_joint_goal[0] = 0.019
        gripper_joint_goal[1] = 0.019
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()        
        
    def close_gripper(self):
        gripper_joint_goal = self.move_group_gripper.get_current_joint_values()
        gripper_joint_goal = [-0.005, -0.005]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()        

    def reach_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal[0]=0.0
        arm_joint_goal[1]=0.85
        arm_joint_goal[2]=-0.4
        arm_joint_goal[3]=-0.55
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def lift_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal=[0.0,-.8,-0.2,0.3]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(1)

    def set_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal=[0.0,0.95,-0.75,-0.3]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.open_gripper()
        self.twist.linear.x = -.05
        self.twist_pub.publish(self.twist)
        rospy.sleep(2)
        self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)

    def face_dumbbell(self,data):
        rotspeed = math.pi/4 # 90 degrees per second
        range_data = np.array(data.ranges) #convert to numpy array
        minval = min(range_data) # closest point in space
        vals = np.argwhere(range_data < .5) # must be within .5m of db
        if len(vals)==0:
            mindir = np.argmin(range_data) # find direction to db
        else:
            for i in range(len(vals)):
                if vals[i] > 180:
                    vals[i] -= 360
        mindir = np.average(vals)
        if np.isnan(mindir):
            mindir = 4
            print("Error in mindir calculation")
            
        if (np.abs(mindir) <= 2.5):
            self.turning = False
            self.twist.angular.z = 0
        else:
            self.turning = True
            self.twist.angular.z = rotspeed*(mindir)/180
            if abs(self.twist.angular.z)<.01:
                self.turning = False
                self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)
            
    def approach_dumbbell(self,data):
        distance = 0.22 # stay 1/4 meters away from the dumbbell
        speed = 0.02 # m/s
        range_data = np.array(data.ranges) #convert to numpy array
        minval = min(range_data) # closest point in space
        if data.ranges[0] > distance: # if we are far enough from the db
            self.twist.linear.x = speed
            self.moving = True
            self.turning = True
        else:
            self.twist.linear.x = 0
            self.moving = False
            self.turning = False
        self.twist_pub.publish(self.twist)
            
    def process_scan(self,data):
        if (self.turning == True):
            self.face_dumbbell(data)
        if (self.moving == True):
            self.approach_dumbbell(data)
        self.twist_pub.publish(self.twist)
        
    def pickup_db(self):
        rate = rospy.Rate(1)
        rate.sleep()
        self.moving = False # boolean for moving toward db
        self.turning = False # boolean for turning toward db
        self.twist_pub.publish(Twist()) # stop moving
        print("resetting position...")
        self.lift_dumbbell()
        print("open gripper...")
        self.open_gripper()
        print("setting orientation:")
        self.turning = True
        while (self.turning == True):
            rospy.sleep(0.1) # wait for orienting toward db
        print("open gripper...")
        self.open_gripper()
        print("reach_dumbbell..")
        self.reach_dumbbell()
        print("moving toward db...")
        self.moving = True
        while (self.moving == True):
            rospy.sleep(0.1) # Wait for movement to stop
        print("close gripper")
        self.close_gripper()
        rate.sleep()
        print("lift dumbbell")
        self.lift_dumbbell()
        rate.sleep()

    def putdown_db(self):
        self.twist.linear.x = 0
        self.twist_pub.publish(Twist())  # stop
        print("putting db down..")
        self.set_dumbbell()
        self.open_gripper()

    def command_received(self,data):
        self.command = data.command
        self.iter = data.iteration
        response_data = ArmResponse()
        if self.command == "up":
            print("Move:picking up")
            self.pickup_db()
            response_data.response = "done"
            response_data.iteration = self.iter
        elif self.command == "down":
            print("Move:putting down")
            self.putdown_db()
            response_data.response = "done"
            response_data.iteration = self.iter
        else:
            print("Error: command_received: unknown command")
            response_data.response = "error"
            response_data.iteration = self.iter
        self.resp_pub.publish(response_data)
            
    def run(self):
        rospy.spin()

if __name__=="__main__":

    node=Robot()
    node.run()
