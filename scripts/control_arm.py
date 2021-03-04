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
from q_learning_project.msg import ArmCommand, ArmResult


class Robot(object):
    def __init__(self):

        # initialize this node
        rospy.init_node("control_arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Declare node as a subscriber to the scan topic and
        # set self.process_scan as the function to be used for callback
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber("/q_learning/arm_cmd", ArmCommand, self.command_received)

        # Get a publisher to the cmd_vel topic
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.resp_pub = rospy.Publisher("/q_learning/arm_res", ArmResult, queue_size=10)

        # Create a default twist msg (all values 0)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)
        self.moving = False

    def home_pose(self):
        arm_joint_goal = [0.0, -1.0, 0.3, 0.7]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def open_gripper(self):
        gripper_joint_goal = self.move_group_gripper.get_current_joint_values()
        gripper_joint_goal[0] = 0.019
        gripper_joint_goal[1] = 0.019
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def close_gripper(self):
        gripper_joint_goal = self.move_group_gripper.get_current_joint_values()
        gripper_joint_goal = [0.008, 0.008]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def reach_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal[0] = 0.0
        arm_joint_goal[1] = 0.85
        arm_joint_goal[2] = -0.4
        arm_joint_goal[3] = -0.55
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def lift_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal = [0.0, -0.8, -0.2, 0.3]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(1)

    def set_dumbbell(self):
        arm_joint_goal = self.move_group_arm.get_current_joint_values()
        arm_joint_goal = [0.0, 0.95, -0.75, -0.3]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.open_gripper()
        self.twist.linear.x = -0.07
        self.twist_pub.publish(self.twist)
        rospy.sleep(4)
        self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)

    def approach_dumbbell(self, db_dir: float, db_dist: float):
        err_lin_min = 0.22  # stay 1/4 meters away from the dumbbell
        kp_lin = 0.1

        err_ang_min = 0.005
        kp_ang = math.pi / 2

        twist = Twist()

        if abs(db_dir) > err_ang_min:
            twist.angular.z = kp_ang * db_dir

        if db_dist > err_lin_min:  # if we are far enough from the db
            twist.linear.x = kp_lin * db_dist
        else:
            self.moving = False

        self.twist_pub.publish(twist)

    def process_scan(self, data):
        def rad_signed(deg: int) -> float:
            return math.radians(deg - 360 if deg > 180 else deg)

        if not self.moving:
            return

        angled = [*enumerate(data.ranges)]

        front_ranges = [
            (rad_signed(deg), dist)
            for (deg, dist) in angled[350:] + angled[:10]
            if math.isfinite(dist)
        ]

        if not front_ranges:
            self.resp_pub.publish(ArmResult(error="no front scan data"))
            return

        (dirs, dists) = [*zip(*front_ranges)]

        dir_min = (dirs[0] + dirs[-1]) / 2.0 if len(dirs) > 1 else dirs[0]

        dist_min = min(dists)

        self.approach_dumbbell(dir_min, dist_min)

    def pickup_db(self):
        rate = rospy.Rate(1)
        rate.sleep()
        self.moving = False  # boolean for moving toward db
        self.twist_pub.publish(Twist())  # stop moving

        self.home_pose()

        self.open_gripper()

        self.reach_dumbbell()

        self.moving = True
        while self.moving == True:
            rospy.sleep(0.1)  # Wait for movement to stop

        self.close_gripper()
        rate.sleep()

        self.lift_dumbbell()
        rate.sleep()

    def putdown_db(self):
        self.twist.linear.x = 0
        self.twist_pub.publish(Twist())  # stop

        self.set_dumbbell()
        self.open_gripper()
        self.home_pose()

    def command_received(self, data: ArmCommand):

        response = ArmResult()
        if data.command == "up":
            self.pickup_db()
            response.result = "up"
        elif data.command == "down":
            self.putdown_db()
            response.result = "down"
        else:
            response.error = "unknown command"
        self.resp_pub.publish(response)

    def run(self):
        rate = rospy.Rate(1)
        connections = self.resp_pub.get_num_connections()
        while connections < 1:
            rate.sleep()
            connections = self.resp_pub.get_num_connections()

        rospy.spin()


if __name__ == "__main__":

    node = Robot()
    node.run()
