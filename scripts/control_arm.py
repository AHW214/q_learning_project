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
        self.turning = False
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
        self.twist.linear.x = -0.05
        self.twist_pub.publish(self.twist)
        rospy.sleep(3)
        self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)

    def face_dumbbell(self, db_dir: float):
        err_min = 0.05
        vel_ang = math.pi / 4

        if abs(db_dir) <= err_min:
            self.turning = False
            self.twist.angular.z = 0
        else:
            self.turning = True
            self.twist.angular.z = vel_ang * db_dir
        self.twist_pub.publish(self.twist)

    def approach_dumbbell(self, db_dist: float):
        distance = 0.25  # stay 1/4 meters away from the dumbbell
        speed = 0.02  # m/s

        if db_dist > distance:  # if we are far enough from the db
            self.twist.linear.x = speed
            self.moving = True
        else:
            self.twist.linear.x = 0
            self.moving = False
        self.twist_pub.publish(self.twist)

    def process_scan(self, data):
        def rad_signed(deg: int) -> float:
            return math.radians(deg - 360 if deg > 180 else deg)

        if not self.turning and not self.moving:
            return

        angled = [*enumerate(data.ranges)]

        front_ranges = [
            (rad_signed(deg), dist)
            for (deg, dist) in angled[:10] + angled[350:]
            if math.isfinite(dist)
        ]

        if not front_ranges:
            print("no front scan data")
            self.resp_pub.publish(ArmResult(error="no front scan data"))
            return

        (dir_min, dist_min) = min(front_ranges)

        print(dir_min, dist_min)

        if self.turning:
            self.face_dumbbell(dir_min)
        if self.moving:
            self.approach_dumbbell(dist_min)
        self.twist_pub.publish(self.twist)

    def pickup_db(self):
        rate = rospy.Rate(1)
        rate.sleep()
        self.moving = False  # boolean for moving toward db
        self.turning = False  # boolean for turning toward db
        self.twist_pub.publish(Twist())  # stop moving
        print("resetting position...")
        self.home_pose()
        # print("setting orientation:")
        # TODO - disabled turning for now
        self.turning = False
        while self.turning == True:
            rospy.sleep(0.1)  # wait for orienting toward db
        print("open gripper...")
        self.open_gripper()
        print("reach_dumbbell..")
        self.reach_dumbbell()
        print("moving toward db...")
        self.moving = True
        while self.moving == True:
            rospy.sleep(0.1)  # Wait for movement to stop
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
        self.home_pose()

    def command_received(self, data: ArmCommand):
        print("command received")
        response = ArmResult()
        if data.command == "up":
            print("Move:picking up")
            self.pickup_db()
            response.result = "up"
        elif data.command == "down":
            print("Move:putting down")
            self.putdown_db()
            response.result = "down"
        else:
            print("Error: command_received: unknown command")
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
