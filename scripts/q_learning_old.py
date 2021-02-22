#!/usr/bin/env python3

import rospy

#from gazebo_msgs.msg import ModelState, ModelStates
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
#import time
#from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward


class Q_Learning(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_q_learning')

        # Create a publisher to the qmatrix 
        print("publisher for qmatrix...")
        self.q_matrix_pub = rospy.Publisher("/q_learning/QMatix", QMatrix, queue_size=10)

        # Create subscriber for reward of message type QLearningReward
        print("subscriber for reward...")
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_received)
        
        # Create a publisher for robot actions
        print("publisher for robot_action...")
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

    def reward_received(data):
        reward = data.reward
        print("reward is " + str(reward))
        return reward

    def run(self):

        self.robot_action_pub.publish(RobotMoveDBToBlock())

        robot_action_data = RobotMoveDBToBlock()
        robot_action_data.robot_db = "red"
        robot_action_data.block_id = 1
        print("about to publish action")
        self.robot_action_pub.publish(robot_action_data)
        print("just published action")
        
        robot_action_data.robot_db = "green"
        robot_action_data.block_id = 2
        print("about to publish action")
        self.robot_action_pub.publish(robot_action_data)
        print("just published action")
        
        robot_action_data.robot_db = "blue"
        robot_action_data.block_id = 3
        print("about to publish action")
        self.robot_action_pub.publish(robot_action_data)
        print("just published action")
        
        rospy.spin()


if __name__=="__main__":

    node = Q_Learning()
    node.run()
