"""
ROS controller subscriptions.
"""

# pyright: reportMissingTypeStubs=false

from typing import Callable, List, TypeVar

from nav_msgs.msg import Odometry
from rospy_util.controller import Sub
from rospy_util.turtle_pose import TurtlePose
import rospy_util.turtle_pose as tp
from sensor_msgs.msg import Image, LaserScan

from q_learning_project.msg import Actions, ArmResult, RobotMoveDBToBlock


__all__ = (
    "arm_result",
    "image_sensor",
    "laser_scan",
    "odometry",
    "optimal_actions",
    "robot_action",
)

Msg = TypeVar("Msg")


def arm_result(to_msg: Callable[[ArmResult], Msg]) -> Sub[ArmResult, Msg]:
    """
    Receive the result of a command executed by the arm controller node.
    """
    return Sub(
        topic_name="/q_learning/arm_res",
        message_type=ArmResult,
        to_msg=to_msg,
    )


def optimal_actions(
    to_msg: Callable[[List[RobotMoveDBToBlock]], Msg],
) -> Sub[Actions, Msg]:
    """
    Receive the optimal actions to perform from the Q-algorithm node.
    """
    return Sub(
        topic_name="/q_learning/optimal_actions",
        message_type=Actions,
        to_msg=lambda a: to_msg(a.actions),
    )


def robot_action(
    to_msg: Callable[[RobotMoveDBToBlock], Msg],
) -> Sub[RobotMoveDBToBlock, Msg]:
    """
    Receive an action to perform.
    """
    return Sub(
        topic_name="/q_learning/robot_action",
        message_type=RobotMoveDBToBlock,
        to_msg=to_msg,
    )


def odometry(to_msg: Callable[[TurtlePose], Msg]) -> Sub[Odometry, Msg]:
    """
    Receive odometry sensor data.
    """
    return Sub(
        topic_name="/odom",
        message_type=Odometry,
        to_msg=lambda odom: to_msg(tp.from_pose(odom.pose.pose)),
    )


def laser_scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[LaserScan, Msg]:
    """
    Receive LiDAR sensor data.
    """
    return Sub(
        topic_name="/scan",
        message_type=LaserScan,
        to_msg=to_msg,
    )


def image_sensor(to_msg: Callable[[Image], Msg]) -> Sub[Image, Msg]:
    """
    Receive image sensor data.
    """
    return Sub(
        topic_name="/camera/rgb/image_raw",
        message_type=Image,
        to_msg=to_msg,
    )
