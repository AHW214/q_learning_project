# pyright: reportMissingTypeStubs=false

from typing import Callable, TypeVar

from q_learning_project.msg import RobotMoveDBToBlock
from rospy_util.controller import Sub
from sensor_msgs.msg import LaserScan, Image

Action = RobotMoveDBToBlock
Msg = TypeVar("Msg")


def robot_action(to_msg: Callable[[Action], Msg]) -> Sub[Action, Msg]:
    return Sub(
        topic_name="/q_learning/robot_action",
        message_type=Action,
        to_msg=to_msg,
    )


def laser_scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[LaserScan, Msg]:
    return Sub(
        topic_name="/scan",
        message_type=LaserScan,
        to_msg=to_msg,
    )


def image_sensor(to_msg: Callable[[Image], Msg]) -> Sub[Image, Msg]:
    return Sub(
        topic_name="/camera/rgb/image_raw",
        message_type=Image,
        to_msg=to_msg,
    )
