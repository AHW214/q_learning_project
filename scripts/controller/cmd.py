# pyright: reportMissingTypeStubs=false

from typing import Any, List

from geometry_msgs.msg import Twist, Vector3
from rospy_util.controller import Cmd


def twist_from_velocities(linear_x: float, angular_z: float) -> Twist:
    return Twist(
        angular=Vector3(x=0.0, y=0.0, z=angular_z),
        linear=Vector3(x=linear_x, y=0.0, z=0.0),
    )


def velocity(linear: float, angular: float) -> Cmd[Twist]:
    return Cmd(
        topic_name="/cmd_vel",
        message_type=Twist,
        message_value=twist_from_velocities(linear, angular),
    )


def turn(vel_angular: float) -> Cmd[Twist]:
    return velocity(linear=0.0, angular=vel_angular)


def drive(vel_linear: float) -> Cmd[Twist]:
    return velocity(linear=vel_linear, angular=0.0)


stop: Cmd[Twist] = velocity(linear=0.0, angular=0.0)

none: List[Cmd[Any]] = []  # TODO: add to rospy_util
