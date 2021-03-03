# pyright: reportMissingTypeStubs=false


from geometry_msgs.msg import Twist, Vector3
from rospy_util.controller.cmd import Cmd, none

from q_learning_project.msg import ArmCommand

__all__ = (
    "drive",
    "none",
    "pickup_dumbbell",
    "place_dumbbell",
    "stop",
    "turn",
    "velocity",
)


def command_arm(cmd: str) -> Cmd[ArmCommand]:
    return Cmd(
        topic_name="/q_learning/arm_cmd",
        message_type=ArmCommand,
        message_value=ArmCommand(cmd),
        latch_publisher=True,
    )


place_dumbbell: Cmd[ArmCommand] = command_arm("down")

lift_dumbbell: Cmd[ArmCommand] = command_arm("up")


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
