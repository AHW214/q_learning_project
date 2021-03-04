"""
ROS controller commands.
"""

# pyright: reportMissingTypeStubs=false


from geometry_msgs.msg import Twist, Vector3
from rospy_util.controller.cmd import Cmd, none

from q_learning_project.msg import ArmCommand

__all__ = (
    "drive",
    "none",
    "lift_dumbbell",
    "place_dumbbell",
    "stop",
    "turn",
    "velocity",
)


def command_arm(cmd: str) -> Cmd[ArmCommand]:
    """
    Send the given command to the arm controller node.
    """
    return Cmd(
        topic_name="/q_learning/arm_cmd",
        message_type=ArmCommand,
        message_value=ArmCommand(cmd),
        latch_publisher=True,
    )


# Instruct the robot to lift the dumbbell.

lift_dumbbell: Cmd[ArmCommand] = command_arm("up")

# Instruct the robot to place the dumbbell.

place_dumbbell: Cmd[ArmCommand] = command_arm("down")


def velocity(linear: float, angular: float) -> Cmd[Twist]:
    """
    Instruct the robot to move with the given velocities.
    """
    return Cmd(
        topic_name="/cmd_vel",
        message_type=Twist,
        message_value=twist_from_velocities(linear, angular),
    )


def twist_from_velocities(linear_x: float, angular_z: float) -> Twist:
    """
    Create a twist message from the given velocities.
    """
    return Twist(
        angular=Vector3(x=0.0, y=0.0, z=angular_z),
        linear=Vector3(x=linear_x, y=0.0, z=0.0),
    )


def turn(vel_angular: float) -> Cmd[Twist]:
    """
    Instruct the robot to turn with the given angular velocity.
    """
    return velocity(linear=0.0, angular=vel_angular)


def drive(vel_linear: float) -> Cmd[Twist]:
    """
    Instruct the robot to drive forward with the given linear velocity.
    """
    return velocity(linear=vel_linear, angular=0.0)


# Instruct the robot to stop moving.

stop: Cmd[Twist] = velocity(linear=0.0, angular=0.0)
