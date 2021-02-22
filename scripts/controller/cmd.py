# pyright: reportMissingTypeStubs=false

from rospy_util.controller.cmd import Cmd, none

from q_learning_project.msg import QMatrix, RobotMoveDBToBlock

__all__ = (
    "RobotAction",
    "none",
    "robot_action",
    "update_q_matrix",
)

RobotAction = RobotMoveDBToBlock


def robot_action(action: RobotAction, initial: bool = False) -> Cmd[RobotAction]:
    return Cmd(
        topic_name="/q_learning/robot_action",
        message_type=RobotAction,
        message_value=action,
        latch_publisher=initial,
    )


def update_q_matrix(q_matrix: QMatrix) -> Cmd[QMatrix]:
    return Cmd(
        topic_name="/q_learning/q_matrix",
        message_type=QMatrix,
        message_value=q_matrix,
    )
