# pyright: reportMissingTypeStubs=false

from rospy_util.controller.cmd import Cmd, none

from q_learning import Action, QMatrix, action as act, q_matrix as qm
import q_learning_project.msg as msg

__all__ = (
    "RobotAction",
    "none",
    "robot_action",
    "update_q_matrix",
)


def robot_action(
    action: Action,
    initial: bool = False,
) -> Cmd[msg.RobotMoveDBToBlock]:
    return Cmd(
        topic_name="/q_learning/robot_action",
        message_type=msg.RobotMoveDBToBlock,
        message_value=act.to_msg(action),
        latch_publisher=initial,
    )


def publish_q_matrix(
    q_matrix: QMatrix,
    initial: bool = False,
) -> Cmd[msg.QMatrix]:
    return Cmd(
        topic_name="/q_learning/q_matrix",
        message_type=msg.QMatrix,
        message_value=qm.to_msg(q_matrix),
    )
