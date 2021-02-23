# pyright: reportMissingTypeStubs=false

from typing import Callable, TypeVar

from rospy_util.controller.sub import Sub, none

from q_learning_project.msg import QLearningReward

__all__ = (
    "none",
    "q_learning_reward",
)

Msg = TypeVar("Msg")


def q_learning_reward(
    to_msg: Callable[[QLearningReward], Msg],
) -> Sub[QLearningReward, Msg]:
    return Sub(
        topic_name="/q_learning/reward",
        message_type=QLearningReward,
        to_msg=to_msg,
    )
