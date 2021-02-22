#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from typing import Any, List, Tuple

import rospy
from rospy_util.controller import Cmd, Controller, Sub

import controller.cmd as cmd
import controller.sub as sub
from q_learning_project.msg import RobotMoveDBToBlock, QLearningReward

### Model ###


@dataclass
class Model:
    actions: List[RobotMoveDBToBlock]
    rewards: List[QLearningReward]


init_model: Model = Model(
    actions=[
        RobotMoveDBToBlock(robot_db="red", block_id=1),
        RobotMoveDBToBlock(robot_db="green", block_id=2),
        RobotMoveDBToBlock(robot_db="blue", block_id=3),
    ],
    rewards=[],
)


### Update ###


def update(reward: QLearningReward, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    rewards = [reward, *model.rewards]

    print(rewards)

    if not model.actions:
        return (replace(model, rewards=rewards), [])

    (action, *rest) = model.actions
    return (
        Model(actions=rest, rewards=rewards),
        [cmd.robot_action(action)],
    )


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, QLearningReward]]:
    return [
        sub.q_learning_reward(lambda r: r),
    ]


### Run ###


def run() -> None:
    rospy.init_node("q_learning")

    Controller.run(
        model=init_model,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
