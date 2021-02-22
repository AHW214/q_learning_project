#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from typing import Any, List, Tuple

import rospy
from rospy_util.controller import Cmd, Controller, Sub

from controller.cmd import RobotAction
import controller.cmd as cmd
import controller.sub as sub

### Model ###


@dataclass
class Model:
    actions: List[RobotAction]
    rewards: List[int]


init_model: Model = Model(
    actions=[
        RobotAction(robot_db="green", block_id=2),
        RobotAction(robot_db="blue", block_id=3),
    ],
    rewards=[],
)

init_cmds: List[Cmd[Any]] = [
    cmd.robot_action(
        RobotAction(robot_db="red", block_id=1),
        initial=True,
    ),
]

init: Tuple[Model, List[Cmd[Any]]] = (init_model, init_cmds)


### Messages ###


@dataclass
class Reward:
    reward: int


### Update ###


def update(msg: Reward, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    rewards = [msg.reward, *model.rewards]

    print(rewards)
    print(model.actions)

    if not model.actions:
        return (replace(model, rewards=rewards), cmd.none)

    (action, *rest) = model.actions

    return (
        Model(actions=rest, rewards=rewards),
        [cmd.robot_action(action)],
    )


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, Reward]]:
    return [
        sub.q_learning_reward(lambda msg: Reward(msg.reward)),
    ]


### Run ###


def run() -> None:
    rospy.init_node("q_learning")

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


# from gazebo_msgs.msg import ModelState, ModelStates
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# import time
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward


class Q_Learning(object):
    def __init__(self):

        # initialize this node
        rospy.init_node("turtlebot3_q_learning")

        self.robot_dbs = ["red", "green", "blue"]
        # db model names
        self.db_model_names = {
            "red": "robot_dumbbell_red",
            "green": "robot_dumbbell_green",
            "blue": "robot_dumbbell_blue",
        }

        # Create a publisher to the qmatrix
        print("publisher for qmatrix...")
        self.q_matrix_pub = rospy.Publisher(
            "/q_learning/QMatix", QMatrix, queue_size=10
        )

        # Create subscriber for reward of message type QLearningReward
        print("subscriber for reward...")
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_received)

        # Create a publisher for robot actions
        print("publisher for robot_action...")
        self.robot_action_pub = rospy.Publisher(
            "/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10
        )

    def reward_received(data):
        reward = data.reward
        print("reward is " + str(reward))
        return reward

    def run(self):
        ctrl_c = False
        rate = rospy.Rate(1)
        while not ctrl_c:
            connections = self.robot_action_pub.get_num_connections()
            if connections > 0:
                for k in range(1, 10):
                    robot_action_data = RobotMoveDBToBlock()
                    robot_action_data.robot_db = self.robot_dbs[k % 3]
                    robot_action_data.block_id = 1 + k % 3
                    print("about to publish action")
                    self.robot_action_pub.publish(robot_action_data)
                    print("just published action")
                else:
                    rate.sleep()
        rospy.spin()


if __name__ == "__main__":

    node = Q_Learning()
    node.run()
