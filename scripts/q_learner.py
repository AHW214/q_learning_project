#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from typing import Any, List, Optional, Tuple

import rospy
from rospy_util.controller import Cmd, Controller, Sub

import controller.cmd as cmd
import controller.sub as sub
from q_learning import (
    Action,
    ActionMatrix,
    QMatrix,
    State,
    action_matrix as am,
    q_matrix as qm,
    state as st,
)

### Model ###


@dataclass
class Model:
    action_matrix: ActionMatrix
    q_matrix: QMatrix
    current_state: State
    last_action: Optional[Action]
    no_update_count: int


def init() -> Tuple[Model, List[Cmd[Any]]]:
    action_matrix = am.create(st.all_possible_pairs())
    action = am.random_action(st.initial, action_matrix)

    q_matrix = qm.init()

    (last_action, cmd_action) = (
        (action, [cmd.robot_action(action, initial=True)])
        if action is not None
        else (None, cmd.none)
    )

    model = Model(
        action_matrix=action_matrix,
        q_matrix=q_matrix,
        current_state=st.initial,
        last_action=last_action,
        no_update_count=0,
    )

    return (model, [cmd.publish_q_matrix(q_matrix, initial=True), *cmd_action])


### Messages ###


@dataclass
class Reward:
    reward: int


### Update ###

CONVERGENCE_THRESH: int = 50
LEARNING_RATE: float = 1.0
DISCOUNT_FACTOR: float = 0.5


def update(msg: Reward, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    rospy.sleep(1)

    if converged(model):
        print("converged")
        return (model, cmd.none)

    if model.last_action is None:
        print("no action")
        return (model, cmd.none)

    step = qm.update(
        state=model.current_state,
        action=model.last_action,
        reward=msg.reward,
        learning_rate=LEARNING_RATE,
        discount_factor=DISCOUNT_FACTOR,
        q_matrix=model.q_matrix,
    )

    if step is None:
        print("bad step")
        return (model, cmd.none)

    (q_matrix, next_state, reward_old, reward_new) = step

    print(f"reward: {msg.reward}; old reward: {reward_old}; new reward: {reward_new}")

    no_update_count = model.no_update_count + 1 if reward_new == reward_old else 0
    new_state = st.initial if st.all_at_blocks(next_state) else next_state
    new_action = am.random_action(new_state, model.action_matrix)

    cmd_action = [cmd.robot_action(new_action)] if new_action is not None else cmd.none

    new_model = replace(
        model,
        q_matrix=q_matrix,
        current_state=new_state,
        last_action=new_action,
        no_update_count=no_update_count,
    )

    return (new_model, [cmd.publish_q_matrix(q_matrix), *cmd_action])


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Reward]]:
    return (
        [sub.q_learning_reward(lambda msg: Reward(msg.reward))]
        if not converged(model)
        else sub.none
    )


def converged(model: Model) -> bool:
    return model.no_update_count >= CONVERGENCE_THRESH


### Run ###


def run() -> None:
    rospy.init_node("q_learning")

    Controller.run(
        init=init(),
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
