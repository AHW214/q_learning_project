#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import rospy
from rospy_util.controller import Cmd, Controller, Sub

# TODO - packages to clean up imports
import controller.cmd as cmd
from controller.sub import RobotAction
import controller.sub as sub
from data.image import ImageBGR, ImageROS
import data.image as image
import perception.color as color
import perception.ocr as ocr
from util import compose

### Model ###

Locator = Callable[[ImageBGR], Optional[Tuple[int, int]]]


@dataclass
class ActionLocators:
    locator_block: Locator
    locator_dumbbell: Locator


class State(IntEnum):
    stop = 1
    locate = 2
    face = 3
    approach = 4


@dataclass
class Model:
    current_action: Optional[ActionLocators]
    pending_actions: List[ActionLocators]
    have_dumbbell: bool
    last_image: Optional[ImageBGR]
    state: State
    cv_bridge: CvBridge
    keras_pipeline: ocr.Pipeline


init_model: Model = Model(
    current_action=None,
    pending_actions=[],
    have_dumbbell=False,
    last_image=None,
    state=State.stop,
    cv_bridge=CvBridge(),
    keras_pipeline=ocr.Pipeline(scale=1.0),
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Action:
    action: RobotAction


@dataclass
class Image:
    image: ImageBGR


@dataclass
class Obstacle:
    distance: float


@dataclass
class Void:
    pass


Msg = Union[Action, Image, Obstacle, Void]


### Update ###

KP_ANG = math.pi / 8.0
KP_LIN = 0.5

DIST_STOP = 0.4
DIR_BLOCKS = 0.0


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(msg, Action):
        if (action := locators_from_action(model.keras_pipeline, msg.action)) is None:
            print(f"invalid action: {msg.action}")
            return (model, cmd.none)

        (new_action, new_pending, new_state) = (
            (action, model.pending_actions, State.locate)
            if model.current_action is None
            else (
                model.current_action,
                [*model.pending_actions, action],
                model.state,
            )
        )

        new_model = replace(
            model,
            current_action=new_action,
            pending_actions=new_pending,
            state=new_state,
        )

        return (new_model, cmd.none)

    if isinstance(msg, Image):
        new_model = replace(model, last_image=msg.image)
        return (new_model, cmd.none)

    if (
        isinstance(msg, Void)
        or model.last_image is None
        or model.current_action is None
        or model.state == State.stop
    ):
        return (model, cmd.none)

    locator = (
        model.current_action.locator_block
        if model.have_dumbbell
        else model.current_action.locator_dumbbell
    )

    location = locator(model.last_image)

    if location is None:
        return (
            (model, [cmd.turn(0.2)])
            if model.state == State.locate
            else (replace(model, state=State.locate), cmd.none)
        )

    if model.state == State.locate:
        return (replace(model, state=State.face), cmd.none)

    (cx, _) = location
    err_ang = 1.0 - 2.0 * (cx / model.last_image.width)

    if model.state == State.face:
        if abs(err_ang) < 0.05:
            return (replace(model, state=State.approach), [cmd.stop])

        vel_ang = KP_ANG * err_ang

        return (model, [cmd.turn(vel_ang)])

    if model.state == State.approach:
        if msg.distance < DIST_STOP:
            if model.have_dumbbell:
                # TODO - place dumbbell down

                if not model.pending_actions:
                    return (
                        replace(
                            model,
                            current_action=None,
                            state=State.stop,
                        ),
                        [cmd.stop],
                    )

                (new_action, *new_pending) = model.pending_actions

                return (
                    replace(
                        model,
                        current_action=new_action,
                        pending_actions=new_pending,
                        state=State.locate,
                    ),
                    cmd.none,
                )

            # TODO - pick dumbbell up

            return (
                replace(
                    model,
                    have_dumbbell=True,
                    state=State.locate,
                ),
                cmd.none,
            )

        err_lin = msg.distance - DIST_STOP

        vel_ang = KP_ANG * err_ang
        vel_lin = KP_LIN * err_lin

        return (model, [cmd.velocity(linear=vel_lin, angular=vel_ang)])

    return (model, cmd.none)


def locators_from_action(
    pipeline: ocr.Pipeline,
    action: RobotAction,
) -> Optional[ActionLocators]:
    if (loc_block := block_locator(pipeline, action.block_id)) is None or (
        loc_dumbbell := dumbbell_locator(action.robot_db)
    ) is None:
        return None

    return ActionLocators(loc_block, loc_dumbbell)


def block_locator(pipeline: ocr.Pipeline, id: int) -> Optional[Locator]:
    return partial(ocr.locate_number, pipeline, id) if id >= 1 and id <= 3 else None


def dumbbell_locator(clr: str) -> Optional[Locator]:
    return (
        color.locate_red
        if clr == "red"
        else color.locate_green
        if clr == "green"
        else color.locate_blue
        if clr == "blue"
        else None
    )


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    return [
        sub.robot_action(Action),
        sub.image_sensor(toImageCV2(model.cv_bridge)),
        sub.laser_scan(obstacle),
    ]


def obstacle(scan: LaserScan) -> Msg:
    ranges: List[float] = [r for r in scan.ranges if math.isfinite(r) and r > 0.2]

    if not ranges:
        return Void()

    return Obstacle(distance=min(ranges))


def toImageCV2(cvBridge: CvBridge) -> Callable[[ImageROS], Msg]:
    return compose(Image, partial(image.from_ros_image, cvBridge))


### Run ###


def run() -> None:
    rospy.init_node("q_learning_robot_action")

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
