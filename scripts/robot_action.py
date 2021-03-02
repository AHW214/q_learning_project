#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

from cv_bridge import CvBridge
import rospy
from rospy_util.controller import Cmd, Controller, Sub
from rospy_util.turtle_pose import TurtlePose

# TODO - packages to clean up imports
import controller.cmd as cmd
import controller.sub as sub
from data.image import ImageBGR, ImageROS
import data.image as image
from perception.color import HSV_CV2, Range
import perception.color as color
import perception.ocr as ocr
from q_learning_project.msg import RobotMoveDBToBlock
from util import compose

### Model ###


class Block(IntEnum):
    one = 1
    two = 2
    three = 3


@dataclass
class RobotAction:
    block: Block
    dumbbell: Range[HSV_CV2]


class State(IntEnum):
    stop = 1
    locate = 2
    face = 3
    approach = 4


@dataclass
class Model:
    current_action: Optional[RobotAction]
    pending_actions: List[RobotAction]
    have_dumbbell: bool
    last_image: Optional[ImageBGR]
    obstacle_dist: float
    state: State
    cv_bridge: CvBridge
    keras_pipeline: ocr.Pipeline


init_model: Model = Model(
    current_action=None,
    pending_actions=[],
    have_dumbbell=False,
    last_image=None,
    obstacle_dist=math.inf,
    state=State.stop,
    cv_bridge=CvBridge(),
    keras_pipeline=ocr.Pipeline(scale=1.0),
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Action:
    action: RobotMoveDBToBlock


@dataclass
class Image:
    image: ImageBGR


@dataclass
class Odom:
    pose: TurtlePose


@dataclass
class Scan:
    ranges: List[float]


Msg = Union[Action, Image, Odom, Scan]


### Update ###

KP_ANG = math.pi / 8.0
KP_LIN = 0.5

DIST_STOP = 0.4
DIR_BLOCKS = 0.0

RED: Range[HSV_CV2] = color.hsv_range(
    lower=(0, 90, 60),
    upper=(40, 100, 80),
)

GREEN: Range[HSV_CV2] = color.hsv_range(
    lower=(100, 90, 60),
    upper=(140, 100, 80),
)


BLUE: Range[HSV_CV2] = color.hsv_range(
    lower=(220, 90, 60),
    upper=(260, 100, 80),
)


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(msg, Action):
        if (action := parse_action(msg.action)) is None:
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

    if isinstance(msg, Scan):
        ranges_sanitized = [r for r in msg.ranges if math.isfinite(r) and r > 0.2]
        closest = math.inf if not ranges_sanitized else min(ranges_sanitized)

        new_model = replace(model, obstacle_dist=closest)
        return (new_model, cmd.none)

    if (
        model.last_image is None
        or model.current_action is None
        or model.state == State.stop
    ):
        return (model, cmd.none)

    (new_state, cmds) = retrieve_dumbbell(
        model.state,
        model.last_image,
        model.obstacle_dist,
        model.current_action.dumbbell,
    )

    return (replace(model, state=new_state), cmds)


def retrieve_dumbbell(
    state: State,
    img: ImageBGR,
    dist: float,
    clr: Range[HSV_CV2],
) -> Tuple[State, List[Cmd[Any]]]:
    if (img_pos := color.locate_color(clr, img)) is None:
        return (State.locate, [cmd.turn(0.2)])

    if state == State.locate:
        return (State.face, [cmd.stop])

    (cx, _) = img_pos
    err_ang = 1.0 - 2.0 * (cx / img.width)

    if state == State.face:
        if abs(err_ang) < 0.05:
            return (State.approach, [cmd.stop])

        vel_ang = KP_ANG * err_ang
        return (state, [cmd.turn(vel_ang)])

    if state == State.approach:
        if dist < DIST_STOP:
            return (State.stop, [cmd.stop])

        err_lin = min(dist - DIST_STOP, 1.0)
        vel_lin = KP_LIN * err_lin
        vel_ang = KP_ANG * err_ang

        return (state, [cmd.velocity(linear=vel_lin, angular=vel_ang)])

    return (state, cmd.none)


def parse_action(action: RobotMoveDBToBlock) -> Optional[RobotAction]:
    if (block := parse_block(action.block_id)) is None or (
        dumbbell := parse_dumbbell(action.robot_db)
    ) is None:
        return None

    return RobotAction(block, dumbbell)


def parse_block(block_id: int) -> Optional[Block]:
    return Block(block_id) if block_id in [b.value for b in Block] else None


def parse_dumbbell(clr: str) -> Optional[Range[HSV_CV2]]:
    return (
        RED
        if clr == "red"
        else GREEN
        if clr == "green"
        else BLUE
        if clr == "blue"
        else None
    )


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    return [
        sub.robot_action(Action),
        sub.image_sensor(toImageCV2(model.cv_bridge)),
        sub.odometry(Odom),
        sub.laser_scan(lambda s: Scan(s.ranges)),
    ]


# def obstacle(scan: LaserScan) -> Msg:
#     ranges: List[float] = [r for r in scan.ranges if math.isfinite(r) and r > 0.2]

#     if not ranges:
#         return Void()

#     return Obstacle(distance=min(ranges))


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
