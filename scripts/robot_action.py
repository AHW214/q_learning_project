#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

from cv_bridge import CvBridge
import rospy
import rospy_util.mathf as mathf
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


@dataclass
class LocateDumbbell:
    pass


@dataclass
class FaceDumbbell:
    pass


@dataclass
class ApproachDumbbell:
    pass


DumbbellState = Union[
    LocateDumbbell,
    FaceDumbbell,
    ApproachDumbbell,
]


@dataclass
class FaceBlocks:
    pass


@dataclass
class LocateBlock:
    num_adjust: int


@dataclass
class FaceBlock:
    direction: float
    num_adjust: int


@dataclass
class ApproachBlock:
    num_adjust: int


BlockState = Union[
    FaceBlocks,
    LocateBlock,
    FaceBlock,
    ApproachBlock,
]


@dataclass
class Wait:
    pass


@dataclass
class Retrieve:
    action: RobotAction
    state: DumbbellState


@dataclass
class Place:
    action: RobotAction
    state: BlockState


State = Union[Wait, Retrieve, Place]


@dataclass
class Model:
    pending_actions: List[RobotAction]
    last_image: Optional[ImageBGR]
    obstacle_dist: float
    block_direction: Optional[float]  # TODO - refactor
    state: State
    cv_bridge: CvBridge
    keras_pipeline: ocr.Pipeline


init_model: Model = Model(
    pending_actions=[],
    last_image=None,
    obstacle_dist=math.inf,
    block_direction=None,
    state=Wait(),
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
DIR_BLOCKS = math.pi

# https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
FOV_HORIZ = math.radians(62.2)

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

        (new_pending, new_state) = (
            ([], Retrieve(action, LocateDumbbell()))
            if isinstance(model.state, Wait)
            else ([*model.pending_actions, action], model.state)
        )

        new_model = replace(model, pending_actions=new_pending, state=new_state)

        return (new_model, cmd.none)

    if isinstance(msg, Image):
        new_model = replace(model, last_image=msg.image)
        return (new_model, cmd.none)

    if isinstance(msg, Scan):
        front = msg.ranges[:60] + msg.ranges[300:]
        ranges_sanitized = [r for r in front if math.isfinite(r) and r > 0.2]
        closest = math.inf if not ranges_sanitized else min(ranges_sanitized)

        new_model = replace(model, obstacle_dist=closest)
        return (new_model, cmd.none)

    if isinstance(model.state, Wait) or model.last_image is None:
        return (model, cmd.none)

    print(model.state)

    if isinstance(model.state, Retrieve):
        print("retrieve")
        (new_state, cmds) = retrieve_dumbbell(
            state=model.state,
            img=model.last_image,
            dist=model.obstacle_dist,
        )

        return (replace(model, state=new_state), cmds)

    print("place")

    (new_state, cmds) = place_dumbbell(
        state=model.state,
        img=model.last_image,
        dist=model.obstacle_dist,
        yaw=msg.pose.yaw,
        pline=model.keras_pipeline,
    )

    return (
        replace(
            model,
            state=new_state,
        ),
        cmds,
    )


# TODO - call on image/scan message (not odom)
def retrieve_dumbbell(
    state: Retrieve,
    img: ImageBGR,
    dist: float,
) -> Tuple[State, List[Cmd[Any]]]:
    if (img_pos := color.locate_color(state.action.dumbbell, img)) is None:
        return (replace(state, state=LocateDumbbell()), [cmd.turn(0.2)])

    if isinstance(state.state, LocateDumbbell):
        return (replace(state, state=FaceDumbbell()), [cmd.stop])

    (cx, _) = img_pos
    err_ang = 1.0 - 2.0 * (cx / img.width)

    if isinstance(state.state, FaceDumbbell):
        if abs(err_ang) < 0.05:
            return (replace(state, state=ApproachDumbbell()), [cmd.stop])

        vel_ang = KP_ANG * err_ang
        return (state, [cmd.turn(vel_ang)])

    if dist < DIST_STOP:
        return (Place(state.action, FaceBlocks()), [cmd.stop])

    err_lin = min(dist - DIST_STOP, 1.0)
    vel_lin = min(KP_LIN * err_lin, 0.1)
    vel_ang = KP_ANG * err_ang

    return (state, [cmd.velocity(linear=vel_lin, angular=vel_ang)])


def place_dumbbell(
    state: Place,
    img: ImageBGR,
    dist: float,
    yaw: float,
    pline: ocr.Pipeline,
) -> Tuple[State, List[Cmd[Any]]]:
    if isinstance(state.state, FaceBlocks):
        if abs(err_ang := (DIR_BLOCKS - yaw) / math.pi) < 0.05:
            return (replace(state, state=LocateBlock(num_adjust=1)), [cmd.stop])

        vel_ang = mathf.sign(err_ang) * max(0.5 * err_ang, 0.3)
        return (state, [cmd.turn(vel_ang)])

    if isinstance(state.state, LocateBlock):
        if (img_pos := ocr.locate_number(pline, state.action.block.value, img)) is None:
            print("rip")
            return (state, cmd.none)

        print(img_pos)

        (cx, _) = img_pos
        err_ang = 0.5 - (cx / img.width)
        yaw_off = FOV_HORIZ * err_ang
        dir_block = yaw + yaw_off

        print(dir_block)

        return (
            replace(
                state,
                state=FaceBlock(
                    direction=dir_block,
                    num_adjust=state.state.num_adjust,
                ),
            ),
            cmd.none,
        )

    if isinstance(state.state, FaceBlock):
        err_ang = state.state.direction - yaw

        if abs(err_ang) < 0.05:
            return (
                replace(state, state=ApproachBlock(state.state.num_adjust)),
                [cmd.stop],
            )

        vel_ang = mathf.sign(err_ang) * max(0.5 * err_ang, 0.1)

        return (state, [cmd.turn(vel_ang)])

    if dist < 1.5 * state.state.num_adjust:
        return (
            replace(state, state=LocateBlock(num_adjust=state.state.num_adjust - 1)),
            [cmd.stop],
        )

    if dist < DIST_STOP:
        # TODO place dumbbell

        return (state, [cmd.stop])

    err_lin = min(dist - 0.3, 1.0)
    vel_lin = 0.4 * err_lin

    return (state, [cmd.drive(vel_lin)])


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
