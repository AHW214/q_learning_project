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
from sensor_msgs.msg import LaserScan

# TODO - packages to clean up imports
import controller.cmd as cmd
import controller.sub as sub
from data.image import ImageBGR, ImageROS
import data.image as image
from perception.color import HSV_CV2, Range
import perception.color as color
import perception.ocr as ocr
from q_learning_project.msg import RobotMoveDBToBlock
from util import compose, head

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
class FaceDumbbell:
    pass


@dataclass
class ApproachDumbbell:
    pass


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


State = Union[
    FaceDumbbell,
    ApproachDumbbell,
    FaceBlocks,
    LocateBlock,
    FaceBlock,
    ApproachBlock,
]


@dataclass
class Model:
    state: State
    actions: List[RobotAction]

    image: Optional[ImageBGR]

    cv_bridge: CvBridge
    keras_pipeline: ocr.Pipeline


init_model: Model = Model(
    state=LocateBlock(1),
    actions=[],
    image=None,
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
    obstacle_dist: float


Msg = Union[
    Action,
    Image,
    Odom,
    Scan,
]


### Update ###

KP_ANG = math.pi / 8.0
KP_LIN = 0.5

DIST_STOP = 0.4
DIR_BLOCKS = math.pi

# https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
FOV_IMAGE = math.radians(62.2)
FOV_LIDAR = 60

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

        actions = [*model.actions, action]

        return (replace(model, actions=actions), cmd.none)

    if (action := head(model.actions)) is None:
        return (model, cmd.none)

    dumbell_alignment_error: Callable[[ImageBGR], Optional[float]] = partial(
        error_from_center,
        partial(
            color.locate_color,
            action.dumbbell,
        ),
    )

    block_alignment_error: Callable[[ImageBGR], Optional[float]] = partial(
        error_from_center,
        partial(
            ocr.locate_number,
            model.keras_pipeline,
            action.block.value,
        ),
    )

    print(model.state)

    if isinstance(msg, Odom):
        if isinstance(model.state, FaceBlocks):
            if abs(err_ang := (DIR_BLOCKS - msg.pose.yaw) / math.pi) < 0.05:
                return (replace(model, state=LocateBlock(num_adjust=1)), [cmd.stop])

            # TODO - add helper
            vel_ang = mathf.sign(err_ang) * max(0.5 * err_ang, 0.3)
            return (model, [cmd.turn(vel_ang)])

        if isinstance(model.state, LocateBlock) and model.image is not None:
            if (err_ang := block_alignment_error(model.image)) is None:
                # TODO - rotate more
                return (model, cmd.none)

            yaw_off = FOV_IMAGE * err_ang
            dir_block = msg.pose.yaw + yaw_off

            return (
                replace(
                    model,
                    state=FaceBlock(
                        direction=dir_block,
                        # TODO - names
                        num_adjust=model.state.num_adjust,
                    ),
                ),
                cmd.none,
            )

        if isinstance(model.state, FaceBlock):
            err_ang = model.state.direction - msg.pose.yaw

            if abs(err_ang) < 0.05:
                return (
                    # TODO - names
                    replace(model, state=ApproachBlock(model.state.num_adjust)),
                    [cmd.stop],
                )

            # TODO - add helper
            vel_ang = mathf.sign(err_ang) * max(0.5 * err_ang, 0.1)

            return (model, [cmd.turn(vel_ang)])

        return (model, cmd.none)

    if isinstance(msg, Scan):
        if isinstance(model.state, ApproachBlock):
            if msg.obstacle_dist < 1.5 * model.state.num_adjust:
                return (
                    # TODO - names
                    replace(
                        model, state=LocateBlock(num_adjust=model.state.num_adjust - 1)
                    ),
                    [cmd.stop],
                )

            if msg.obstacle_dist < DIST_STOP:
                # TODO - place dumbbell
                return (replace(model, actions=model.actions[1:]), [cmd.stop])

            # TODO - add helper
            err_lin = min(msg.obstacle_dist - 0.3, 1.0)
            vel_lin = 0.4 * err_lin

            return (model, [cmd.drive(vel_lin)])

        if isinstance(model.state, ApproachDumbbell) and model.image is not None:
            if msg.obstacle_dist < DIST_STOP:
                return (replace(model, state=FaceBlocks()), [cmd.stop])

            if (err_ang := dumbell_alignment_error(model.image)) is None:
                return (model, [cmd.turn(0.2)])

            err_lin = min(msg.obstacle_dist - DIST_STOP, 1.0)
            vel_lin = min(KP_LIN * err_lin, 0.1)
            vel_ang = KP_ANG * err_ang

            return (model, [cmd.velocity(linear=vel_lin, angular=vel_ang)])

        return (model, cmd.none)

    # if isinstance(msg, Image):
    new_model = replace(model, image=msg.image)

    if isinstance(new_model.state, FaceDumbbell):
        if (err_ang := dumbell_alignment_error(msg.image)) is None:
            return (new_model, [cmd.turn(0.2)])

        if abs(err_ang) < 0.05:
            return (replace(new_model, state=ApproachDumbbell()), [cmd.stop])

        vel_ang = KP_ANG * err_ang
        return (new_model, [cmd.turn(vel_ang)])

    return (new_model, cmd.none)


def error_from_center(
    with_image: Callable[[ImageBGR], Optional[Tuple[int, int]]],
    img: ImageBGR,
) -> Optional[float]:
    if (img_pos := with_image(img)) is None:
        return None

    (cx, _) = img_pos
    return 0.5 - (cx / img.width)


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
        sub.laser_scan(partial(obstacle, FOV_LIDAR)),
    ]


def obstacle(fov: int, scan: LaserScan) -> Msg:
    front = scan.ranges[: fov // 2] + scan.ranges[360 - (fov // 2) :]
    # TODO - is the latter check necessary
    ranges_sanitized = [r for r in front if math.isfinite(r) and r > 0.2]
    dist = math.inf if not ranges_sanitized else min(ranges_sanitized)

    return Scan(dist)


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
