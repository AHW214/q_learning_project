#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

from cv_bridge import CvBridge
import rospy
from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2
from sensor_msgs.msg import LaserScan

from controller import Cmd, Controller, Sub, cmd, sub
from perception import HSV_CV2, ImageBGR, ImageROS, Range, color, image, ocr
from q_learning_project.msg import Actions, ArmResult, RobotMoveDBToBlock
from util import approx_zero, compose, head, set_under_abs

### Model ###


# Robot Actions


class Block(IntEnum):
    """
    Block identifiers.
    """

    one = 1
    two = 2
    three = 3


@dataclass
class RobotAction:
    """
    An action specifying a dumbbell to be brought to a block.

    @attribute `block`: Identifier of the block.

    @attribute `dumbbell`: HSV color range identifying the dumbbell.
    """

    block: Block
    dumbbell: Range[HSV_CV2]


# Dumbell retrieval states


@dataclass
class FaceDumbbells:
    """
    Face the general location of the dumbbells.
    """

    pass


@dataclass
class FaceDumbbell:
    """
    Face the dumbbell specified by the current action.
    """

    pass


@dataclass
class ApproachDumbbell:
    """
    Approach the dumbbell specified by the current action.
    """

    pass


@dataclass
class LiftDumbbell:
    """
    Lift the dumbbell specified by the current action.
    """

    pass


# Dumbbell placement states


@dataclass
class FaceBlocks:
    """
    Face the general location of the blocks.

    @attribute `ocr_targets`: Positions to face when identifying blocks by OCR.
    """

    ocr_targets: List[Vector2]


@dataclass
class LocateBlock:
    """
    Locate the block specified by the current action.

    @attribute `ocr_targets`: Positions to face when identifying block by OCR.

    @attribute `alignments`: Number of times to re-align the robot when driving
    towards the block.
    """

    ocr_targets: List[Vector2]
    alignments: int


@dataclass
class FaceBlock:
    """
    Face the block (previously located) for the current action.

    @attribute `direction`: Absolute direction of the block.

    @attribute `alignments`: Number of times to re-align the robot when driving
    towards the block.
    """

    direction: float
    alignments: int


@dataclass
class ApproachBlock:
    """
    Approach the block specified by the current action.

    @attribute `alignments`: Number of times to re-align the robot when driving
    towards the block.
    """

    alignments: int


@dataclass
class PlaceDumbbell:
    """
    Place the dumbbell down.
    """

    pass


# Robot states

State = Union[
    FaceDumbbells,
    FaceDumbbell,
    ApproachDumbbell,
    LiftDumbbell,
    FaceBlocks,
    LocateBlock,
    FaceBlock,
    ApproachBlock,
    PlaceDumbbell,
]


@dataclass
class Model:
    """
    Model of the robot

    @attribute `state`: Current robot state.

    @attribute `actions`: Queue of actions to execute.

    @attribute `image`: Last image taken by the camera.

    @attribute `cv_bridge`: CV2 bridge for image format conversion.

    @attribute: `keras_pipeline`: Keras pipeline for OCR.
    """

    state: State
    actions: List[RobotAction]
    image: Optional[ImageBGR]
    cv_bridge: CvBridge
    keras_pipeline: ocr.Pipeline


init_model: Model = Model(
    state=FaceDumbbell(),
    actions=[],
    image=None,
    cv_bridge=CvBridge(),
    keras_pipeline=ocr.Pipeline(scale=1.0),
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Actions:
    """
    Optimal actions sent by the Q-algorithm node.
    """

    actions: List[RobotMoveDBToBlock]


@dataclass
class Arm:
    """
    Result of commands executed by the arm controller node.
    """

    status: ArmResult


@dataclass
class Image:
    """
    Image taken by the robot camera.
    """

    image: ImageBGR


@dataclass
class Odom:
    """
    Robot pose sent by the odometry sensor.
    """

    pose: TurtlePose


@dataclass
class Scan:
    """
    Distance to the closest obstacle measured by robot LiDAR.
    """

    obstacle_dist: float


# Inbound messsages

Msg = Union[
    Actions,
    Arm,
    Image,
    Odom,
    Scan,
]


### Update ###

# Angular movement

KP_ANG_SPIN = math.pi / 1.5
KP_ANG_FACE = math.pi / 8.0

VEL_ANG_SPIN_MIN = math.pi / 10
VEL_ANG_FACE_MIN = math.pi / 30

# Linear movement

KP_LIN_TO_DB = 0.5
KP_LIN_TO_BLOCK = 0.4

VEL_LIN_TO_DB_MAX = 0.3
VEL_LIN_TO_BLOCK_MAX = 0.5

# Stopping and alignment

DIST_STOP_DB = 0.4
DIST_STOP_BLOCK = 0.4
DIST_BLOCK_ALIGNMENT = 1.5

# General positions of dumbbells and blocks

POS_DBS = Vector2(x=1.0, y=0.0)
POS_BLOCK_RIGHT = Vector2(x=-2.0, y=1.0)
POS_BLOCK_LEFT = Vector2(x=-2.0, y=-1.0)

# Camera field of view as specified by
# https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/

FOV_IMAGE = math.radians(62.2)

# LiDAR field of view

FOV_LIDAR = 60

# Dumbbell color ranges

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
    """
    Given an inbound message and the current robot model, produce a new model
    and a list of commands to execute.
    """

    if isinstance(msg, Actions):
        # Received actions from the Q-algorithm node.

        parsed = [parse_action(act) for act in msg.actions]
        new_actions = [act for act in parsed if act is not None]

        if not new_actions:
            print("WARN: None of the received optimal actions could be parsed")
            return (model, cmd.none)

        # Add received actions to the action queue.

        actions = model.actions + new_actions
        return (replace(model, actions=actions), cmd.none)

    if (action := head(model.actions)) is None:
        # No actions currently in the queue; do nothing.

        return (model, cmd.none)

    # Function to compute dumbbell alignment error.

    dumbell_alignment_error: Callable[[ImageBGR], Optional[float]] = partial(
        error_from_center,
        partial(
            color.locate_color,
            action.dumbbell,
        ),
    )

    # Function to compute block alignment error.

    block_alignment_error: Callable[[ImageBGR], Optional[float]] = partial(
        error_from_center,
        partial(
            ocr.locate_number,
            model.keras_pipeline,
            action.block.value,
        ),
    )

    if isinstance(msg, Arm):
        # Received a command result from the arm controller node.

        if msg.status.error:
            print(f"WARN: Arm controller sent an error '{msg.status.error}'")
            return (model, cmd.none)

        if isinstance(model.state, LiftDumbbell):
            # Done lifting the dumbbell; begin facing blocks.

            ocr_targets = [POS_BLOCK_LEFT, POS_BLOCK_RIGHT]

            return (replace(model, state=FaceBlocks(ocr_targets)), cmd.none)

        if isinstance(model.state, PlaceDumbbell):
            # Done placing dumbbell at block; begin facing dumbbells.

            new_model = replace(
                model,
                actions=model.actions[1:],
                state=FaceDumbbells(),
            )

            return (new_model, cmd.none)

        return (model, cmd.none)

    if isinstance(msg, Odom):
        # Received an odometry message.

        if isinstance(model.state, FaceDumbbells):
            # Compute direction off from the dumbbells.

            dir_dbs = POS_DBS - msg.pose.position
            dir_off = v2.signed_angle_between(v2.from_angle(msg.pose.yaw), dir_dbs)

            if approx_zero(err_ang := dir_off / math.pi):
                # Approximately aligned with dumbbells; begin facing dumbbell
                # specified by the current action.

                return (replace(model, state=FaceDumbbell()), [cmd.stop])

            # Rotate to face the dumbbells.

            vel_ang = set_under_abs(KP_ANG_SPIN * err_ang, VEL_ANG_SPIN_MIN)

            return (model, [cmd.turn(vel_ang)])

        if isinstance(model.state, FaceBlocks):
            # No targets given to face; assume already facing blocks and begin
            # locating block specified by the current action.

            if not model.state.ocr_targets:
                new_state = LocateBlock(ocr_targets=[], alignments=0)

                return (replace(model, state=new_state), [cmd.stop])

            # Compute direction off from current block OCR target position.

            (target, *rest_targets) = model.state.ocr_targets

            dir_block = target - msg.pose.position
            dir_off = v2.signed_angle_between(v2.from_angle(msg.pose.yaw), dir_block)

            if approx_zero(err_ang := dir_off / math.pi):
                # Approximately aligned with block OCR target position; begin
                # locating the block specified by the current action.
                new_state = LocateBlock(ocr_targets=rest_targets, alignments=1)

                return (replace(model, state=new_state), [cmd.stop])

            # Rotate to face the block OCR target position.

            vel_ang = set_under_abs(KP_ANG_SPIN * err_ang, VEL_ANG_SPIN_MIN)

            return (model, [cmd.turn(vel_ang)])

        if isinstance(model.state, LocateBlock) and model.image is not None:
            # Locate block for the current action given image data.

            if (err_ang := block_alignment_error(model.image)) is None:
                # Block not in view of the image; align with target positions
                # at which to perform OCR.

                new_state = FaceBlocks(model.state.ocr_targets)

                return (replace(model, state=new_state), cmd.none)

            # Compute direction to the block specified by the current action.

            yaw_off = FOV_IMAGE * err_ang
            dir_block = msg.pose.yaw + yaw_off

            # Face block at the computed direction; specify number of alignments
            # to perform when later approaching the block.

            new_state = FaceBlock(
                direction=dir_block,
                alignments=model.state.alignments,
            )

            return (replace(model, state=new_state), cmd.none)

        if isinstance(model.state, FaceBlock):
            # Compute angular error to face the block specified by the current action.

            err_ang = model.state.direction - msg.pose.yaw

            if approx_zero(err_ang):
                # Approximately aligned; begin approaching the block.

                new_state = ApproachBlock(model.state.alignments)

                return (replace(model, state=new_state), [cmd.stop])

            # Compute angular velocity with which to turn towards the block.

            vel_ang = set_under_abs(0.5 * err_ang, VEL_ANG_FACE_MIN)

            return (model, [cmd.turn(vel_ang)])

        return (model, cmd.none)

    if isinstance(msg, Scan):
        # Received a message from robot LiDAR.

        if isinstance(model.state, ApproachBlock):
            # Compute distance to next block alignment.

            alignment_dist = DIST_BLOCK_ALIGNMENT * model.state.alignments

            if msg.obstacle_dist < alignment_dist:
                # Locate the block to perform re-alignment.

                new_state = LocateBlock(
                    # Assume we're already looking at the target; none specified.
                    ocr_targets=[],
                    # Decrement number of remaning block alignments.
                    alignments=model.state.alignments - 1,
                )

                return (replace(model, state=new_state), [cmd.stop])

            if msg.obstacle_dist < DIST_STOP_BLOCK:
                # Within stopping distance of the block; begin placing the dumbbell.

                return (
                    replace(model, state=PlaceDumbbell()),
                    [cmd.stop, cmd.place_dumbbell],
                )

            # Compute linear velocity with which to approach the block.

            err_lin = msg.obstacle_dist - DIST_STOP_BLOCK
            vel_lin = min(KP_LIN_TO_BLOCK * err_lin, VEL_LIN_TO_BLOCK_MAX)

            return (model, [cmd.drive(vel_lin)])

        if isinstance(model.state, ApproachDumbbell) and model.image is not None:
            # Approach the dumbbell specified by the current action.

            if msg.obstacle_dist < DIST_STOP_DB:
                # Within stopping distance of the dumbbell; begin lifting the dumbbell.

                return (
                    replace(model, state=LiftDumbbell()),
                    [cmd.stop, cmd.lift_dumbbell],
                )

            if (err_ang := dumbell_alignment_error(model.image)) is None:
                # Dumbell not in view of the image; rotate until dumbbells are
                # in view.

                return (model, [cmd.turn(0.2)])

            # Compute velocities with which to approach the dumbell.

            err_lin = msg.obstacle_dist - DIST_STOP_DB
            vel_lin = min(KP_LIN_TO_DB * err_lin, VEL_LIN_TO_DB_MAX)
            vel_ang = KP_ANG_FACE * err_ang

            return (model, [cmd.velocity(linear=vel_lin, angular=vel_ang)])

        return (model, cmd.none)

    # Received a message from the camera; update robot model with new image.

    new_model = replace(model, image=msg.image)

    if isinstance(new_model.state, FaceDumbbell):
        # Face the dumbell specified by the current action.

        if (err_ang := dumbell_alignment_error(msg.image)) is None:
            # Dumbell not in view of the image; rotate until dumbbell is in view.

            return (new_model, [cmd.turn(0.2)])

        if approx_zero(err_ang):
            # Approximately aligned with the dumbbell; begin approaching dumbbell.

            return (replace(new_model, state=ApproachDumbbell()), [cmd.stop])

        # Compute angular velocity with which to face the dumbbell.

        vel_ang = KP_ANG_FACE * err_ang

        return (new_model, [cmd.turn(vel_ang)])

    return (new_model, cmd.none)


def error_from_center(
    with_image: Callable[[ImageBGR], Optional[Tuple[int, int]]],
    img: ImageBGR,
) -> Optional[float]:
    """
    Return the horizontal error of the computed pixel coordinates from the center
    of the given image.

    @param `with_image`: Function to compute pixel coordinates from the given image.

    @param `img`: The image to process.
    """
    if (img_pos := with_image(img)) is None:
        return None

    (cx, _) = img_pos
    return 0.5 - (cx / img.width)


def parse_action(action: RobotMoveDBToBlock) -> Optional[RobotAction]:
    """
    Parse a robot action from a RobotMoveDBToBlock message.
    """
    if (block := parse_block(action.block_id)) is None or (
        dumbbell := parse_dumbbell(action.robot_db)
    ) is None:
        return None

    return RobotAction(block, dumbbell)


def parse_block(block_id: int) -> Optional[Block]:
    """
    Parse a block identifier from an int.
    """
    return Block(block_id) if block_id in [b.value for b in Block] else None


def parse_dumbbell(clr: str) -> Optional[Range[HSV_CV2]]:
    """
    Parse a dumbbell color range from the name of a color.
    """
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
    """
    Subscriptions from which to receive messages.
    """
    return [
        sub.optimal_actions(Actions),
        sub.arm_result(Arm),
        sub.image_sensor(toImageCV2(model.cv_bridge)),
        sub.odometry(Odom),
        sub.laser_scan(partial(obstacle, FOV_LIDAR)),
    ]


def obstacle(fov: int, scan: LaserScan) -> Msg:
    """
    Find the distance to the nearest obstacle in a specific FOV of the laser scan.
    """
    front = scan.ranges[: fov // 2] + scan.ranges[360 - (fov // 2) :]
    # TODO - is the latter check necessary
    ranges_sanitized = [r for r in front if math.isfinite(r) and r > 0.2]
    dist = math.inf if not ranges_sanitized else min(ranges_sanitized)

    return Scan(dist)


def toImageCV2(cvBridge: CvBridge) -> Callable[[ImageROS], Msg]:
    """
    Convert a ROS image message to a CV2 image in BGR format.
    """
    return compose(Image, partial(image.from_ros_image, cvBridge))


### Run ###


def run() -> None:
    rospy.init_node("q_learning_robot_action")

    # Run the ROS controller

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
