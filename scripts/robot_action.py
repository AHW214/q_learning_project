#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import Enum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import rospy
from rospy_util.controller import Cmd, Controller, Sub

import controller.cmd as cmd
import controller.sub as sub
from data.image import ImageHSV, ImageROS
import data.image as image
import perception.color as color
from util import compose

### Model ###

Locator = Callable[[ImageHSV], Optional[Tuple[int, int]]]


class Movement(Enum):
    stop = 1
    locate = 2
    face = 3
    approach = 4


@dataclass
class AwaitImage:
    pass


@dataclass
class HaveImage:
    image: ImageHSV
    movement: Movement


State = Union[AwaitImage, HaveImage]


@dataclass
class Model:
    cvBridge: CvBridge
    locator: Locator
    state: State


init_model: Model = Model(
    cvBridge=CvBridge(),
    locator=color.locate_green,
    state=AwaitImage(),
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, [cmd.turn(0.2)])


### Messages ###


@dataclass
class Image:
    image: ImageHSV


@dataclass
class Obstacle:
    distance: float


@dataclass
class Void:
    pass


Msg = Union[Image, Obstacle, Void]


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(msg, Image):
        if isinstance(model.state, AwaitImage):
            new_state = HaveImage(msg.image, Movement.locate)
            return (replace(model, state=new_state), cmd.none)

        new_state = replace(model.state, image=msg.image)
        return (replace(model, state=new_state), cmd.none)

    if (
        isinstance(msg, Void)
        or isinstance(model.state, AwaitImage)
        or model.state.movement == Movement.stop
    ):
        return (model, cmd.none)

    location = model.locator(model.state.image)
    movement = model.state.movement

    if movement == Movement.locate:
        if location is None:
            return (model, [cmd.turn(0.2)])

        return (new_movement(model, Movement.face), cmd.none)

    if movement == Movement.face:
        if location is None:
            return (new_movement(model, Movement.locate), [cmd.stop])

        (cx, _) = location

        err_ang = (model.state.image.width / 2) - cx

        if err_ang < 10:
            return (new_movement(model, Movement.approach), [cmd.stop])

        kp_ang = 1.0 / 500.0
        vel_ang = kp_ang * err_ang

        return (model, [cmd.turn(vel_ang)])

    if movement == Movement.approach:
        if location is None:
            return (new_movement(model, Movement.locate), [cmd.stop])

        (cx, _) = location
        err_ang = (model.state.image.width / 2) - cx
        kp_ang = 1.0 / 500.0

        err_lin = msg.distance - 0.3

        if err_lin <= 0.0:
            print("stop")
            return (new_movement(model, Movement.stop), [cmd.stop])

        kp_lin = 0.5

        vel_ang = kp_ang * err_ang
        vel_lin = kp_lin * err_lin

        return (model, [cmd.velocity(linear=vel_lin, angular=vel_ang)])

    return (model, cmd.none)


def new_movement(model: Model, movement: Movement) -> Model:
    new_state = replace(model.state, movement=movement)
    return replace(model, state=new_state)


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    return [
        sub.image_sensor(toImageHSV(model.cvBridge)),
        sub.laser_scan(obstacle),
    ]


def obstacle(scan: LaserScan) -> Msg:
    ranges: List[float] = [r for r in scan.ranges if math.isfinite(r) and r > 0.2]

    if not ranges:
        return Void()

    closest = min(ranges)

    print(closest)

    return Obstacle(distance=closest)


def toImageHSV(cvBridge: CvBridge) -> Callable[[ImageROS], Image]:
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
