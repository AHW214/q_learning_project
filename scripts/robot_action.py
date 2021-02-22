#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import Enum
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

### Model ###

Locator = Callable[[ImageHSV], Optional[Tuple[int, int]]]


class State(Enum):
    locate = 1
    face = 2
    approach = 3


@dataclass
class Model:
    cvBridge: CvBridge
    locator: Locator
    state: State


init_model: Model = Model(
    cvBridge=CvBridge(),
    locator=color.locate_green,
    state=State.locate,
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, [cmd.turn(0.2)])


### Messages ###


@dataclass
class Camera:
    image: ImageROS


@dataclass
class Scanner:
    scan: LaserScan


Msg = Union[Camera, Scanner]


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(msg, Scanner):
        return (model, cmd.none)

    imageHSV = image.from_ros_image(model.cvBridge, msg.image)
    location = model.locator(imageHSV)

    if model.state == State.locate:
        if location is None:
            return (model, [cmd.turn(0.2)])

        return (replace(model, state=State.face), cmd.none)

    if model.state == State.face:
        if location is None:
            return (replace(model, state=State.locate), [cmd.stop])

        (cx, _) = location

        err = (imageHSV.width / 2) - cx

        if err < 10:
            return (replace(model, state=State.approach), [cmd.stop])

        k_p = 1.0 / 500.0
        return (model, [cmd.turn(k_p * err)])

    if model.state == State.approach:
        if location is None:
            return (replace(model, state=State.locate), [cmd.stop])

        (cx, _) = location
        err = (imageHSV.width / 2) - cx
        k_p = 1.0 / 100.0

        return (model, [cmd.velocity(linear=0.5, angular=k_p * err)])

    return (model, cmd.none)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, Msg]]:
    return [
        sub.image_sensor(Camera),
        sub.laser_scan(Scanner),
    ]


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
