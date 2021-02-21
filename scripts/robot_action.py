#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Any, List, Tuple, Union

from sensor_msgs.msg import LaserScan, Image
import rospy
from rospy_util.controller import Cmd, Controller, Sub

import controller.cmd as cmd
import controller.sub as sub

### Model ###


@dataclass
class Noop:
    pass


Model = Union[Noop]

init_model: Model = Noop()


### Messages ###


@dataclass
class Camera:
    image: Image


@dataclass
class Scanner:
    scan: LaserScan


Msg = Union[Camera, Scanner]


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:

    if isinstance(msg, Camera):
        print("camera")
        return (model, cmd.none)

    print("scan")
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
        model=init_model,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
