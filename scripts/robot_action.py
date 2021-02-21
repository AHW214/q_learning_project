#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Any, List, Tuple, Union

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import rospy
from rospy_util.controller import Cmd, Controller, Sub

import controller.cmd as cmd
import controller.sub as sub
from data.image import ImageROS
import data.image as image
from perception.color import center_of_color

### Model ###


@dataclass
class Model:
    cvBridge: CvBridge


init_model: Model = Model(
    cvBridge=CvBridge(),
)


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

    if isinstance(msg, Camera):
        imageHSV = image.from_ros_image(model.cvBridge, msg.image)
        center = center_of_color(
            lower_bound=(74, 128, 64),
            upper_bound=(0, 255, 42),
            image=imageHSV,
        )

        print(center)

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
