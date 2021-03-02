# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Any, Generic, Literal, TypeVar

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageROS

T = TypeVar("T")


@dataclass
class Image(Generic[T]):
    data: Any
    height: int
    width: int


ImageBGR = Image[Literal["BGR"]]
ImageHSV = Image[Literal["HSV"]]


def to_hsv(img: ImageBGR) -> ImageHSV:
    data = cv2.cvtColor(img.data, cv2.COLOR_BGR2HSV)
    return ImageHSV(data, img.height, img.width)


def from_ros_image(bridge: CvBridge, img: ImageROS) -> ImageBGR:
    data = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

    return ImageBGR(
        data=data,
        height=img.height,
        width=img.width,
    )
