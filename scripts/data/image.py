# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Any

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

ImageROS = Image


@dataclass
class ImageHSV:
    data: Any


def from_ros_image(bridge: CvBridge, img_msg: ImageROS):
    bgr8 = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    hsv = cv2.cvtColor(bgr8, cv2.COLOR_BGR2HSV)

    return ImageHSV(hsv)
