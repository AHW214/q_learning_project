# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Dict, Generic, NewType, Optional, Tuple, TypeVar

import cv2
from numpy import array
import rospy_util.mathf as mathf

from data.image import ImageBGR
import data.image as image

T = TypeVar("T")

HSV_CV2 = NewType("HSV", Tuple[float, float, float])


@dataclass
class Range(Generic[T]):
    lower: T
    upper: T


def locate_color(range: Range[HSV_CV2], img: ImageBGR) -> Optional[Tuple[int, int]]:
    hsv = image.to_hsv(img)

    print(range)

    masked = cv2.inRange(
        src=hsv.data,
        lowerb=array(range.lower),
        upperb=array(range.upper),
    )

    cv2.imshow("window", masked)
    cv2.waitKey(3)

    moments: Dict[str, int] = cv2.moments(masked)

    if (
        (m00 := moments.get("m00")) is not None
        and m00 > 0.0
        and (m10 := moments.get("m10")) is not None
        and (m01 := moments.get("m01")) is not None
    ):
        cx = round(m10 / m00)
        cy = round(m01 / m00)
        return (cx, cy)

    return None


def hsv_range(
    lower: Tuple[float, float, float],
    upper: Tuple[float, float, float]
) -> Range[HSV_CV2]:
    return Range(
        lower=hsv_cv2(*lower),
        upper=hsv_cv2(*upper),
    )


def hsv_cv2(hue: float, sat: float, val: float) -> HSV_CV2:
    return HSV_CV2(
        (
            mathf.lerp(0, 179, hue / 360),
            mathf.lerp(0, 255, sat / 100),
            mathf.lerp(0, 255, val / 100),
        )
    )
