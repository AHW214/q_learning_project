"""
Perception of colors.
"""

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Dict, Generic, NewType, Optional, Tuple, TypeVar

import cv2
from numpy import array
import rospy_util.mathf as mathf

from perception.image import ImageBGR
import perception.image as image

__all__ = (
    "HSV_CV2",
    "Range",
    "locate_color",
    "hsv_cv2",
    "hsv_range",
)

T = TypeVar("T")

# HSV in CV2 specification.

HSV_CV2 = NewType("HSV", Tuple[float, float, float])


@dataclass
class Range(Generic[T]):
    """
    A range [lower, upper] of some type.
    """

    lower: T
    upper: T


def locate_color(range: Range[HSV_CV2], img: ImageBGR) -> Optional[Tuple[int, int]]:
    """
    Locate the center point of pixels in the given color range.
    """

    hsv = image.to_hsv(img)

    masked = cv2.inRange(
        src=hsv.data,
        lowerb=array(range.lower),
        upperb=array(range.upper),
    )

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
    upper: Tuple[float, float, float],
) -> Range[HSV_CV2]:
    """
    Create a range of CV2 HSV values.
    """

    return Range(
        lower=hsv_cv2(*lower),
        upper=hsv_cv2(*upper),
    )


def hsv_cv2(hue: float, sat: float, val: float) -> HSV_CV2:
    """
    Create an HSV value in CV2 specification from standard HSV components.
    """

    return HSV_CV2(
        (
            mathf.lerp(0, 179, hue / 360),
            mathf.lerp(0, 255, sat / 100),
            mathf.lerp(0, 255, val / 100),
        )
    )
