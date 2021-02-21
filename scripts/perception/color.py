# pyright: reportMissingTypeStubs=false

from typing import Dict, NewType, Optional, Tuple

import cv2
from numpy import array, uint8

from data.image import ImageHSV

HSV = NewType("HSV", Tuple[uint8, uint8, uint8])


def center_of_color(
    lower_bound: Tuple[int, int, int],
    upper_bound: Tuple[int, int, int],
    image: ImageHSV,
) -> Optional[Tuple[int, int]]:
    masked = cv2.inRange(
        src=image.data,
        lowerb=array(hsv_from_rgb(*lower_bound)),
        upperb=array(hsv_from_rgb(*upper_bound)),
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


def hsv_from_rgb(r: int, g: int, b: int) -> HSV:
    rgb = array([[[r, g, b]]], dtype=uint8)
    [[hsv]] = cv2.cvtColor(src=rgb, code=cv2.COLOR_RGB2HSV)
    return HSV(hsv)
