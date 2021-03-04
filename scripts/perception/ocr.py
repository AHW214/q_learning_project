"""
Perception of textual characters.
"""

# pyright: reportMissingTypeStubs=false

from typing import List, Optional, Tuple

from keras_ocr.pipeline import Pipeline
from numpy import ndarray
import numpy as np

from perception.image import ImageBGR

__all__ = (
    "locate_all_text",
    "locate_front_facing_text",
    "locate_number",
    "recognize",
)

# An OCR prediction

Prediction = Tuple[str, ndarray]


def locate_number(
    pipeline: Pipeline,
    num: int,
    img: ImageBGR,
) -> Optional[Tuple[int, int]]:
    """
    Try to locate the most front-facing instance of the specified number in the
    given image.
    """

    box = locate_front_facing_text(pipeline, str(num), img)

    if box is None:
        return None

    (cx, cy) = center_of_box(box)

    return (round(cx), round(cy))


def locate_front_facing_text(
    pipeline: Pipeline,
    text: str,
    img: ImageBGR,
) -> Optional[ndarray]:
    """
    Try to locate the most front-facing instance of the specified text in the
    given image.
    """

    boxes = locate_all_text(pipeline, text, img)
    return None if not boxes else max(boxes, key=width_of_box)


def locate_all_text(pipeline: Pipeline, text: str, img: ImageBGR) -> List[ndarray]:
    """
    Locate all instances of the specified text in the given image.
    """

    predictions = recognize(pipeline, img)

    return [box for (txt, box) in predictions if txt == text]


def recognize(pipeline: Pipeline, img: ImageBGR) -> List[Prediction]:
    """
    Recognize characters in the given image.
    """

    groups: List[List[Prediction]] = pipeline.recognize([img.data])
    return [] if not groups else groups[0]


def width_of_box(box: ndarray) -> float:
    """
    Compute the width of a bounding box.
    """

    (w, _) = dimensions_of_box(box)
    return w


def dimensions_of_box(box: ndarray) -> Tuple[float, float]:
    """
    Compute the dimensions of a bounding box.
    """

    (top_left, _, bottom_right, _) = box

    (x1, y1) = top_left
    (x2, y2) = bottom_right

    return (x2 - x1, y2 - y1)


def center_of_box(box: ndarray) -> Tuple[float, float]:
    """
    Compute the center coordinates of a bounding box.
    """

    return np.average(box, axis=0)
