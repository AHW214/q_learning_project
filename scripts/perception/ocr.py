# pyright: reportMissingTypeStubs=false

from typing import List, Optional, Tuple

import cv2
from keras_ocr.pipeline import Pipeline
from numpy import ndarray
import numpy as np

from data.image import ImageBGR

Prediction = Tuple[str, ndarray]

# box = ndarray[ndarray[float32]]


def locate_number(
    pipeline: Pipeline,
    num: int,
    img: ImageBGR,
) -> Optional[Tuple[int, int]]:
    box = locate_front_facing_text(pipeline, str(num), img)

    if box is None:
        return None

    ((x1, y1), _, (x2, y2), _) = box
    rec = cv2.rectangle(
        img=img.data,
        pt1=(round(x1), round(y1)),
        pt2=(round(x2), round(y2)),
        color=(255, 0, 0),
        thickness=2,
    )
    # cv2.imshow("window", rec)
    # cv2.waitKey(3)

    (cx, cy) = center_of_box(box)

    return (round(cx), round(cy))


def locate_front_facing_text(
    pipeline: Pipeline,
    text: str,
    img: ImageBGR,
) -> Optional[ndarray]:
    boxes = locate_all_text(pipeline, text, img)
    return None if not boxes else max(boxes, key=area_of_box)


def locate_all_text(pipeline: Pipeline, text: str, img: ImageBGR) -> List[ndarray]:
    predictions = recognize(pipeline, img)

    return [box for (txt, box) in predictions if txt == text]


def recognize(pipeline: Pipeline, img: ImageBGR) -> List[Prediction]:
    groups: List[List[Prediction]] = pipeline.recognize([img.data])
    return [] if not groups else groups[0]


def area_of_box(box: ndarray) -> float:
    (w, h) = dimensions_of_box(box)
    return w * h


def dimensions_of_box(box: ndarray) -> Tuple[float, float]:
    (top_left, _, bottom_right, _) = box

    (x1, y1) = top_left
    (x2, y2) = bottom_right

    return (x2 - x1, y2 - y1)


def center_of_box(box: ndarray) -> Tuple[float, float]:
    return np.average(box, axis=0)
