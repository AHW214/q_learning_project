from typing import NewType, Optional, Tuple

from numpy import ndarray
import numpy as np
import rospy
from std_msgs.msg import Header

from q_learning.action import Action
import q_learning.action as act
from q_learning.state import State
import q_learning.state as st
import q_learning_project.msg as msg

QMatrix = NewType("QMatrix", ndarray)


def update(
    state: State,
    action: Action,
    reward: int,
    learning_rate: float,
    discount_factor: float,
    q_matrix: QMatrix,
) -> Optional[Tuple[QMatrix, State, float, float]]:
    def outside(ix: int, bound: int) -> bool:
        return ix < 0 or ix >= bound

    next_state = act.apply(state, action)

    print("----")
    print(state)
    print(action)
    print(next_state)

    (rows, cols) = q_matrix.shape

    st_ix = st.to_index(state)
    stt_ix = st.to_index(next_state)
    act_ix = act.to_index(action)

    if outside(st_ix, rows) or outside(stt_ix, rows) or outside(act_ix, cols):
        return None

    mx = max(q_matrix[stt_ix], default=0)

    reward_old = q_matrix[st_ix][act_ix]
    reward_new = reward_old + learning_rate * (
        reward + discount_factor * mx - reward_old
    )

    q_matrix[st_ix][act_ix] = reward_new  # TODO: rip
    return (q_matrix, next_state, reward_old, reward_new)


def to_msg(q_matrix: QMatrix) -> msg.QMatrix:
    header = Header(stamp=rospy.Time.now())
    rows = [msg.QMatrixRow([round(r) for r in row]) for row in q_matrix]
    return msg.QMatrix(header=header, q_matrix=rows)


def init() -> QMatrix:
    return QMatrix(np.zeros(shape=(64, 9), dtype=float))
