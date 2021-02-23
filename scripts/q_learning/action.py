from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Optional

from q_learning.location import Location, at_origin
import q_learning.location as Loc
from q_learning.state import State
from q_learning_project.msg import RobotMoveDBToBlock


# TODO: merge with Location
class Block(IntEnum):
    block_1 = 1
    block_2 = 2
    block_3 = 3


class Dumbbell(IntEnum):
    red = 0
    green = 1
    blue = 2


@dataclass
class Action:
    dumbbell: Dumbbell
    to_block: Block

    def __str__(self) -> str:
        return f"{self.dumbbell.name} -> {self.to_block.name}"


def apply(state: State, action: Action) -> State:
    location = Location(action.to_block)

    return (
        replace(state, db_red=location)
        if action.dumbbell == Dumbbell.red
        else replace(state, db_green=location)
        if action.dumbbell == Dumbbell.green
        else replace(state, db_blue=location)
    )


def from_states(
    state_init: State,
    state_final: State,
) -> Optional[Action]:
    locs = zip(state_init, state_final)
    labeled = zip(Dumbbell, locs)

    transitions = [(db, locs) for (db, locs) in labeled if moved(*locs)]

    if len(transitions) != 1:
        return None

    [(db, (li, lf))] = transitions

    if not at_origin(li):
        return None

    return Action(db, Block(lf))


def moved(loc_init: Location, loc_final: Location) -> bool:
    return loc_init != loc_final


def to_index(action: Action) -> int:
    return (action.dumbbell.value * 3) + (action.to_block.value - 1)


def from_index(index: int) -> Action:
    db = index // 3
    blk = index - (db * 3) + 1
    return Action(Dumbbell(db), Block(blk))


def to_msg(action: Action) -> RobotMoveDBToBlock:
    dumbbell_name = (
        "red"
        if action.dumbbell == Dumbbell.red
        else "green"
        if action.dumbbell == Dumbbell.green
        else "blue"
    )

    return RobotMoveDBToBlock(
        robot_db=dumbbell_name,
        block_id=action.to_block.value,
    )
