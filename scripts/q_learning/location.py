from enum import IntEnum


class Location(IntEnum):
    origin = 0
    block_1 = 1
    block_2 = 2
    block_3 = 3


def at_block(loc: Location) -> bool:
    return loc != Location.origin


def at_origin(loc: Location) -> bool:
    return loc == Location.origin
