from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

from q_learning.location import Location
import q_learning.location as Loc
from util import all_unique


@dataclass
class State:
    db_red: Location
    db_green: Location
    db_blue: Location

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, State)
            and other.db_red == self.db_red
            and other.db_green == self.db_green
            and other.db_blue == self.db_blue
        )

    def __hash__(self) -> int:
        return hash((self.db_red, self.db_green, self.db_blue))

    def __iter__(self) -> Iterator[Location]:
        return iter((self.db_red, self.db_green, self.db_blue))

    def __str__(self) -> str:
        return f"{{ r: {self.db_red.name}, g: {self.db_green.name}, b: {self.db_blue.name} }}"


def all_at_blocks(state: State) -> bool:
    return all(map(Loc.at_block, state))


def to_index(state: State) -> int:
    return state.db_red.value + 4 * state.db_green.value + 16 * state.db_blue.value


def from_index(index: int) -> Optional[State]:
    db_blue = index // 16
    db_green = (index - 16 * db_blue) // 4
    db_red = index - 16 * db_blue - 4 * db_green

    return from_locations(
        db_red=Location(db_red),
        db_green=Location(db_green),
        db_blue=Location(db_blue),
    )


def from_locations(
    db_red: Location,
    db_green: Location,
    db_blue: Location,
) -> Optional[State]:
    locs = [db_red, db_green, db_blue]
    at_blocks = [l for l in locs if Loc.at_block(l)]

    return State(db_red, db_green, db_blue) if all_unique(at_blocks) else None


def all_possible_pairs() -> List[Tuple[State, State]]:
    return [(s, t) for s in all_possible() for t in all_possible()]


def all_possible() -> List[State]:
    # TODO mapMaybe instead
    all_states = [
        from_locations(r, g, b) for r in Location for g in Location for b in Location
    ]

    return [state for state in all_states if state is not None]


initial: State = State(
    Location.origin,
    Location.origin,
    Location.origin,
)
