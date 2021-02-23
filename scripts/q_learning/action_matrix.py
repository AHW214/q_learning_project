from functools import reduce
import random
from typing import Dict, List, NewType, Tuple, Optional


from q_learning.action import Action
import q_learning.action as act
from q_learning.state import State

ActionMatrix = NewType("ActionMatrix", Dict[State, Dict[State, Action]])


def random_action(state: State, matrix: ActionMatrix) -> Optional[Action]:
    actions = [*possible_actions(state, matrix).values()]
    return random.choice(actions) if actions else None


def possible_actions(state: State, matrix: ActionMatrix) -> Dict[State, Action]:
    return matrix.get(state, {})


def create(states: List[Tuple[State, State]]) -> ActionMatrix:
    def process_states(
        d: Dict[State, Dict[State, Action]],
        ss: Tuple[State, State],
    ):
        (state_init, state_final) = ss

        if (action := act.from_states(state_init, state_final)) is None:
            return d

        actions = d.get(state_init, {})
        return {**d, state_init: {**actions, state_final: action}}

    return ActionMatrix(
        reduce(
            process_states,
            states,
            {},
        )
    )
