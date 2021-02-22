from typing import Callable, TypeVar

T = TypeVar("T")
U = TypeVar("U")
V = TypeVar("V")


def zero_under(low: float, value: float) -> float:
    return 0.0 if value < low else value


def zero_abs_under(low: float, value: float) -> float:
    return 0.0 if abs(value) < low else value


def compose(f: Callable[[U], V], g: Callable[[T], U]) -> Callable[[T], V]:
    return lambda x: f(g(x))
