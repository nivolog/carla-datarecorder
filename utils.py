from math import hypot, atan2, sin, cos
from state import State

def distanceBtwStates(state1, state2):
    return hypot(state1.x - state2.x, state1.y - state2.y)


def distance(x1 , y1 , x2 , y2 ):
    return hypot(x2 - x1, y2 - y1)


def slope(state1 , state2 ):
    return atan2(state2.y - state1.y, state2.x - state1.x)


def normalizeAngle(angle ):
    return atan2(sin(angle), cos(angle))
