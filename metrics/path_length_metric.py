from .trajectory_metric import TMetric
from state import State
from utils import *


class PathLengthMetric(TMetric):
    def __init__(self, more_is_better=False):
          super().__init__(more_is_better)
    
    @staticmethod
    def evaluate_metric(trajectory, **kwargs):
        length = 0
        if (len(trajectory) < 2):
            return length

        init_state = trajectory[0]
        for i in range(1,len(trajectory)):
            goal_state = trajectory[i]
            length = length + distanceBtwStates(init_state, goal_state)
            init_state = goal_state
        return length