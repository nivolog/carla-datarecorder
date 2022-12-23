from metrics.trajectory_metric import TMetric
from metrics.path_length_metric import PathLengthMetric 
from state import State
from utils import *


class AOLMetric(TMetric):
    """_summary_
    Computes total angle / path length metric.
     *
     * Total angle is \int |\dot{yaw}(t)|dt, where yaw is the heading angle
     * pointing forwards, i.e., it changes by pi for a cusp.
     * Total path length is \int 1 dt.
     *
     * This metric can be interpreted as a combination of curvature and cusp.
     *
     * @param trajectory The trajectory to evaluate.
     * @param visualize
     * @return AOL.
    """

    def __init__(self, more_is_better=False):
        super().__init__(more_is_better)
        self.max_curvature = 1e10
    
    @staticmethod
    def evaluate_metric(path, **kwargs):
        path_length = PathLengthMetric.evaluate_metric(path)
        total_yaw_change = 0
        
        prev = current = next = 0
        while next != len(path):
            if path[prev].distance(path[current]) <= 0:
                current += 1
                next += 1
            elif path[current].distance(path[next]) <= 0:
                next += 1
            else:
                yaw_prev = slope(path[prev], path[current])
                yaw_next = slope(path[current], path[next])
                # compute angle difference in [0, pi)
                # close to pi -> cusp; 0 -> straight line; inbetween -> curve
                yaw_change = abs(normalizeAngle(yaw_next - yaw_prev))
                # both in [-pi, pi]

                total_yaw_change += yaw_change;

                prev = current
                current = next
                next += 1
        return total_yaw_change / path_length;
