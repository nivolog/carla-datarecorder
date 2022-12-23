from metrics.trajectory_metric import TMetric
from state import State
from distance_matrix import DistanceMatrix 

class ClearingMetric(TMetric):
    """_summary_

        Returns:
            _type_: _description_
            /**
            * Computes mean clearing distance of a path in given map.
            */
    """
    
    def __init__(self, more_is_better=False):
       super().__init__(more_is_better)
        
    
    @staticmethod
    def evaluate_metric(path, **kwargs):
        if 'dm' in kwargs:
            dm = kwargs['dm']
        elif 'map' in kwargs:
            map = kwargs['map']
            dm = DistanceMatrix(map, map.height, map.width)
        else:
            print('No distance matrix or obstacle map passed to clearing metric!')
            raise ValueError
        distMat = dm.getDistMatrix()
        ans = 0
        for point in path:
            ans += distMat[int(point.x)][int(point.y)];
        return ans/len(path) if len(path) > 0 else -1