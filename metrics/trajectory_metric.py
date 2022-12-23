from state import State

class TMetric:
    def __init__(self, more_is_better = False):
        self.more_is_better = more_is_better
    
    @staticmethod
    def evaluate_metric(path, **kwargs):
        raise NotImplementedError
    
    def evaluate(self, trajectory):
        return self.evaluate_metric(trajectory)