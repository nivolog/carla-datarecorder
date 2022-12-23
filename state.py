from math import hypot

class State:
    def __init__(self, x  = 0, y  = 0, theta  = 0, other = None):
        if other is None:
            self.x = x
            self.y = y
            self.theta = theta
        else:
            self.x = other.x
            self.y = other.y
            self.theta = other.theta
                  
    def distance(self, other):
        return hypot(self.x - other.x, self.y - other.y)
    
    def __repr__(self):
        return ' '.join([str(self.x), str(self.y), str(self.theta)])