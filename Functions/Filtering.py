""" Developped by : Pierre """

class Filtering:
    """ Handles the aggregation of data """
    i = 12345

    def __init__(self):
        self.data = []

    def f(self):
        return 'hello world'

    def filter(self,camera_position,state_estimator,target = False):
        """ input:      the position as computed by the vision algorithm (a 3d numpy vector)
                        the position estimated by the state estimator (a 3d numpy vector), 

            returns:    an estimation of position a 3d numpy vector """

test = Filtering()
print(test.f())