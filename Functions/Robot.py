""" Developped by: Thomas """
class Robot:
    """ Handles the control of the robot """
    i = 12345
    #test
    def __init__(self):
        self.data = []

    def f(self):
        return 'hello world'

    def stateEstimator(self):
        """ returns: coordonées en x,y,theta globales (un vecteur 3d numpy) """
        return ""

test = Robot()
print(test.f())