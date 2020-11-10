""" Developped by: Flask """
class Vision:
    """ Handles vision """
    i = 12345

    def __init__(self):
        self.data = []

    def f(self):
        return 'hello world'

    def returnMap(self,oldmap=[]):
        """ returns an object containing the map: an array of scipy 
        polygons and the start and end points (2d numpy vectors) """
        return ""

    def returnDynamicCoordinates(self):
        """ returns an object containing the coordinates of the robot (a 3d numpy vector) 
        as well as the end point coordinates (a 2d numpy vector) 
        OR False if the image is not exploitable"""
        return ""

test = Vision()
print(test.f())