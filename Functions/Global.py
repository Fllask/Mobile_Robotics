""" Developped by: Titouan """
class Global:
    """ Handles global path planning """"
    i = 12345

    def __init__(self):
        self.data = []

    def f(self):
        return 'hello world'

    def returnPath(self,map,startPoint,endPoint):
        """ takes map: an array of scipy polygons, startPoint 
        and endPoint two scipy vectors (2d) returns a 
        list of 2d scipy vectors representing the points along the path"""
        return ""

test = Global()
print(test.f())