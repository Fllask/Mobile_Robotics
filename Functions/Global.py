""" Developped by: Titouan """

import matplotlib.pyplot as plt
import functools
import itertools
import operator

""" just returns a test map so that test runs of the planning algorithm may be computed """
def TestMap():
    polyA = [[40,30],
             [40,50],
             [45,40],
             [45,35]]

    polyB = [[70,35],
             [60,46],
             [50,63],
             [70,55]]

    polyC = [[35,60],
             [35,70],
             [42,76],
             [45,60]]

    return [polyA,polyB,polyC]

""" placeholder map plotting function """
def mapPlot(map,start,finish):
    #plotting parameters
    fsz = 8 # size of the plotted figure
    polyColor = "blue" # color of the map polygons
    startColor = "green" # color of the starting point
    finishColor = "red" # color the the finish point

    #defining a function to plot a polygon :
    def polyPlot(poly,color):
        for i in range(1,len(poly)):
            plt.plot([ poly[i-1][0],poly[i][0] ],[ poly[i-1][1],poly[i][1] ],color=color)
        plt.plot([ poly[0][0],poly[len(poly)-1][0] ],[ poly[0][1],poly[len(poly)-1][1] ], color=color)

    #setting map limits
    plt.xlim(0,100)
    plt.ylim(0,100)

    fig = plt.gcf()
    ax = fig.gca()

    ## we need origin to right so we flip the plot
    ax.invert_yaxis()
    fig.set_size_inches((fsz,fsz))

    ## plotting polygons
    for poly in map:
        polyPlot(poly,polyColor)
    
    #Displaying start and finish
    startCircle = plt.Circle(start,radius=1,color=startColor)
    ax.add_artist(startCircle)

    finishCircle = plt.Circle(finish,radius=1,color=finishColor)
    ax.add_artist(finishCircle)

    #displaying the map
    plt.show()

""" Global Navigation Class """
class Global:
    """ Handles global path planning """
    i = 12345

    def __init__(self, polyMap, start, finish):
        self.map = polyMap
        self.start = start
        self.finish = finish

    def plot(self):
        mapPlot(self.map,self.start,self.finish)

    def vPaths(self):
        vertices = functools.reduce(operator.iconcat,self.map)
        paths = list(itertools.combinations(vertices,2))
        print(paths)

    def returnPath(self,map,startPoint,endPoint):
        """ takes map: an array of scipy polygons, startPoint 
        and endPoint two scipy vectors (3d) returns a 
        list of 3d scipy vectors representing the points along the path"""
        return ""




#print(test.f())

start = (80,30)
finish = (20,70)

test = Global(TestMap(),start,finish)

test.vPaths()