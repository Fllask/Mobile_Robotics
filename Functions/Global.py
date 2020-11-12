""" Developped by: Titouan """

import matplotlib.pyplot as plt
from shapely import geometry as geo
import itertools
import operator
import copy

""" just returns a test map so that test runs of the planning algorithm may be computed """
def TestMap():
    polyA = geo.Polygon([(40.,30.),
                         (40.,50.),
                         (45.,40.),
                         (45.,35.)])

    polyB = geo.Polygon([(70.,35.),
                         (60.,46.),
                         (50.,63.),
                         (70.,55.)])

    polyC = geo.Polygon([(35.,60.),
                         (35.,70.),
                         (42.,76.),
                         (45.,60.)])

    return [polyA,polyB,polyC]

""" placeholder map plotting function """
def mapPlot(map,start,finish,debugLines=False):
    #plotting parameters
    fsz = 8 # size of the plotted figure
    polyColor = "blue" # color of the map polygons
    startColor = "green" # color of the starting point
    finishColor = "red" # color the the finish point
    debugLineColor = "black"

    #defining a function to plot a polygon :
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
        plt.plot(*poly.exterior.xy,color=polyColor)
    
    #Displaying start and finish
    startCircle = plt.Circle(start,radius=1,color=startColor)
    ax.add_artist(startCircle)
    finishCircle = plt.Circle(finish,radius=1,color=finishColor)
    ax.add_artist(finishCircle)

    #Plotting the debug lines
    if debugLines != False:
        for line in debugLines:
            plt.plot([ line[0][0],line[1][0] ],[ line[0][1],line[1][1] ],color=debugLineColor,linestyle="dashed")

    #displaying the map
    plt.show()

""" Global Navigation Class """
class Global:
    """ Handles global path planning """

    #constructor
    def __init__(self, polyMap, start, finish):
        self.polyMap = polyMap
        self.start = start
        self.finish = finish

    #plots the map and paths
    def plot(self,debugLines=False):
        mapPlot(self.polyMap,self.start,self.finish,debugLines)

    # Computes possible vertex the robot can drive to (first vertex in array is start, last is finish)
    def navPoints(self):
        #print(list(self.polyMap[i].exterior.coords))
        polyVertices = [list(poly.exterior.coords) for poly in self.polyMap]
        mapVertices = [y for x in polyVertices for y in x]
        vertices = copy.deepcopy(mapVertices)
        vertices.insert(0,self.start)
        vertices.append(self.finish)
        
        return vertices

    #returns all segements of the map polygons (usefull for intersection)
    def mapSegments(self):
        polyVertices = [list(poly.exterior.coords) for poly in self.polyMap]
        mapVertices = [y for x in polyVertices for y in x]
        segments = list(itertools.combinations(mapVertices,2))
        return segments

    def intersect(self,seg1,seg2):
        for i in range(2):
            for j in range(2):
                if(seg1[i]==seg2[j]):
                    return False
        line1 = geo.LineString(seg1)
        line2 = geo.LineString(seg2)
        return line1.intersects(line2)


    #computes all possible paths
    def paths(self):

        def visiblePaths(unvisited):

            mapSeg = self.mapSegments()
            current = unvisited.pop(0)
            paths = []

            for succ in unvisited:
                newSeg = (current,succ)
                flag = False

                paths.append(newSeg)
                for ms in mapSeg:
                    if self.intersect(ms,newSeg):
                        paths.pop()
                        break

            return paths

        unvisited = self.navPoints();
        paths = visiblePaths(unvisited)
        

        return paths

    def returnPath(self,map,startPoint,endPoint):
        """ takes map: an array of scipy polygons, startPoint 
        and endPoint two scipy vectors (3d) returns a 
        list of 3d scipy vectors representing the points along the path"""
        return ""




start = (80.,20.)
finish = (20.,70.)

test = Global(TestMap(),start,finish)

test.plot(test.paths())