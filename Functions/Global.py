""" Developped by: Titouan """

from Utilities import Utilities
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as plt_colors
import networkx as nx
import numpy as np
import itertools
import operator
import copy


""" Global Navigation Class """
class Global:
    """ Handles global path planning """

    #constructor
    def __init__(self, polyMap, start, finish):

        self.polyMap = []
        
        for poly in polyMap:
            nPoly = []
            for p in poly:
                nPoly.append( (float(p[0][0]),float(p[0][1])))
            self.polyMap.append(nPoly)

        self.start = start
        self.finish = finish
        self.navGraph = nx.Graph()
        self.utilities = Utilities()
        self.gPath = False
        self.path = False

    #plots the map and paths
    def plot(self,debugLines=False,emphasedLines=False):
        # I guess in the end we may just use the vision module map plot function
        self.utilities.mapPlot(self.polyMap,self.start,self.finish,debugLines,emphasedLines)

    #computes and plot a path (mostly for showoff purposes)
    def plotPath(self,plotMap=True,plotGraph=False):
        path = self.returnPath(self.polyMap,self.start,self.finish)
        dpath = []
        for i in range(len(path)-1):
            dpath.append((path[i],path[i+1]))
        if plotMap:
            self.plot(self.computePaths(),dpath)
        if plotGraph:
            self.netPlot(self.navGraph,self.gPath,self.start,self.finish)
        self.path = path
        return path
        
    # plots the map as a networkx graph
    def netPlot(self,G,path=False,start = False, finish = False):
        def generatePos(nodes):
            pos = {}
            for node in nodes:
                pos[node] = [node[0],-node[1]] # node[1] is negative because we use top-right coords
            return pos

        norm = plt_colors.Normalize(vmin=0, vmax=150)

        pos = generatePos(G.nodes)
        
        nx.draw(G,pos,node_color="#666666",width=1,with_labels=True)
        
        

        nodeStart = ( (start[0],start[1]) )
        nodeFinish = ( (finish[0],finish[1]) )
        
        if path != False:
            nx.draw_networkx_nodes(G,pos,nodelist=path,node_color='r')
            path_edges = []
            for i in range(len(path)-1):
                edge = ((path[i][0],path[i][1]),(path[i+1][0],path[i+1][1]))
                path_edges.append(edge)
            nx.draw_networkx_edges(G,pos,edgelist=path_edges,edge_color='r',width=3)
        if start != False:
            nx.draw_networkx_nodes(G,pos,nodelist=[nodeStart],node_color='g')
        if start != False:    
            nx.draw_networkx_nodes(G,pos,nodelist=[nodeFinish],node_color='b')
        plt.show()

    # Computes possible vertex the robot can drive to (first vertex in array is start, last is finish)
    def navPoints(self):
        #print(list(self.polyMap[i].exterior.coords))
        #polyVertices = [list(poly.exterior.coords) for poly in self.polyMap]
        mapVertices = []
        for poly in self.polyMap:
            for vert in poly:
                mapVertices.append(vert)
        

        #mapVertices = [y for x in polyVertices for y in x]
        vertices = copy.deepcopy(mapVertices)
        vertices.insert(0,self.start)
        vertices.append(self.finish)
        
        return vertices

    #returns all segements of the map polygons (usefull for intersection)
    def mapSegments(self):
        #polyList = [list(poly.exterior.coords) for poly in self.polyMap]
        segments = []
        for poly in self.polyMap:
            segs = list(itertools.combinations(poly,2))
            for seg in segs:
                segments.append(seg)
        return segments

    def intersect(self,seg1,seg2):
        def intersectHelper(s1,s2):
            def val(p,q,r):
                return (q[1]-p[1])*(r[0]-q[0])-(q[0]-p[0])*(r[1]-q[1])
            v = val(s1[0],s1[1],s2[0]) * val(s1[0],s1[1],s2[1])
            return v < 0
            
        return intersectHelper(seg1,seg2) and intersectHelper(seg2,seg1)


    #computes all possible paths and generates the weighted graph for dikjstra
    def computePaths(self):

        def visiblePaths(unvisited):
            mapSeg = self.mapSegments()
            current = unvisited.pop(0)

            paths = []
            nextVertices = []

            for succ in unvisited:
                newSeg = (current,succ)
                flag = False

                paths.append(newSeg)
                nextVertices.append(succ)
                for ms in mapSeg:
                    if self.intersect(ms,newSeg):
                        paths.pop()
                        nextVertices.pop()
                        break
            return paths

        unvisited = self.navPoints();
        paths = []
        while len(unvisited) > 1:
            nPaths = visiblePaths(unvisited)
            for p in nPaths:
                paths.append(p)
        

        return paths

    #generates a weighted graph that can then be used to compute shortest path
    def computeGraph(self):
        rawVerts = self.navPoints()
        rawEdges = self.computePaths()
        vertices = []
        edges = []

        self.navGraph = nx.Graph()
        for vert in rawVerts:
            vertices.append( (vert[0],vert[1]) )
        for edge in rawEdges:
            edges.append( [(edge[0][0],edge[0][1]),(edge[1][0],edge[1][1])] )

        self.navGraph.add_nodes_from(vertices)
        for e in edges:
            l=np.linalg.norm(np.array(e[0])-np.array(e[1]))
            self.navGraph.add_edge(e[0],e[1],weight=l)
        
        return self.navGraph

    def returnPath(self,map,startPoint,endPoint):
        """ takes map: an array of scipy polygons, startPoint 
        and endPoint two scipy vectors (3d) returns a 
        list of 3d scipy vectors representing the points along the path"""

        self.computeGraph()
        graphPath = nx.dijkstra_path(self.navGraph,source=self.start,target=self.finish)
        self.gPath = graphPath
        return graphPath