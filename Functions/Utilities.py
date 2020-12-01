import numpy as np
import matplotlib.pyplot as plt
import math as m

class Utilities:
    """Utilities for testing and displaying"""

    def __init__(self):
        self.data = []

    """ 
    Returns a test map so that test runs of the planning algorithm 
    may be computed, also works as a reference of what a map object should look like
    """
    def TestMap(self):
        polyA = [[40.,30.],
                 [40.,50.],
                 [45.,40.],
                 [45.,35.]]

        polyB = [[70.,35.],
                 [60.,46.],
                 [50.,63.],
                 [70.,55.]]

        polyC = [[35.,60.],
                 [35.,70.],
                 [42.,76.],
                 [45.,60.]]

        polyD = [[20.,70.],
                 [10.,90.],
                 [42.,96.],
                 [45.,90.]]

        return [polyA,polyB,polyC,polyD]
    
    """ 
    Plots a map plotting function
    INPUTS :
        REQUIRED : 
            map <- an array of array of vertices : arrray
            in the array describes a single polygon (see test-map for example)

            start <- a 2 element array representing the coordinates of the starting point of the robot

            finish <- a 2 element array representing the coordinates of the point we want the robot to go to

        OPTIONAL :
            debugLines <- an array of 2 elements array of 2 coordinates, each of the two element array will be plotted really with a small width : usefull to represent loads of data (I used it to plot all the valid paths)

            emphasedLines <- an array of 2 elements array of 2 coordinates, each of the two element array will be plotted really a thick width in green : usefull to represent important stuff, such as the computed path
    """
    def mapPlot(self,map=False,start=False,finish=False,debugLines=False,emphasedLines = False):
        #plotting parameters
        fsz = 8 # size of the plotted figure
        polyColor = "blue" # color of the map polygons
        polyFillColor = "#b8e7f5" # color of interior of the map polygons
        startColor = "green" # color of the starting point
        finishColor = "red" # color the the finish point
        debugLineColor = "black"
        emphasedLineColor = "green"

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
        if map !=False:
            for poly in map:
                for i in range(len(poly)-1):
                    plt.plot([ poly[i][0],poly[i+1][0] ],[ poly[i][1],poly[i+1][1] ],color=polyColor)
                plt.plot([ poly[len(poly)-1][0],poly[0][0] ],[ poly[len(poly)-1][1],poly[0][1] ],color=polyColor)
                x = []
                y = []
                for vertex in poly:
                    x.append(vertex[0])
                    y.append(vertex[1])
                plt.Polygon(poly,color=polyColor)
                ax.fill(x,y,color = polyFillColor)  
        
        #Displaying start and finish
        if start != False:
            startCircle = plt.Circle(start,radius=1,color="black")
            ax.add_artist(startCircle)
            ax.text(*start,"Start",fontsize=11,color=startColor,weight="bold")
        if finish != False:
            finishCircle = plt.Circle(finish,radius=1,color="black")
            ax.add_artist(finishCircle)
            ax.text(*finish,"Finish",fontsize=11,color=finishColor,weight="bold")

        #Plotting the debug lines
        if debugLines != False:
            for line in debugLines:
                plt.plot([ line[0][0],line[1][0] ],[ line[0][1],line[1][1] ],color=debugLineColor,linestyle="dashed",linewidth=0.5)
        #Plotting the emphased lines
        if emphasedLines != False:
            for line in emphasedLines:
                plt.plot([ line[0][0],line[1][0] ],[ line[0][1],line[1][1] ],color=emphasedLineColor,linestyle="solid",linewidth=4)

        #displaying the map
        plt.show()

    def compute_angle(self,Pos1,Pos2):
        angle_value=m.atan2(Pos2[1]-Pos1[1],Pos2[0]-Pos1[0])
        return angle_value

    def compute_distance(self,Pos1,Pos2):
        dist=m.sqrt((Pos1[0]-Pos2[0])**2 + (Pos1[1]-Pos2[1])**2 )
        return dist

    def init_robot(self):

        Init_pos = False
        Ts = 0.1
        kp = 3    #0.15   #0.5
        ka = 35  #0.4    #0.8
        kb = -8   #-0.07  #-0.2
        vTOm=31.5 #30.30
        wTOm=(200*180)/(80*math.pi) #130.5 #

        global_path = d['path']
        thym = Robot(global_path,Init_pos,Ts, kp,ka,kb,vTOm,wTOm)
        return thym

    def init_filter(self,thym):

        Rvel = np.array([[1.53, 0.], [0.,1.53]])
        Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
        Rcam = np.array([[0.000001,0.,0.],[0.,0.000001,0.],[0.,0.,0.000001]])
        Hcam = np.array([[1.,0.,0.,0.,0.],[0.,1.,0.,0.,0.],[0.,0.,1.,0.,0.]])

        filter = Filtering(Rvel, Rcam, thym, Hvel, Hcam,Ts)

        return filter