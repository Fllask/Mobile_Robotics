import pickle
import matplotlib.pyplot as plt
import math
import numpy as np
import Functions.Utilities as ut


"""
    Class that handles visualization of recorded data
    @autor: Titou
"""
class  Visualization:

    def loadData(self,input):
        f = open(input, "rb")
        data = pickle.load(f)
        return data

    def loadKeySeq(self, data, key):
        retList = []
        for item in data:
            retList.append(item[key])
            
        return retList  

    def parseNone(self, rawValues, index = 0):
        ret = []
        for el in rawValues:
            if el is None:
                ret.append(0.0)
            elif isinstance(el,np.ndarray):
                ret.append(el[index])
            else:
                ret.append(el)
        return ret


    def plotSensor(self, data):
        if isinstance(data,str):
            data = self.loadData(data)

        sensor = self.loadKeySeq(data,'sensor')
        time = self.loadKeySeq(data,'time')

        for i in range(5):
            rawsensor = self.parseNone(sensor,i)
            plt.plot(time,rawsensor)
            plt.show()


    def plotAstolfi(self,data):

        #if input is a string load the file at the adress pointed to by the string
        if isinstance(data,str):
            data = self.loadData(data)

        """
            Parsing the data
        """


        time = self.loadKeySeq(data,'time')
        visTimes = self.loadKeySeq(data,'vtime')
        ML = self.loadKeySeq(data,'ML')
        MR = self.loadKeySeq(data,'MR')

        rawRho = self.loadKeySeq(data,'rho')
        rho = self.parseNone(rawRho)

        rawAlpha = self.loadKeySeq(data,'alpha')
        alpha = self.parseNone(rawAlpha)

        rawBeta = self.loadKeySeq(data,'beta')
        beta = self.parseNone(rawBeta)

        """
            Plotting the data
        """

        plt.plot(time,ML, 'bo', color = 'red') 
        plt.plot(time,MR, 'bo', color = 'blue')
        plt.title("Motor Speed Values (left motor in red, right motor in blue) over time")
        plt.show()

        plt.plot(time,alpha, color = 'blue')
        plt.plot(time,beta, color = 'red')
        plt.title("Alpha (blue) and Beta (red) of ASTOLFI over time")
        plt.show()

        plt.plot(time,rho, color = 'green')
        plt.title("Rho of ASTOLFI over time")
        plt.show()

    def mapPlot(self,map=False,start=False,finish=False,visPos=False, kalPos = False):
        #plotting parameters
        fsz = 8 # size of the plotted figure
        polyColor = "blue" # color of the map polygons
        polyFillColor = "#b8e7f5" # color of interior of the map polygons
        startColor = "green" # color of the starting point
        finishColor = "red" # color the the finish point
        debugLineColor = "red"
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
        if map is not False:
            polyMap = []
            for poly in map:
                nPoly = []
                for p in poly:
                    nPoly.append( (float(p[0][0]),float(p[0][1])))
                polyMap.append(nPoly)
                
            for poly in polyMap:
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
        if start is not False:
            startCircle = plt.Circle(start,radius=1,color="black")
            ax.add_artist(startCircle)
            ax.text(*start,"Start",fontsize=11,color=startColor,weight="bold")
        if finish is not False:
            finishCircle = plt.Circle(finish,radius=1,color="black")
            ax.add_artist(finishCircle)
            ax.text(*finish,"Finish",fontsize=11,color=finishColor,weight="bold")
            
        if visPos is not False:
            for i in range(1,len(visPos)):
                if ( not isinstance(visPos[i],bool) ) and ( not isinstance(visPos[i-1],bool) ):
                    plt.plot([ visPos[i-1][0],visPos[i][0] ],[ visPos[i-1][1],visPos[i][1] ],color="red",linestyle="dashed",linewidth=2)
                        
        if kalPos is not False:
            for i in range(1,len(kalPos)):
                if ( not isinstance(kalPos[i],bool) ) and ( not isinstance(kalPos[i-1],bool) ):
                    plt.plot([ kalPos[i-1][0],kalPos[i][0] ],[ kalPos[i-1][1],kalPos[i][1] ],color="blue",linestyle="dashed",linewidth=2)
                        
        #displaying the map
        plt.show()

    def plotTrajectories(self,data):
        if isinstance(data,str):
            data = self.loadData(data)

        visPos = self.loadKeySeq(data,'visPos')

        kalPos = self.loadKeySeq(data,'fltPos')

        start = (data[0]['visPos'][0],data[0]['visPos'][1])
        goal = data[-1]['path'][-1]

        mp = data[-1]['map']
        self.mapPlot(map = mp,start=start, finish = goal, visPos=visPos,kalPos = kalPos)

    def metaPlotter(self,data,plotType):
        if plotType == 'trace':
            self.plotTrajectories(data)
        elif plotType == 'ASTOLFI':
            self.plotAstolfi(data)
        elif plotType == 'sensor':
            self.plotSensor(data)