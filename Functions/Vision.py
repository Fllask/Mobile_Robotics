""" Developped by: Flask """
import cv2 #read video and images
import numpy as np
from skimage import measure, morphology
from scipy import linalg


def getTransform(image):
    #return a geometric transform (usable with cv2.warpPerspective or with )
    filr = colorfilter("RED")
    filg = colorfilter("GREEN")
    filb = colorfilter("BLUE")
    fily = colorfilter("YELLOW")
    maskr= filr.get_mask(image)
    BL = getCentroid(maskr)
    maskg= filg.get_mask(image)
    TL = getCentroid(maskg)
    maskb= filb.get_mask(image)
    BR = getCentroid(maskb)
    masky= fily.get_mask(image)
    TR = getCentroid(masky)
    corners = np.array([TL,TR,BL,BR], np.float32)
    trans = projection(corners)
    return trans

class Vision:
    """ Handles vision """
    i = 12345

    def __init__(self,image):
        self.trans = getTransform(image)
        self.map = createMap(image,self.trans)
        

    def getMap(self, downscale = True):
        #return a map of polygons (numpy array of size (n.polygons, n.corners,1,2))
        #the corners of the maps are at (0,0) to (100,100)
        if downscale:
            return np.array([np.array(el).astype(float)/10 for el in self.map])
        else:
            return self.map


    def returnDynamicCoordinates(self):
        """ returns an object containing the coordinates of the robot (a 3d numpy vector) 
        as well as the end point coordinates (a 2d numpy vector) 
        OR False if the image is not exploitable"""
        coord = Features()
        if coord.isUsable():
            return coord
        else:
            return False

    
class colorfilter:
    def __init__(self, color):
        self.morph = False
        if color == "RED":
            self.band = np.array([[6,29],[8,35],[72,97]])
        if color == "YELLOW":
            self.band = np.array([[0,39],[124,166],[139,181]])
        if color == "BLUE":
            self.band = np.array([[37,65],[11,38],[1,28]])
        if color == "GREEN":
             self.band = np.array([[11,41],[58,94],[46,77]])
        if color == "BLACK":
             self.band = np.array([[0,39],[0,50],[0,50]])
             self.morph = True
    def get_mask(self,image):
        mask = np.zeros((image.shape[0], image.shape[1]))
        for x in range(image.shape[0]):
            for y in range(image.shape[1]):
                pix = image[x][y]
                #print((pix<self.band[:,1]).all())
                if (pix>self.band[:,0]).all() & (pix<self.band[:,1]).all():
                    mask[x,y] = 1
                    #print("CAL")
        if self.morph:

            morphology.binary_opening(mask,selem = morphology.square(3), out = mask)
            morphology.binary_closing(mask,selem = morphology.square(3), out = mask)
        else:
            morphology.binary_opening(mask, out=mask)
            morphology.binary_closing(mask,out=mask)
        return mask
    
    
def projection(corners):
    #corners: coordinates of TL TR BL BR in the image []
    
    #project TL on (5,5), then calculate the least mean square transformation
    
    rcorners = np.array([[5,5],[95,5],[5,95],[95,95]],np.float32)*10
    trans = cv2.getPerspectiveTransform(corners, rcorners)
    return trans
def applyTransform(trans,x):
    #note that trans transform an image of size(img) to a (1000,1000) grid
    #to get a correct transform if you use it, please multiply the input by 10
    x = x.reshape(2)
    xp = np.append(x,1)
    yp = np.dot(linalg.inv(trans),xp)
    y = (yp/yp[2])[0:2]
    return y

def createMap(img,trans,R_ROBOT = 65):
    filter_poly = colorfilter("BLACK")
    maskpoly = filter_poly.get_mask(img)
    polyproj	=	cv2.warpPerspective(maskpoly, trans, (1000,1000))
    ret, bin_polygons = cv2.threshold(polyproj,0.2,1,0)
    margin = morphology.binary_dilation(bin_polygons,selem = morphology.disk(R_ROBOT))
    polyprojbin = margin.astype(np.uint8)
    contours, ret = cv2.findContours(polyprojbin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    polygons = []
    for c in contours:
        polygon    =  cv2.approxPolyDP(c, 15, True)
        polygons.append(polygon)
    return polygons

def get_first_frame(input):
    video = cv2.VideoCapture(input)
    if(video.isOpened() == False):
        print("Error opening video")

    #get first frame for object recognition
    video.set(1, 0)
    ret, frame = video.read()
    return frame
def get_image(input):
    frame = cv2.imread(input)
    small_frame = cv2.resize(frame,(624,416))
    return np.array(small_frame)



def getCentroid(imageBin):
    moments = measure.moments(imageBin, order = 1)
    centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
    return centroid

def getcoord(imageBin, trans):
    #get the real coordinates of a the robot
    pose = np.zeros(3)
    pose[0:1] = trans.transform(getCentroid(imageBin))
    
    

