""" Developped by: Flask """
import cv2 #read video and images
import numpy as np
from skimage import measure, morphology
from scipy import linalg
import math

def getTransform(image):
    #return a geometric transform (usable with cv2.warpPerspective or with )
    invalid = False
    filr = colorfilter("RED")
    filg = colorfilter("GREEN")
    filb = colorfilter("BLUE")
    fily = colorfilter("YELLOW")
    maskr= filr.get_mask(image)
    BL,fr = getCentroid(maskr)
    if fr:
        invalid = True
    maskg= filg.get_mask(image)
    TL,fg = getCentroid(maskg)
    if fg:
        invalid = True
    maskb= filb.get_mask(image)
    BR,fb = getCentroid(maskb)
    if fb:
        invalid = True
    masky= fily.get_mask(image)
    TR,fy = getCentroid(masky)
    if fy:
        invalid = True
    if invalid:
        corners = np.array([[0,0],[624,0],[0,416],[624,416]], np.float32)
    else:
        corners = np.array([TL,TR,BL,BR], np.float32)
    trans = projection(corners)
    return trans,invalid

class Vision:
    """ Handles vision """
    i = 12345

    def __init__(self,image):
        self.trans, invalid = getTransform(image)
        self.map = createMap(image,self.trans)
        if invalid:
            print("initialisation failed")

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
        self.color = color
        if color == "RED":
            self.band = np.array([[0,11],[142,255],[30,255]])
        if color == "YELLOW":
            self.band = np.array([[25,33],[92,255],[92,255]])
        if color == "BLUE":
            self.band = np.array([[110,130],[161,255],[41,220]])
        if color == "GREEN":
            #filter in HSV
             self.band = np.array([[34,78],[135,255],[76,217]])
        if color == "ROBOT":
            self.band = np.array([[0.547,0.631],[0.145,0.281],[0.906,1]])
        if color == "BLACK":
             self.band = np.array([[0,179],[0,255],[0,25]])
             self.morph = True
    def get_mask(self,image):
        mask = cv2.inRange(image,self.band[:,0],self.band[:,1])/255
        
        #c'est sale, mais comme ça ça passe le wrapping
        if self.color == "RED":
            mask += cv2.inRange(image,np.array([173,142,30]),np.array([179,255,255]))
            cv2.threshold(mask,0.5,1,0,dst = mask)
        if self.morph:
            morphology.binary_opening(mask,selem = morphology.square(3), out = mask)
            morphology.binary_closing(mask,selem = morphology.square(7), out = mask)
        else:
            morphology.binary_opening(mask, selem = morphology.disk(4),out=mask)
            morphology.binary_closing(mask,out=mask,selem = morphology.disk(3))

        return mask

def preprocess(img):
    imgsmall = cv2.resize(img,(624,416))
    imgsmooth = np.copy(imgsmall)
    cv2.GaussianBlur(imgsmall,(5,5),0,dst = imgsmooth)
    imgHSV = cv2.cvtColor(imgsmooth,cv2.COLOR_BGR2HSV)
    imgHSV[:,:,1] = cv2.equalizeHist(imgHSV[:,:,1])
    imgHSV[:,:,2] = cv2.equalizeHist(imgHSV[:,:,2])
    return imgHSV
    
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
    redflag = 0
    moments = measure.moments(imageBin, order = 2)
    centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
    varx = moments[0,2]/moments[0,0]-centroid[0]**2
    vary = moments[2,0]/moments[0,0]-centroid[1]**2
    if (np.isnan(varx)) or (max(varx,vary)>math.sqrt(imageBin.size)):
        print("invalide centroid")
        redflag = True
    return centroid,redflag


def getRobotPos(imageBin):
    moments = measure.moments(imageBin, order = 3)
    centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
    varx = moments[0,2]/moments[0,0]-centroid[0]**2
    vary = moments[2,0]/moments[0,0]-centroid[1]**2
    varxy = moments[1,1]/moments[0,0]-centroid[0]*centroid[1]
    
    #get the angle
    phi = math.atan(2*varxy/(varx-vary))/2 +(varx<vary)*math.pi/2
    
    #check direction
    cm3x = moments[0,3] \
           -3*moments[0,2]*moments[0,1]/moments[0,0]\
           +2*moments[0,1]**3/moments[0,0]**2
   
    cm3y = moments[3,0] \
       -3*moments[2,0]*moments[1,0]/moments[0,0]\
       +2*moments[1,0]**3/moments[0,0]**2

    print(cm3x)
    print(cm3y)
    
    if abs(cm3x) >abs(cm3y):
        print("hor")
        if cm3x < 0:
            phi+= math.pi
    else:
        print("vert")
        if cm3y < 0:
            phi+= math.pi
    pos = np.append(centroid,phi)
    return pos
    
    

