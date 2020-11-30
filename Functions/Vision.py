""" Developped by: Flask """
import cv2 #read video and images
import numpy as np
from skimage import measure, morphology
from scipy import linalg, ndimage
import math

DEFAULT = 0
BIG = 1
NONE = 2


def getTransform(image,camera):
    #return a geometric transform (usable with cv2.warpPerspective or with )
    image = preprocess(image)
    invalid = False
    filr = colorfilter("RED",camera)
    filg = colorfilter("GREEN",camera)
    filb = colorfilter("BLUE",camera)
    fily = colorfilter("YELLOW",camera)
    
    maskr= filr.get_mask(image)
    BL,fr = getCentroid(maskr)
    if fr:
        invalid = True
        print("RED ERROR")
    maskg= filg.get_mask(image)
    TL,fg = getCentroid(maskg)
    if fg:
        print("GREEN ERROR")

        invalid = True
    maskb= filb.get_mask(image)
    BR,fb = getCentroid(maskb)
    if fb:
        print("BLUE ERROR")
        invalid = True
    masky= fily.get_mask(image)
    TR,fy = getCentroid(masky)
    if fy:
        print("YELLOW ERROR")
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

    def __init__(self,image, camera = "ANDROID FLASK"):
        self.camera = camera
        self.trans, invalid = getTransform(image, camera)
        self.setframe(image)    #generate self.frame
        self.map = createMap(self.frame,camera = camera)
        if invalid:
            print("initialisation failed")

    def getMap(self, downscale = True):
        #return a map of polygons (numpy array of size (n.polygons, n.corners,1,2))
        #the corners of the maps are at (0,0) to (100,100)
        if downscale:
            return np.array([np.array(el).astype(float)/5 for el in self.map])
        else:
            return self.map

    def setframe(self, imgraw):
        img_prep = preprocess(imgraw)
        img_real = cv2.warpPerspective(img_prep, self.trans, (500,500))
        self.frame = img_real
        
    
    def returnDynamicCoordinates(self):
        """ returns an object containing the coordinates of the robot (a 3d numpy vector) 
        as well as the end point coordinates (a 2d numpy vector) 
        OR False if the image is not exploitable"""
        '''
    
        '''
        filterob = colorfilter("ROBOT",self.camera)
        mask = filterob.get_mask(self.frame).get().astype(np.uint8)
        pos,valid = getRobotPos(mask)
        return pos,valid
    def returnDynamicAim(self):
        '''
        Returns the coodinates of the aim (in the 100x100 scale)
        -------
        TYPE
            DESCRIPTION. to set a frame, first use self.setframe(img)
                
        '''
        filteraim = colorfilter("FINISH",self.camera)
        mask = filterob.get_mask(self.frame)
        finish = getCentroid(mask)
        return finish/5
    
class colorfilter:
    def __init__(self, color, camera = "XT3"):
        self.morph = DEFAULT
        self.camera = camera
        self.color = color
        if camera == "XT3":
            if color == "RED":
                self.band = np.array([[0,11],[142,255],[30,255]])
            if color == "YELLOW":
                self.band = np.array([[25,33],[92,255],[92,255]])
            if color == "BLUE":
                self.band = np.array([[110,130],[161,255],[41,220]])
            if color == "GREEN":
                 self.band = np.array([[34,78],[135,255],[76,217]])
            if color == "ROBOT":
                self.band = np.array([[85,109],[107,255],[71,255]])
                #self.morph = NONE
            if color == "BLACK":
                 self.band = np.array([[0,179],[0,255],[0,25]])
                 self.morph = BIG
        if camera == "ANDROID FLASK":
            if color == "RED":
                self.band = np.array([[0,8],[180,255],[128,255]])
            if color == "YELLOW":
                self.band = np.array([[22,25],[169,255],[210,255]])
            if color == "BLUE":
                self.band = np.array([[115,127],[73,255],[17,191]])
            if color == "GREEN":
                 self.band = np.array([[41,67],[200,255],[36,255]])
            if color == "ROBOT":
                self.band = np.array([[151,171],[75,255],[60,255]])
                self.morph = NONE
            if color == "BLACK":
                 self.band = np.array([[0,115],[0,255],[0,55]])
                 self.morph = BIG
            if color == "FINISH":
                self.band = np.array([[80,105],[100,255],[40,255]])
                self.morph = NONE

    def get_mask(self,image):
        mask = np.multiply(cv2.inRange(image,self.band[:,0],self.band[:,1]).get().astype(np.uint8),(1/255))
        # (wx,wy) = mask.shape

        # print(str(wx) + " " + str(wy))
        #c'est sale, mais comme ça ça passe le wrapping
        if self.color == "RED":
            img = image.get().astype(np.uint8)
            if self.camera == "ANDROID FLASK":
                mask += cv2.inRange(img,np.array([173,180,128]),np.array([179,255,255]))
                cv2.threshold(mask,0.5,1,0,dst = mask)

        if self.color == "BLACK":
            img = image.get().astype(np.uint8)
            if self.camera == "ANDROID FLASK":
                 mask += cv2.inRange(img,np.array([127,0,0]),np.array([179,255,55]))
                 cv2.threshold(mask,0.5,1,0,dst = mask)
            
        mask = cv2.UMat(mask) ## convert the mask to an opencl object for fast morphology
            
        if self.morph == BIG:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((6,6)).astype("uint8"))
        elif self.morph == DEFAULT:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7,7)).astype("uint8"))
        elif self.morph == NONE:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((4,4)).astype("uint8"))
        return cv2.UMat(mask)

def preprocess(img):
    imgsmall = cv2.resize(img,(624,416))
    imgsmooth = cv2.UMat(imgsmall)
    cv2.GaussianBlur(imgsmall,(5,5),0,dst = imgsmooth)
    imgHSV = cv2.cvtColor(imgsmooth,cv2.COLOR_BGR2HSV).get().astype(np.uint8)
    imgHSV[:,:,1] = cv2.equalizeHist(imgHSV[:,:,1])
    imgHSV[:,:,2] = cv2.equalizeHist(imgHSV[:,:,2])
    return cv2.UMat(imgHSV)
    
def projection(corners):
    #corners: coordinates of TL TR BL BR in the image []
    
    #project TL on (5,5), then calculate the least mean square transformation
    
    rcorners = np.array([[5,5],[95,5],[5,95],[95,95]],np.float32)*5
    trans = cv2.getPerspectiveTransform(corners, rcorners)
    return trans
def applyTransform(trans,x):
    #note that trans transform an image of size(img) to a (500,500) grid
    #to get a correct transform if you use it, please divise the input by 5
    x = x.reshape(2)
    xp = np.append(x,1)
    yp = np.dot(linalg.inv(trans),xp)
    y = (yp/yp[2])[0:2]
    return y

def createMap(img,R_ROBOT = 65,camera= "XT3"):
    
    filter_poly = colorfilter("BLACK",camera)

    maskpoly = filter_poly.get_mask(img)

    #maskpoly = cv2.resize(maskpoly,(500,500), interpolation=cv2.INTER_NEAREST)
    yd,xd = np.ogrid[-R_ROBOT: R_ROBOT+1, -R_ROBOT: R_ROBOT+1]
    disk = (xd**2+yd**2 <= (R_ROBOT//2)**2).astype('uint8')
    margin = cv2.dilate(maskpoly,disk,iterations = 1)
    polyprojbin = margin.get().astype(np.uint8)

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
    #frame = cv2.VideoCapture(0).read()
    
    small_frame = cv2.resize(frame,(624,416))
    return cv2.UMat(small_frame)



def getCentroid(imageBin):
    invalid = False
    img = imageBin.get().astype(np.uint8)
    moments = measure.moments(img, order = 2)
    if moments[0,0]:
        centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
        varx = moments[0,2]/moments[0,0]-centroid[0]**2
        vary = moments[2,0]/moments[0,0]-centroid[1]**2
        if max(varx,vary)>img.size/50:
            print("invalid centroid:noise")
            invalid = True
    else:
        print("invalid centroid: no pixel")
        invalid = True
        centroid = np.array([0,0])

    return centroid,invalid


def getRobotPos(imageBin, verbose = 0):
    if type(imageBin) is cv2.UMat:
        imageBin = imageBin.get().astype(np.uint8)
    valid = True
    moments = measure.moments(imageBin, order = 3)
    if moments[0,0]>800:
        centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
        varx = moments[0,2]/moments[0,0]-centroid[0]**2
        vary = moments[2,0]/moments[0,0]-centroid[1]**2
        varxy = moments[1,1]/moments[0,0]-centroid[0]*centroid[1]
        
        #check the variance of the image
        if max(varx,vary)>2*imageBin.size**0.5:
            print("invalide coord:noise")
            valid = False
        #get the angle
        if abs(varx-vary)<0.0001:
            phi = math.pi/4
        else:
            phi = math.atan(2*varxy/(varx-vary))/2 +(varx<vary)*math.pi/2
        if verbose:
            print(phi)
        #check direction
        imgsegmented = imageBin[int(centroid[1]-50):int(centroid[1]+50)\
                                ,int(centroid[0]-50):int(centroid[0]+50)]
        
        # print(imgsegmented.shape)
        #cv2.imshow("seg",imgsegmented*255)
        # print(phi)
        imgrot = ndimage.rotate(imgsegmented,phi*180/math.pi, reshape = False)
        #cv2.imshow("rot",imageBin*255)
        newmoments = measure.moments(imgrot)
        cm03 = newmoments[0,3] \
               -3*newmoments[0,2]*newmoments[0,1]/newmoments[0,0]\
               +2*newmoments[0,1]**3/newmoments[0,0]**2
        if cm03 < 0:
            phi+=math.pi
        pos = np.append(centroid,phi)
    else:
        print("invalide coord: no or not enough pixel")
        valid = False
        pos = np.array([0,0,0])
        
    pos[0:2] = pos[0:2]/5


    return pos,valid
    
    