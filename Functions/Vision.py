""" Developped by: Flask """
import cv2 #read video and images
import numpy as np
from skimage import measure, morphology,exposure
from scipy import linalg, ndimage
import math
import pickle

DEFAULT = 0
BIG = 1
NONE = 2

mask_watershed = 0
pmin = 0
pmax = 100
def getTransform(image,camera,prevtrans,percentiles, setmanually = False):
    #return a geometric transform (usable with cv2.warpPerspective)
    image = preprocess(image,percentiles)
    invalid = False
    filr = colorfilter("RED",camera)
    filg = colorfilter("GREEN",camera)
    filb = colorfilter("BLUE",camera)
    fily = colorfilter("YELLOW",camera)
    
    
    maskg= filg.get_mask(image)
    TL,fg = getCentroid(maskg)
    if fg:
        print("GREEN ERROR")
        if setmanually:
            TL = manually_get_centroid(image, preprocessed=True)
        else:
            invalid = True

    masky= fily.get_mask(image)
    TR,fy = getCentroid(masky)
    if fy:
        print("YELLOW ERROR")
        if setmanually:
            TR = manually_get_centroid(image, preprocessed=True)
        else:
            invalid = True
            
            
    maskr= filr.get_mask(image)
    BL,fr = getCentroid(maskr)
    if fr:
        print("RED ERROR")
        if setmanually:
            BL = manually_get_centroid(image, preprocessed=True)
        else:
            invalid = True


            
    maskb= filb.get_mask(image)
    BR,fb = getCentroid(maskb)
    if fb:
        print("BLUE ERROR")
        if setmanually:
            BR = manually_get_centroid(image, preprocessed=True)
        else:
            invalid = True
            
            
    
    if invalid:
        trans = prevtrans
    else:
        corners = np.array([TL,TR,BL,BR], np.float32)
        trans = projection(corners)
    return trans,invalid

class Vision:
    """ Handles vision """
    i = 12345

    def __init__(self,image, camera = "ANDROID FLASK", prevtrans = np.identity(3),\
                 verbose = False, setmanually = False, setpercentiles = False):
        self.camera = camera
        self.percentiles = (2,80) #default value
        if setpercentiles:
            self.percentiles = adjustlum(image,self.percentiles)
            
            
        self.trans, self.invalid = getTransform(image, camera,prevtrans, self.percentiles,setmanually = setmanually)
        self.setframe(image)    #generate self.frame
        if self.invalid:
            print("initialisation failed")
            self.map = False
        else:
            self.map = createMap(self.frame,camera = camera)

    def getMap(self, downscale = True):
        #return a map of polygons (numpy array of size (n.polygons, n.corners,1,2))
        #the corners of the maps are at (0,0) to (100,100)
        if downscale:
            return np.array([np.array(el).astype(float)/5 for el in self.map])
        else:
            return self.map

    def setframe(self, imgraw):
        img_prep = preprocess(imgraw,self.percentiles)
        img_real = cv2.warpPerspective(img_prep, self.trans, (500,500),borderMode=cv2.BORDER_REFLECT_101)
        self.frame = img_real
        
    
    def returnDynamicCoordinates(self,display= 0):
        """ returns an object containing the coordinates of the robot (a 3d numpy vector) 
        as well as the end point coordinates (a 2d numpy vector) 
        OR False if the image is not exploitable"""
        '''
    
        '''
        filterob = colorfilter("ROBOT",self.camera)
        mask = filterob.get_mask(self.frame).get().astype(np.uint8)
        pos,valid = getRobotPos(mask,display = display)
        
        if  not valid:
            pos = False
        
        return pos,valid
    def returnDynamicAim(self):
        '''
        Returns the coodinates of the aim (in the 100x100 scale)
        -------
        TYPE
            DESCRIPTION. to set a frame, first use self.setframe(img)
                
        '''
        filteraim = colorfilter("FINISH",self.camera)
        mask = filteraim.get_mask(self.frame)
        finish,invalid = getCentroid(mask)
        finish /= 5.
        if invalid:
            finish = False
        return finish, not invalid


def adjustlum(img,percentiles):
    cv2.namedWindow("preprocess")
    cv2.namedWindow("corner masks")
    cv2.createTrackbar("max", "preprocess" , 0, 100, on_trackbar_max)
    cv2.createTrackbar("min", "preprocess" , 0, 100, on_trackbar_min)
    while(True):

        
        imgprep = preprocess(img,percentiles= (pmin,pmax))
        
        cv2.imshow("preprocess",cv2.cvtColor(imgprep,cv2.COLOR_HSV2BGR))
        
        maskg = colorfilter("GREEN",camera = "ANDROID FLASK")
        masky = colorfilter("YELLOW",camera = "ANDROID FLASK")
        maskr = colorfilter("RED",camera = "ANDROID FLASK")
        maskb = colorfilter("BLUE",camera = "ANDROID FLASK")
        
        masktot = maskg.get_mask(imgprep)
        masktot = cv2.bitwise_or(masktot,masky.get_mask(imgprep))
        masktot = cv2.bitwise_or(masktot,maskr.get_mask(imgprep))
        masktot = cv2.bitwise_or(masktot,maskb.get_mask(imgprep))
        
        cv2.imshow("corner masks",masktot)
        key = cv2.waitKey(1)
        if key & 0XFF == ord('v'):
            break
    return (pmin,pmax)
def on_trackbar_max(val):
    global pmax
    pmax = val
def on_trackbar_min(val):
    global pmin
    pmin = val
    


class colorfilter:
    def __init__(self, color, camera = "XT3"):
        self.morph = DEFAULT
        self.camera = camera
        self.color = color
        self.inv = False
        if camera == "XT3":
            if color == "RED":
                self.band = np.array([[0,11],[142,255],[30,255]])
            if color == "YELLOW":
                self.band = np.array([[20,30],[90,255],[110,255]])
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
                self.band = np.array([[11,172],[108,255],[70,255]])
                self.inv = True
            if color == "YELLOW":
                self.band = np.array([[15,30],[120,255],[70,255]])
            if color == "BLUE":
                self.band = np.array([[110,130],[95,255],[50,191]])
            if color == "GREEN":
                 self.band = np.array([[41,78],[100,255],[50,255]])
            if color == "ROBOT":
                self.band = np.array([[150,170],[100,255],[100,255]])
                self.morph = NONE
            if color == "BLACK":
                 self.band = np.array([[0,179],[0,255],[0,50]])
                 self.morph = BIG
            if color == "FINISH":
                self.band = np.array([[80,105],[100,255],[70,255]])
                self.morph = NONE

    def get_mask(self,image):
       
        # (wx,wy) = mask.shape

        # print(str(wx) + " " + str(wy))
        #c'est sale, mais comme ça ça passe le wrapping
        if self.inv:
            band1 = np.copy(self.band)
            band2 = np.copy(self.band)
            band1[0,:] = np.array([band1[0,1],179])
            band2[0,:] = np.array([0,band2[0,0]])
            mask1 = np.multiply(cv2.inRange(image,band1[:,0],band1[:,1]).get().astype(np.uint8),(1/255))
            mask2 = np.multiply(cv2.inRange(image,band2[:,0],band2[:,1]).get().astype(np.uint8),(1/255))
            mask = cv2.bitwise_or(mask1,mask2)
        else:
             mask = np.multiply(cv2.inRange(image,self.band[:,0],self.band[:,1]).get().astype(np.uint8),(1/255))

        # if self.color == "BLACK":
        #     img = image.get().astype(np.uint8)
        #     if self.camera == "ANDROID FLASK":
        #          mask += cv2.inRange(img,np.array([127,0,0]),np.array([179,255,55]))
        #          cv2.threshold(mask,0.5,1,0,dst = mask)
            
        mask = cv2.UMat(mask) ## convert the mask to an opencl object for fast morphology
            
        if self.morph == BIG:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((6,6)).astype("uint8"))
        elif self.morph == DEFAULT:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((15,15)).astype("uint8"))
        # elif self.morph == NONE:
        #     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((4,4)).astype("uint8"))
        return cv2.UMat(mask)
        
    
    

    # def change_color(self, img, preprocessed = True, use_watershed = True):
    #     if isinstance(img, cv2.UMat):
    #         img = img.get().astype(int)
    #     else:
    #         img = img.astype(int)
    #     if not preprocessed:
    #         img = preprocess(img)
    #     print("click on "+str(self.color))
    #     cv2.namedWindow("image")
    #     cv2.setMouseCallback("image",watershed,img)
    #     cv2.imshow("image", cv2.cvtColor(img.astype("uint8"),cv2.COLOR_HSV2BGR))
        
        
def manually_get_centroid(img, preprocessed = False):
    global mask_watershed
    if isinstance(img, cv2.UMat):
        img = img.get().astype(int)
    else:
        img = img.astype(int)
    if not preprocessed:
        img = preprocess(img)
    mask_watershed = np.zeros(img.shape[0:2])
    print("CLic on ROI, then press y if the centroid is correct, n to reset")
    while(True):
        cv2.namedWindow("image")
        cv2.setMouseCallback("image",watershed,img)
        img_disp = cv2.cvtColor(img.astype("uint8"),cv2.COLOR_HSV2BGR)
        cv2.imshow("image", img_disp)
        cv2.imshow("masked", cv2.bitwise_and(img_disp,img_disp, mask= mask_watershed.astype("uint8")))
        key = cv2.waitKey(1)
        if key & 0XFF == ord('y'):
            centroid, ret = getCentroid(cv2.UMat(mask_watershed))
            break
        if key & 0XFF == ord('n'):
            mask_watershed = np.zeros(img.shape[0:2])
    cv2.destroyWindow("image")
    cv2.destroyWindow("masked")
    return centroid
def watershed(event, y_ori, x_ori, flags, img):
    if event == cv2.EVENT_LBUTTONDOWN:
        global mask_watershed
        difhmax = 10
        difsmax = 70
        difvmax = 70
        flagWrap = False
        listnew = [(x_ori,y_ori)]
        hue_ori = img[x_ori,y_ori,0]
        visited = np.zeros(img.shape[0:2])
        while len(listnew)>0: 
            coord = listnew.pop(0)
            for x in range(coord[0]-1,coord[0]+1):
                if x >= img.shape[0] or x<0:
                    continue
                for y in range(coord[1]-1,coord[1]+1):
                    if y >= img.shape[1] or y <0:
                        continue
                    if visited[x,y] == 0:
                        #difh = min(abs(img[coord][0]-img[x,y,0]),abs(180-abs(img[coord][0]-img[x,y,0])))
                        difs = abs(img[coord][1]-img[x,y,1])
                        difv = abs(img[coord][2]-img[x,y,2])
                        dif_ori = min(abs(hue_ori-img[x,y,0]),abs(180-abs(hue_ori-img[x,y,0])))
                        if dif_ori<difhmax and difs<difsmax and difv < difvmax:
                            
                            visited[x,y] = 1
                            listnew.append((x,y))
                            if abs(img[coord][0]-img[x,y,0]) > abs(180-abs(img[coord][0]-img[x,y,0])):
                                flagWrap = True
                        # else:
                        #     print("difh: "+str(dif)+" difs: "+str(difs)+" difv: "+str(difv))
        mask_watershed =cv2.bitwise_or(visited,mask_watershed)
        
        
def preprocess(img, percentiles= (10,80)):
    imgsmall = cv2.resize(img,(624,416))
    #imgsmooth = cv2.UMat(imgsmall)
    #
    pmin, pmax = np.percentile(imgsmall, percentiles)
    # Contrast stretching, we keep the image intensity between 2% and 90%
    imgbright = exposure.rescale_intensity(imgsmall, in_range=(pmin, pmax))
    #imgsmooth = cv2.GaussianBlur(imgbright[:,:,1:],(11,11),0)
    #imgrecomp = imgbright
    #imgrecomp[:,:,1:]= imgsmooth
    imgHSV = cv2.cvtColor(imgbright,cv2.COLOR_BGR2HSV)#.get().astype(np.uint8)
    #imgHSV[:,:,1] = cv2.equalizeHist(imgHSV[:,:,1])
    #imgHSV[:,:,2] = cv2.equalizeHist(imgHSV[:,:,2])
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

def createMap(img,R_ROBOT = 50,camera= "XT3"):
    border_size =30
    filter_poly = colorfilter("BLACK",camera)

    maskpoly = filter_poly.get_mask(img)
    # cv2.imshow("mask poly", maskpoly)
    margin = cv2.dilate(maskpoly,cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))\
                        ,iterations = 20,borderType =cv2.BORDER_CONSTANT,borderValue = 0)
    margin = cv2.dilate(margin,cv2.getStructuringElement(cv2.MORPH_RECT, (border_size,border_size))
                        ,iterations = 1,borderType = cv2.BORDER_CONSTANT,borderValue = 0)

    polyprojbin = margin.get().astype(np.uint8)
    contours, ret = cv2.findContours(polyprojbin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    polygons = []
    for c in contours:
        polygon    =  cv2.approxPolyDP(c, 15, True)
        polygons.append(polygon)
    # if len(polygons) == 0:
    #     polygons = False
    return polygons

def getCentroid(imageBin, setmanually = False):
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


def getRobotPos(imageBin, verbose = 0,display = 1):
    if type(imageBin) is cv2.UMat:
        imageBin = imageBin.get().astype(np.uint8)
    if display:
        cv2.imshow("mask",imageBin*255)
    valid = True
    moments = measure.moments(imageBin, order = 3)
    if moments[0,0]>800:
        centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
        varx = moments[0,2]/moments[0,0]-centroid[0]**2
        vary = moments[2,0]/moments[0,0]-centroid[1]**2
        varxy = moments[1,1]/moments[0,0]-centroid[0]*centroid[1]
        
        #check the variance of the image
        if max(varx,vary)>2*imageBin.size**0.5:
            if verbose:
                print("invalide coord:noise")
            return False,False
        #get the angle
        if abs(varx-vary)<0.0001:
            phi = math.pi/4
        else:
            phi = math.atan(2*varxy/(varx-vary))/2 +(varx<vary)*math.pi/2
        if verbose:
            print("phi(without correction): "+str(phi))
        #check direction
        imgsegmented = imageBin[int(centroid[1]-50):int(centroid[1]+50)\
                                ,int(centroid[0]-50):int(centroid[0]+50)]
        
        # print(imgsegmented.shape)
        #cv2.imshow("seg",imgsegmented*255)
        # print(phi)
        imgrot = ndimage.rotate(imgsegmented,phi*180/math.pi, reshape = False)

        newmoments = measure.moments(imgrot)
        cm03 = newmoments[0,3] \
               -3*newmoments[0,2]*newmoments[0,1]/newmoments[0,0]\
               +2*newmoments[0,1]**3/newmoments[0,0]**2
        if cm03 < 0:
            phi+=math.pi
        
        phi = (phi+math.pi)%(2*math.pi)-math.pi
        if verbose:
            print("phi (with correction):"+str(phi))
        pos = np.append(centroid,phi)
        pos[0:2] = pos[0:2]/5
    else:
        if verbose:
            print("invalide coord: no or not enough pixel")
        valid = False
        pos = False
        
    


    return pos,valid
    
    