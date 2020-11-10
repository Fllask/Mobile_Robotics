import sys
import os  # methods to work with file/directory paths
import getopt #get arguments
import cv2 #read video and images
import numpy as np
import time #wait, sleep
import skimage.io as io
import math

from object_recognition import show_image_test, object_recognition
from network import classification, reshape
from handmade import symbol_recongnition
from tracking import get_center_robot, get_centers_features, feature_hovered

import warnings
warnings.filterwarnings("ignore")




#code: [+,-,*,%,=] ---> [0,1,2,3,4]

indextoop = ['+','-','*','/','=']


#Get the arguments --input and --output
def get_arg(argv):
    input_path = ''
    output_path = ''
    if argv == []: #so I can run the debugger (I didn't find how to put argument when launching it)
        input_path = "robot_parcours_1.avi"
        output_path = "output.avi"
    try:
        options, args = getopt.getopt(argv, "i:o:", ["input=", "output="])
    except getopt.GetoptError as err:
        print(err)
        sys.exit(2)

    for name, value in options:
        if name == '--input':
            input_path = value
        if name in ('--output'):
            output_path = value

    print("Input path:", input_path)
    print("Output path:", output_path)
    return input_path, output_path



if __name__ == "__main__":

    #get the input and output path
    input_path, output_path = get_arg(sys.argv[1:])

    #open video CAREFUL: PATH WTH / NOT \
    video = cv2.VideoCapture(input_path)
    data_path = os.path.join(os.pardir, 'data')

    if(video.isOpened() == False):
        print("Error opening video")

    #get first frame for object recognition
    video.set(1, 0)
    ret, frame = video.read()

    #1st frame traitement, return the cropped feature
    cropped_features, corners = object_recognition(frame)
    centers_features = get_centers_features(corners)



    #classification of the images
    digits = np.empty(len(cropped_features))
    operator = np.empty(len(cropped_features))

    for k in range(len(digits)):
        digits[k] = classification(cropped_features[k])
        operator[k] = symbol_recongnition(cropped_features[k],verbose = 0)


    #initialize the loop
    frame_rgb = np.copy(frame)
    frame_rgb[:,:,0]=frame[:,:,2]
    frame_rgb[:,:,2]=frame[:,:,0]
    traj = np.array(get_center_robot(frame_rgb),ndmin = 2)
    features_used = []
    equation = 'x='

    #create the output video
    fourcc	= cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_path, fourcc, 2, (frame.shape[1],frame.shape[0]))

    #display video
    while(ret):
        #convert the frame to RGB
        frame_rgb = np.copy(frame)

        frame_rgb[:,:,0]=frame[:,:,2]
        frame_rgb[:,:,2]=frame[:,:,0]

        #get the center of the robot
        cr = get_center_robot(frame_rgb)

        #check the center was found
        if not math.isnan(cr[0]):
            #draw the center
            cv2.circle(frame, tuple(cr), 15, (255,0,0), thickness=3, lineType=8, shift=0)

            #build the trajectory
            traj = np.append(traj,np.array(cr,ndmin = 2),axis = 0)

            #check if the robot is over a feature
            feature = feature_hovered(centers_features,cr)
            if feature != -1 and (len(features_used)==0 or \
               feature != features_used[-1]):

                #add the feature to the list of used ones
                features_used.append(feature)

                #use the order to either use the operator or number classifier
                if len(features_used)%2==1:
                    equation = equation+ ("%1.0f" %(digits[feature]))
                    print(digits[feature])
                else:
                    op = indextoop[int(operator[feature])]
                    print(op)
                    #check is =; if so print the equation and result
                    if op != '=':
                        equation = equation+op
                    else:
                        x=0 #the compiler doesn't like the undefined x
                        exec(equation)
                        equation = equation+"="+str(x)
                        print(equation)



        #plot the centers of the features and the trajectory
        for i,center in enumerate(centers_features):
            if i in features_used:
                cv2.circle(frame, tuple(center), 15, (0,255,0), thickness=3, lineType=8, shift=0)
            else:
                cv2.circle(frame, tuple(center), 15, (0,0,255), thickness=3, lineType=8, shift=0)
        for i in range(len(traj)-1):
            cv2.line(frame, tuple(traj[i]), tuple(traj[i+1]), (255,0,0), thickness=3)

        #Print equation on video
        x_text = int(frame.shape[1] / 4)
        y_text = int(frame.shape[0] - frame.shape[0]/6)
        cv2.putText(frame, equation, (x_text, y_text), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,0), 1)



        #show_image_test(frame)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1) #wait 1 ms

        #write the output video
        out.write(frame)

        ret, frame = video.read() #grab next frame and read it, ret = false if end of Video

    #close video
    video.release()
    out.release()
    cv2.destroyAllWindows()
