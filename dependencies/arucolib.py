############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

#Detected_ArUco_markers = {}
#ArUco_marker_angles = {}

def detect_ArUco(img):
    ## function to detect ArUco markers in the image using ArUco library
    ## argument: img is the test image
    ## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    ##         for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    ##              {0: array([[315, 163],
    #                           [319, 263],
    #                           [219, 267],
    #                           [215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##
    
    # read image in gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # grab dictionary and create object
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()

    # detect markers
    corners, ids, _ =  aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    
    '''
    cv2.imshow('testx',aruco.drawDetectedMarkers(im, corners, ids))
    cv2.waitKey(0*1000)
    cv2.destroyAllWindows()
    '''
    
    # update dictionary
    '''
    Dictionary Stored Value Syntax:    [corner]   | [coordinate]
                                    <TL,TR,BR,BL> |    <0,1>

    Finds use in Calculate_orientatio_in_degrees
    '''
    for i in range(len(ids)):
        Detected_ArUco_markers.update({ids[i][0]:corners[i][0]})

    #print Detected_ArUco_markers
    return Detected_ArUco_markers

def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ## function to calculate orientation of ArUco with respective to the scale mentioned in Problem_Statement.pdf
    ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the ProblemStatement.pdf)
    ##          for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
    ##          function should return: {1: 120 , 2: 164}

    dam = Detected_ArUco_markers
    ArUco_marker_angles = {}
    ## enter your code here ##

    for marker in dam.keys():
        # calculate atan(dy/dx)
        xcentre = int(dam.get(marker)[0][0] + dam.get(marker)[1][0] + dam.get(marker)[2][0] + dam.get(marker)[3][0])/4
        ycentre = int(dam.get(marker)[0][1] + dam.get(marker)[1][1] + dam.get(marker)[2][1] + dam.get(marker)[3][1])/4
        topMidx = int(dam.get(marker)[0][0] + dam.get(marker)[1][0])/2
        topMidy = int(dam.get(marker)[0][1] + dam.get(marker)[1][1])/2
        
        dy = ycentre - topMidy
        dx = topMidx - xcentre
        print dx, dy

        angle = int(math.degrees(math.atan2(dy,dx)))
        if(topMidy > ycentre):
            angle += 360
        ArUco_marker_angles.update({marker:angle})
        # Write angle at far most left edge
        # Mark everything else

    #print ArUco_marker_angles
    return ArUco_marker_angles    ## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
    ## function to mark ArUco in the test image as per the instructions given in problem_statement.pdf 
    ## arguments: img is the test image 
    ##            Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    ##            ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    ## return: image namely img after marking the aruco as per the instruction given in Problem_statement.pdf

    ## enter your code here ##
    dam = Detected_ArUco_markers
    
    for marker in dam.keys():
        # calculate atan(dy/dx)
        xcentre = int(dam.get(marker)[0][0] + dam.get(marker)[1][0] + dam.get(marker)[2][0] + dam.get(marker)[3][0])/4
        ycentre = int(dam.get(marker)[0][1] + dam.get(marker)[1][1] + dam.get(marker)[2][1] + dam.get(marker)[3][1])/4
        
        topMidy = int(dam.get(marker)[0][1] + dam.get(marker)[1][1])/2
        topMidx = int(dam.get(marker)[0][0] + dam.get(marker)[1][0])/2
        dy = topMidy - ycentre
        dx = topMidx - xcentre
        
        #xmin = int(min(dam.get(marker)[0][0], dam.get(marker)[1][0], dam.get(marker)[2][0], dam.get(marker)[3][0]))
        #xmax = int(max(dam.get(marker)[0][0], dam.get(marker)[1][0], dam.get(marker)[2][0], dam.get(marker)[3][0]))
        edge = int(math.sqrt(math.pow(dy,2)+math.pow(dx,2)))
        # mark everything
        
        #id
        img = cv2.putText(img,str(marker),(xcentre + edge/2, ycentre),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

        # corners
        img = cv2.circle(img,(dam.get(marker)[0][0], dam.get(marker)[0][1]), 4, [125,125,125], -1)
        img = cv2.circle(img,(dam.get(marker)[1][0], dam.get(marker)[1][1]), 4, [000,255,000], -1)
        img = cv2.circle(img,(dam.get(marker)[2][0], dam.get(marker)[2][1]), 4, [180,105,255], -1)
        img = cv2.circle(img,(dam.get(marker)[3][0], dam.get(marker)[3][1]), 4, [255,255,255], -1)

        # line and centre
        img = cv2.line(img,(xcentre, ycentre),(topMidx, topMidy),(255,0,0),4)
        img = cv2.circle(img,(xcentre, ycentre), 4, [0,0,255], -1)
        
        # angle
        img = cv2.putText(img,str(ArUco_marker_angles.get(marker)),(xcentre-edge, ycentre),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2,cv2.LINE_AA)
        
        cv2.imshow('im', img)
        cv2.waitKey(0*1000)
        cv2.destroyAllWindows()

    return img