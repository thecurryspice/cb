import numpy as np
import cv2
import cv2.aruco as aruco
import time
#from arucolib import *
import os
import sys
import vrep
import math
from time import sleep

#######ARUCO DETECTION####################################

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
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray=img
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

    Finds use in Calculate_orientation_in_degrees
    '''
    for i in range(len(ids)):
        Detected_ArUco_markers.update({ids[i][0]:corners[i][0]})

    #print Detected_ArUco_markers
    return Detected_ArUco_markers

def Calculate_orientation_in_radians(Detected_ArUco_markers):
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
        # print dx, dy

        '''
        angle = int(math.degrees(math.atan2(dy,dx)))
        if(topMidy > ycentre):
            angle += 360
        '''

        angle = (math.atan2(dy,dx))

        ArUco_marker_angles.update({marker:angle})
        # Write angle at far most left edge
        # Mark everything else

    #print ArUco_marker_angles
    return ArUco_marker_angles    ## returning the angles of the ArUco markers in degrees as a dictionary

def getArcuoAngleRadians(ArUco_marker_angles, ArUco_id_no):
    radians = ArUco_marker_angles.get(ArUco_id_no)
    degrees = int(math.degrees(math.atan2(dy,dx)))
        if(degrees < 0):
            degrees += 360
    return degrees

def getArcuoAngleDegrees(ArUco_marker_angles, ArUco_id_no):
    radians = ArUco_marker_angles.get(ArUco_id_no)
    return radians

def Find_coordinates(Detected_ArUco_markers, ArUco_id_no):

    dam = Detected_ArUco_markers
    ArUco_marker_angles = {}

    xcentre = int(dam.get(ArUco_id_no)[0][0] + dam.get(ArUco_id_no)[1][0] + dam.get(ArUco_id_no)[2][0] + dam.get(ArUco_id_no)[3][0])/4
    ycentre = int(dam.get(ArUco_id_no)[0][1] + dam.get(ArUco_id_no)[1][1] + dam.get(ArUco_id_no)[2][1] + dam.get(ArUco_id_no)[3][1])/4

    return xcentre, ycentre


#######ARUCO DETECTION####################################

######## V-Rep functions #############################################
emptyBuff = bytearray()
# Write a function here to choose a goal.

def chooseNextGoal(currentGoal):
    global goal_handles
    print (str(len(goal_handles)) + " in list")
    distance = sys.maxint
    for goal in goal_handles:
        returnCode, coordinates = vrep.simxGetObjectPosition(clientID,goal,currentGoal,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.1)
        print ("Processing Coordinates: "+str(coordinates))
        dist = math.sqrt(coordinates[0]**2 + coordinates[1]**2)
        if dist < distance:
            newGoal = goal
    goal_handles.remove(newGoal)
    print ("newGoal: "+ str(newGoal))
    return newGoal


# Write a function(s) to set/reset goal and other so that you can iterate the process of path planning

def setStartGoal(robot, prevGoal):
    global goal_dummy_handle,start_dummy_handle

    newStart = robot
    # get reference from prevStart
    returnCode, robotPos = vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_oneshot_wait)
    returnCode, newGoalPos = vrep.simxGetObjectPosition(clientID,newGoalPos,-1,vrep.simx_opmode_oneshot_wait)

    # shift Start
    returnCode = vrep.simxSetObjectPosition(clientID,start_dummy_handle,-1,robotPos,vrep.simx_opmode_oneshot_wait)
    # choose next best goal
    newGoal = chooseNextGoal(prevGoal)
    # get reference from newGoal
    returnCode, ngwrtpg = vrep.simxGetObjectPosition(clientID,newGoal,-1,vrep.simx_opmode_oneshot_wait)
    # shift Goal
    returnCode = vrep.simxSetObjectPosition(clientID,goal_dummy_handle,-1,vrep.simx_opmode_oneshot_wait)

    returnCode, newGoalPos = vrep.simxGetObjectPosition(clientID,newGoal,-1,vrep.simx_opmode_oneshot_wait)

    print("Processing Coordinates:")
    print("Previous Start: "+str(prevStartPos)+"\nPrevious Goal: "+str(prevGoalPos)+"\nRobot: "+str(robotPos)+"\nNext Goal: "+str(newGoalPos))
    # modified values
    print("Approaching goal "+str(newGoalPos))
    return prevStart, prevGoal


def forceGoal(robot,newGoal):
    global goal_dummy_handle,start_dummy_handle

    # get reference from robot
    returnCode, robotPos = vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_oneshot_wait)
    # shift Start
    returnCode = vrep.simxSetObjectPosition(clientID,start_dummy_handle,-1,robotPos,vrep.simx_opmode_oneshot_wait)
    # get reference from newGoal
    returnCode, newGoalPos = vrep.simxGetObjectPosition(clientID,newGoal,-1,vrep.simx_opmode_oneshot_wait)
    # shift Goal
    returnCode = vrep.simxSetObjectPosition(clientID,goal_dummy_handle,-1,newGoalPos,vrep.simx_opmode_oneshot_wait)

# Write a function to create a path from Start to Goal

def createPath():
    retCode, pathParameters, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'newPath', [], [], [], emptyBuff, vrep.simx_opmode_oneshot_wait)

# Write a function to make the robot move in the generated path. 
# Make sure that you give target velocities to the motors here in python script rather than giving in lua.
# Note that the your algorithm should also solve the conditions where partial paths are generated.

def travel(goal):
    while True:
        retCode, retInts, distance, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'travel', {goal}, [], [], emptyBuff, vrep.simx_opmode_blocking)
        #print distance
        if distance[0] < 0.02:
            print (str(distance[0])+" from current goal")
            break
    # stop the bot, position must not change in between setting references
    returnCode=vrep.simxSetJointTargetVelocity(clientID,rightjoint_handle,0,vrep.simx_opmode_oneshot_wait)
    returnCode=vrep.simxSetJointTargetVelocity(clientID,leftjoint_handle,0,vrep.simx_opmode_oneshot_wait)
    time.sleep(0.1)

######## V-Rep functions #############################################


######## openCV functions ############################################
def createROI():
    # global WALength, WABreadth, roiul, roiur, roibl, roibr, xscale, yscale
    WALength, WABreadth = 4, 3

    xscale = 0
    yscale = 0

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    print "Processing Video for ROI..."
    while ((not xscale) and (not yscale)):
        try:
            ret, img = cap.read()
            cv2.namedWindow('frame',0)
            cv2.imshow('frame',img)
            cv2.resizeWindow('frame',500,300)
            Detected_ArUco_markers = detect_ArUco(img)
            ArUco_marker_angles = Calculate_orientation_in_radians(Detected_ArUco_markers)
            # print Detected_ArUco_markers
            # Upper Left coordinates
            ULxcentre, ULycentre = Find_coordinates(Detected_ArUco_markers, 10)
            # Bottom Right coordinates
            BRxcentre, BRycentre = Find_coordinates(Detected_ArUco_markers, 11)

            # construct ROI
            roiul = [ULxcentre, ULycentre]
            roiur = [BRxcentre, ULycentre]
            roibl = [ULxcentre, BRycentre]
            roibr = [BRxcentre, BRycentre]

            # get scaling constants
            xscale = float(WALength)/(BRxcentre - ULxcentre)
            yscale = float(WABreadth)/(BRycentre - ULycentre)
            print xscale, yscale
            
            if cv2.waitKey(1) & 0xFF == ord('Q'):
                break
        except:
            print ("Bad Frame : Processing ROI")
            if cv2.waitKey(1) & 0xFF == ord('Q'):
                break
    cap.release()
    return xscale, yscale, ULxcentre, ULycentre
######## openCV functions ############################################


def getDistance(Detected_ArUco_markers, id1, id2):
    x1, y1 = Find_coordinates(ArUco_marker_angles, id1)
    x2, y2 = Find_coordinates(ArUco_marker_angles, id2)
    dist = math.sqrt(math.pow(x2-x1,2)+math.pow(y2-y1,2))
    return dist

# INITIALISATION
'''
vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
    print "connected to remote api server"
else:
    print 'connection not successful'
    sys.exit("could not connect")

returnCode = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

returnCode,cb_handle=vrep.simxGetObjectHandle(clientID,'cb',vrep.simx_opmode_oneshot_wait)
returnCode,leftjoint_handle=vrep.simxGetObjectHandle(clientID,'cb_left_joint',vrep.simx_opmode_oneshot_wait)
returnCode,rightjoint_handle=vrep.simxGetObjectHandle(clientID,'cb_right_joint',vrep.simx_opmode_oneshot_wait)
returnCode,start_dummy_handle = vrep.simxGetObjectHandle(clientID,'Start',vrep.simx_opmode_oneshot_wait)
returnCode,goal_dummy_handle = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_oneshot_wait)
returnCode,script_dummy_handle = vrep.simxGetObjectHandle(clientID,'LuaFunctions',vrep.simx_opmode_oneshot_wait)

returnCode,cylinder_handle1=vrep.simxGetObjectHandle(clientID,'Cylinder1',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle2=vrep.simxGetObjectHandle(clientID,'Cylinder2',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle3=vrep.simxGetObjectHandle(clientID,'Cylinder3',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle4=vrep.simxGetObjectHandle(clientID,'Cylinder4',vrep.simx_opmode_oneshot_wait )


#####################################################################################################################

# Write your code here


returnCode,disc_handle1=vrep.simxGetObjectHandle(clientID,'Disc1',vrep.simx_opmode_oneshot_wait )
returnCode,disc_handle2=vrep.simxGetObjectHandle(clientID,'Disc2',vrep.simx_opmode_oneshot_wait )
returnCode,disc_handle3=vrep.simxGetObjectHandle(clientID,'Disc3',vrep.simx_opmode_oneshot_wait )
returnCode,disc_handle4=vrep.simxGetObjectHandle(clientID,'Disc4',vrep.simx_opmode_oneshot_wait )
returnCode,disc_handle5=vrep.simxGetObjectHandle(clientID,'Disc5',vrep.simx_opmode_oneshot_wait )
'''
#returnCode,dummy_goal=vrep.simxGetObjectHandle(clientID,'DummyGoal',vrep.simx_opmode_oneshot_wait )

# fruit detection

# Stop
#vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)