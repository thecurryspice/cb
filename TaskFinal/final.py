import numpy as np
import cv2
import cv2.aruco as aruco
import time
#from arucolib import *
import os
import sys
import vrep
import math
import time

emptyBuff = bytearray()
vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
    print "connected to remote api server"
else:
    print 'connection not successful'
    sys.exit("could not connect")

returnCode = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

returnCode,FFruit_handle1=vrep.simxGetObjectHandle(clientID,'FFruit1',vrep.simx_opmode_oneshot_wait )
returnCode,FFruit_handle2=vrep.simxGetObjectHandle(clientID,'FFruit2',vrep.simx_opmode_oneshot_wait )
returnCode,FFruit_handle3=vrep.simxGetObjectHandle(clientID,'FFruit3',vrep.simx_opmode_oneshot_wait )
returnCode,FFruit_handle4=vrep.simxGetObjectHandle(clientID,'FFruit4',vrep.simx_opmode_oneshot_wait )
FFruit_handles=[FFruit_handle1,FFruit_handle2,FFruit_handle3,FFruit_handle4]

returnCode,DFruit_handle1=vrep.simxGetObjectHandle(clientID,'DFruit1',vrep.simx_opmode_oneshot_wait )
returnCode,DFruit_handle2=vrep.simxGetObjectHandle(clientID,'DFruit2',vrep.simx_opmode_oneshot_wait )
returnCode,DFruit_handle3=vrep.simxGetObjectHandle(clientID,'DFruit3',vrep.simx_opmode_oneshot_wait )
returnCode,DFruit_handle4=vrep.simxGetObjectHandle(clientID,'DFruit4',vrep.simx_opmode_oneshot_wait )
DFruit_handles=[DFruit_handle1,DFruit_handle2,DFruit_handle3,DFruit_handle4]

image_list = ["aruco3.jpg"]
img = cv2.imread(image)
Detected_ArUco_markers = detect_ArUco(img)
for id in Detected_ArUco_markers:
	if id in [2,3,4,5]:
		retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformFruit', [FFruit_handles.pop()], [2.0,2.0,0], [], emptyBuff, vrep.simx_opmode_blocking)
	if id in [6,7,8,9]:
		retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformFruit', [DFruit_handles.pop()], [2.0,2.0,0], [], emptyBuff, vrep.simx_opmode_blocking)
time.sleep(5)

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)