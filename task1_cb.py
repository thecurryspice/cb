## Task 1.2 - Path Planning in V-REP ##
# Import modules

import sys
import vrep
import math
import time


# Write a function here to choose a goal.

def chooseNextGoal(currentGoal):
    global goal_handles
    distance = sys.maxint
    for goal in goal_handles:
        returnCode, coordinates = vrep.simxGetObjectPosition(clientID,goal,currentGoal,vrep.simx_opmode_oneshot_wait)
        print coordinates
        dist = math.sqrt(coordinates[0]**2 + coordinates[1]**2)
        if dist < distance:
            newGoal = goal
    goal_handles.remove(newGoal)
    print newGoal
    return newGoal


# Write a function(s) to set/reset goal and other so that you can iterate the process of path planning


def setNextGoal(goal):
    newGoal = chooseNextGoal(goal)
    returnCode, newPos = vrep.simxGetObjectPosition(clientID,newGoal,goal,vrep.simx_opmode_oneshot_wait)
    print ("newPos: "+str(newPos))
    retCode = vrep.simxSetObjectPosition(clientID,goal,goal,newPos,vrep.simx_opmode_oneshot_wait)
    return goal

def setNewStart(start, newStart):
    returnCode, newPos = vrep.simxGetObjectPosition(clientID,newStart,start,vrep.simx_opmode_oneshot_wait)
    retCode = vrep.simxSetObjectPosition(clientID,start,start,newPos,vrep.simx_opmode_oneshot_wait)
    return start    




# Write a function to create a path from Start to Goal

def createPath(start, goal):
    emptyBuff = bytearray()
    retCode, pathParameters, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'createPath', [start, goal], [], [], emptyBuff, vrep.simx_opmode_oneshot_wait)
    print ("Path Parameters: " + str(pathParameters))
    return pathParameters


# Write a function to make the robot move in the generated path. 
# Make sure that you give target velocities to the motors here in python script rather than giving in lua.
# Note that the your algorithm should also solve the conditions where partial paths are generated.

def move(path_handle, robot_handle, leftjoint_handle, rightjoint_handle):
    rob_pos=vrep.simxGetObjectPosition(rebot_handle,-1)
    path_pos=simGetObjectPositionOnPath(path_handle,pos_on_path)

    m=simGetObjectMatrix(m)
    m=simGetInvertedMatrix(m)
    path_pos=simMultiplyVector(m,path_pos)
    distance=math.sqrt(path_pos[1]^2+path_pos[2]^2)
    phi=math.atan2(path_pos[2],path_pos[1])

    if(pos_on_path<1):
        v_des=0.1
        w_des=0.8*phi
    else:
        v_des=0
        w_des=0
        

    wheel_separation=0.208
    v_r=v_des+(wheel_separation/2)*w_des
    v_l=v_des-(wheel_separation/2)*w_des

    wheel_diameter=0.0701
    wheel_radius=wheel_diameter/2

    w_r=v_r/wheel_radius
    w_l=v_l/wheel_radius

    vrep.simSetJointTargetVelocity(leftjoint_handle,w_l)
    vrep.simSetJointTargetVelocity(rightjoint_handle,w_r)

    if(distance<0.1):
        pos_on_path=pos_on_path+0.01

    # For example:
    
    # local position=vrep.simxGetObjectPosition(handle,-1)
    # position[1]=position[1]+0.001
    # vrep.simxSetObjectPosition(handle,-1,position)


################ Initialization of handles. Do not change the following section ###################################

vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
    print "connected to remote api server"
else:
    print 'connection not successful'
    sys.exit("could not connect")

returnCode,robot_handle=vrep.simxGetObjectHandle(clientID,'CollectorBot',vrep.simx_opmode_oneshot_wait)
returnCode,leftjoint_handle=vrep.simxGetObjectHandle(clientID,'left_joint',vrep.simx_opmode_oneshot_wait)
returnCode,rightjoint_handle=vrep.simxGetObjectHandle(clientID,'right_joint',vrep.simx_opmode_oneshot_wait)
returnCode,start_dummy_handle = vrep.simxGetObjectHandle(clientID,'Start',vrep.simx_opmode_oneshot_wait)
returnCode,goal_dummy_handle = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_oneshot_wait)
returnCode,script_dummy_handle = vrep.simxGetObjectHandle(clientID,'LuaFunctions',vrep.simx_opmode_oneshot_wait)

returnCode,cylinder_handle1=vrep.simxGetObjectHandle(clientID,'Cylinder1',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle2=vrep.simxGetObjectHandle(clientID,'Cylinder2',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle3=vrep.simxGetObjectHandle(clientID,'Cylinder3',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle4=vrep.simxGetObjectHandle(clientID,'Cylinder4',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle5=vrep.simxGetObjectHandle(clientID,'Cylinder5',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle6=vrep.simxGetObjectHandle(clientID,'Cylinder6',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle7=vrep.simxGetObjectHandle(clientID,'Cylinder7',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle8=vrep.simxGetObjectHandle(clientID,'Cylinder8',vrep.simx_opmode_oneshot_wait )

cylinder_handles=[cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5,cylinder_handle6,cylinder_handle7,cylinder_handle8]

#goal_handles = [cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5]
#cylinder_obstacle_handles = [cylinder_handle6,cylinder_handle7,cylinder_handle8]

#####################################################################################################################

# Write your code here

returnCode = vrep.simxSetJointTargetVelocity(clientID, rightjoint_handle, 1, vrep.simx_opmode_streaming)
returnCode = vrep.simxSetJointTargetVelocity(clientID, leftjoint_handle, 1, vrep.simx_opmode_streaming)

time.sleep(1)
# start = start_dummy_handle
# goal = cylinder_handle2
# createPath(start, goal)

# start = goal
# goal = setNextGoal(goal)
# createPath(start,goal)


################     Do not change after this #####################

#end of simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)