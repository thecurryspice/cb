from functions import *
import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
DELIMITER = ""

# functions for CB's motion
def forward():
    ser.write('qw'+DELIMITER)
def backward():
    ser.write('as'+DELIMITER)
def right():
    ser.write('qs'+DELIMITER)
def left():
    ser.write('aw'+DELIMITER)
def stop():
    ser.write('zx'+DELIMITER)

# Gives out CB state (absolute and scaled, both)
def cbState():
    global cap,img,Detected_ArUco_markers,ArUco_marker_angles, xscale, yscale
    try:
        ret, img = cap.read()
        cv2.namedWindow('frame',0)
        cv2.imshow('frame',img)
        cv2.resizeWindow('frame',500,300)
        
        if cv2.waitKey(1) & 0xFF == ord('Q'):
            return
        Detected_ArUco_markers = detect_ArUco(img)
        ArUco_marker_angles = Calculate_orientation_in_radians(Detected_ArUco_markers)

        # CB coordinates
        cbxcentre, cbycentre = Find_coordinates(Detected_ArUco_markers, 1)
        # get orientation of truck
        cbAngle = getArcuoAngleDegrees(ArUco_marker_angles,1)
        print ("Recognised CB State: (x,y): (" + str(cbxcentre) + "," + str(cbycentre) + "); " + str(cbAngle) + " degrees")
        # calculate scaled value
        # shift of origin is used wrt coordinates in V-Rep emulation
        cbScaledParams = [cbxcentre, cbycentre, ((cbxcentre - ULxcentre)*xscale) - 2, -(((cbycentre - ULycentre)*yscale) - 1.5), cbAngle]
        return cbScaledParams
    except:
        print "Bad Frame : Recognising CB State"
        sleep(0.25)
        cbState()

def moveCB(goalX, goalY):
    global Fruit1Done, Fruit2Done
    # take readings
    cbxcentre, cbycentre, cbPosX, cbPosY, cbAngle = cbState()

    if (not Fruit1Done):
        dy = goalY - cbycentre
        dx = goalX - cbxcentre
        # print dx, dy
        target = int(math.degrees(math.atan2(dy,dx)))
        if(target < 0):
            target += 360

        # check if properly oriented
        while(target - cbAngle > 2):
            print(str(target-cbAngle) + " degrees more to turn")
            cbxcentre, cbycentre, cbPosX, cbPosY, cbAngle = cbState()
            right()
        
        # check distance and move
        distance = getDistance(Detected_ArUco_markers, 2, 1)
        while(distance > 50):
            print(str(distance) + " units more to move")
            forward()
            distance = getDistance(Detected_ArUco_markers, 2, 1)

        stop()
        Fruit1Done = True
    
    if (Fruit1Done and (not Fruit2Done)):
        dy = goalY - cbycentre
        dx = goalX - cbxcentre
        # print dx, dy
        target = int(math.degrees(math.atan2(dy,dx)))
        if(target < 0):
            target += 360

        # check if properly oriented
        while(target - cbAngle > 5):
            print(str(target-cbAngle) + " degrees more to turn")
            cbxcentre, cbycentre, cbPosX, cbPosY, cbAngle = cbState()
            right()
        
        # check distance and move
        distance = getDistance(Detected_ArUco_markers, 3, 1)
        while(distance > 50):
            print(str(distance) + " units more to move")
            forward()
            distance = getDistance(Detected_ArUco_markers, 3, 1)
            
        stop()


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
returnCode,dummy_goal=vrep.simxGetObjectHandle(clientID,'DummyGoal',vrep.simx_opmode_oneshot_wait )

returnCode,cylinder_handle1=vrep.simxGetObjectHandle(clientID,'Cylinder1',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle2=vrep.simxGetObjectHandle(clientID,'Cylinder2',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle3=vrep.simxGetObjectHandle(clientID,'Cylinder3',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle4=vrep.simxGetObjectHandle(clientID,'Cylinder4',vrep.simx_opmode_oneshot_wait )

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


cap = cv2.VideoCapture(1)
if not cap.isOpened():
    pass
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
print "Capturing Markers"
try:
        ret, img = cap.read()
        cv2.namedWindow('frame',0)
        cv2.imshow('frame',img)
        cv2.resizeWindow('frame',500,300)
        
        if cv2.waitKey(1) & 0xFF == ord('Q'):
            break
        Detected_ArUco_markers = detect_ArUco(img)
except:
        print "Bad Frame"

for id in Detected_ArUco_markers:
    if id in [2,3,4,5]:
        print("Detected " + str(id))
        retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformFruit', [FFruit_handles.pop()], [2.0,2.0,0], [], emptyBuff, vrep.simx_opmode_blocking)
    if id in [6,7,8,9]:
        print("Detected " + str(id))
        retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformFruit', [DFruit_handles.pop()], [2.0,2.0,0], [], emptyBuff, vrep.simx_opmode_blocking)
cap.close()


# serial console initialisation
ser = serial.Serial(PORT, BAUDRATE)
Fruit1Done = False
Fruit2Done = False
Fruit3Done = False
Fruit4Done = False

xscale, yscale, ULxcentre, ULycentre = createROI()

cap = cv2.VideoCapture(1)
print "Processing Live Feed..."
while (not Fruit1Done or (not Fruit2Done)):
    try:
        ret, img = cap.read()
        cv2.namedWindow('frame',0)
        cv2.imshow('frame',img)
        cv2.resizeWindow('frame',500,300)
        
        if cv2.waitKey(1) & 0xFF == ord('Q'):
            break
        Detected_ArUco_markers = detect_ArUco(img)
        ArUco_marker_angles = Calculate_orientation_in_radians(Detected_ArUco_markers)

        # Fruits' coordinates
        fruit1Pos = Find_coordinates(Detected_ArUco_markers, 2)
        retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformFruit', [], scaledFruitLocation, [], emptyBuff, vrep.simx_opmode_blocking)
        fruit2Pos = Find_coordinates(Detected_ArUco_markers, 3)

        # set goal at fruit 1
        while (not Fruit1Done):
            try:
                moveCB(fruit1Pos[0], fruit1Pos[1])
            except:
                pass
        print "Fruit1 done"
        while (not Fruit2Done):
            try:
                moveCB(fruit2Pos[0], fruit2Pos[1])
            except:
                pass
        print "Fruit2 done"

        # send scaled coordinates
        retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformCB', [], cbScaledParams, [], emptyBuff, vrep.simx_opmode_blocking)
    except:
        print "Bad Frame"
cap.release()