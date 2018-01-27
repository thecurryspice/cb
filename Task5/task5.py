from task3_2.py import *
import pyserial

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

# Gives out scaled value
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
        cbAngle = ArUco_marker_angles.get(1)
        print ("Recognised CB State: (x,y): (" + str(cbxcentre) + "," + str(cbycentre) + "); " + str(cbAngle) + " degrees")
        # calculate scaled value
        # shift of origin is used wrt coordinates in V-Rep emulation
        cbScaledParams = [((cbxcentre - ULxcentre)*xscale) - 2, -(((cbycentre - ULycentre)*yscale) - 1.5), cbAngle]
        return cbScaledParams
    except:
        print "Bad Frame"
        cbState()

def moveCB(goalX, goalY):
    
    # find first reading
    cbScaledParams = cbState()
    cbPosX, cbPosY, cbAngle = cbScaledParams

    dy = goalY - cbPosY
    dx = goalX - cbPosX
    # print dx, dy
    target = int(math.degrees(math.atan2(dy,dx)))
    if(cbPosY > goalY):
        target += 360

    right()
    while(target - cbAngle > 5):
        cbPosX, cbPosY, cbAngle = cbState()
        # send scaled coordinates
        #retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformCB', [], cbScaledParams, [], emptyBuff, vrep.simx_opmode_blocking)
    
    stop()
    cbPosX, cbPosY, cbAngle = cbState()
    forward()
    while(goalX - cbPosX > 0.02):
        pass
    stop()


# serial console initialisation
serCon = serial.Serial(PORT, BAUDRATE)

xscale, yscale = createROI()

cap = cv2.VideoCapture(1)
print "Processing Live Feed..."
while (True):
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
        fruit2Pos = Find_coordinates(Detected_ArUco_markers, 3)

        # set goal at fruit 1
        moveCB(fruit1Pos[0], fruit1Pos[2])
        moveCB(fruit2Pos[0], fruit2Pos[2])
        # calculate scaled value
        # shift of origin is used wrt coordinates in V-Rep emulation
        cbScaledParams = [((cbxcentre - ULxcentre)*xscale) - 2, -(((cbycentre - ULycentre)*yscale) - 1.5), cbAngle]
        # print truckScaledParams

        # send scaled coordinates
        #retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformCB', [], cbScaledParams, [], emptyBuff, vrep.simx_opmode_blocking)
    except:
        print "Bad Frame"
cap.release()