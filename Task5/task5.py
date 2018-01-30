from functions import *
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
        while(target - cbAngle > 5):
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

# serial console initialisation
ser = serial.Serial(PORT, BAUDRATE)
Fruit1Done = False
Fruit2Done = False

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
        #retCode, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript, 'transformCB', [], cbScaledParams, [], emptyBuff, vrep.simx_opmode_blocking)
    except:
        print "Bad Frame"
cap.release()