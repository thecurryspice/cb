-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

createPath=function(inInts,inFloats,inStrings,inBuffer)
    
    path_handle=simGetObjectHandle('Path')
    path_plan_handle=simGetPathPlanningHandle('PathPlanningTask')
    planstate=simSearchPath(path_plan_handle,2)
    path_pos=simGetObjectPositionOnPath(path_handle,pos_on_path)

    return {path_handle, planstate, path_handle},{},'',{}
end

getPathParameters = function(inInts,inFloats,inStrings,inBuffer)
    
    path_pos=simGetObjectPositionOnPath(path_handle,pos_on_path)
    return path_pos

end
    

if (sim_call_type==sim_childscriptcall_initialization) then

    -- Put some initialization code here

    motorLeft=simGetObjectHandle('left_joint')
    motorRight=simGetObjectHandle('right_joint')

    rebot_handle=simGetObjectHandle('CollectorBot')
    path_handle=simGetObjectHandle('Path')

    pos_on_path=0
    distance=0

    path_plan_handle=simGetPathPlanningHandle('PathPlanningTask')
    planstate=simSearchPath(path_plan_handle,2)

    start_dummy_handle=simGetObjectHandle('Start')


    --simExtRemoteApiStart(19997)
    robot_handle = simGetObjectHandle('CollectorBot')
    cylinder_handle1 = simGetObjectHandle('Cylinder1')
    cylinder_handle2 = simGetObjectHandle('Cylinder2')
    cylinder_handle3 = simGetObjectHandle('Cylinder3')
    cylinder_handle4 = simGetObjectHandle('Cylinder4')
    cylinder_handle5 = simGetObjectHandle('Cylinder5')
    cylinder_handle6 = simGetObjectHandle('Cylinder6')
    cylinder_handle7 = simGetObjectHandle('Cylinder7')
    cylinder_handle8 = simGetObjectHandle('Cylinder8')
    fruits = {cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5,cylinder_handle6,cylinder_handle7,cylinder_handle8}

    collection_handle=simGetCollectionHandle('Collection')

    --fruits = {cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5,cylinder_handle6,cylinder_handle7,cylinder_handle8}



    --simExtRemoteApiStart(19997)

    -- Make sure you read the section on "Accessing general-type objects programmatically"
    -- For instance, if you wish to retrieve the handle of a scene object, use following instruction:
    --
    -- handle=simGetObjectHandle('sceneObjectName')
    -- 
    -- Above instruction retrieves the handle of 'sceneObjectName' if this script's name has no '#' in it
    --
    -- If this script's name contains a '#' (e.g. 'someName#4'), then above instruction retrieves the handle of object 'sceneObjectName#4'
    -- This mechanism of handle retrieval is very convenient, since you don't need to adjust any code when a model is duplicated!
    -- So if the script's name (or rather the name of the object associated with this script) is:
    --
    -- 'someName', then the handle of 'sceneObjectName' is retrieved
    -- 'someName#0', then the handle of 'sceneObjectName#0' is retrieved
    -- 'someName#1', then the handle of 'sceneObjectName#1' is retrieved
    -- ...
    --
    -- If you always want to retrieve the same object's handle, no matter what, specify its full name, including a '#':
    --
    -- handle=simGetObjectHandle('sceneObjectName#') always retrieves the handle of object 'sceneObjectName' 
    -- handle=simGetObjectHandle('sceneObjectName#0') always retrieves the handle of object 'sceneObjectName#0' 
    -- handle=simGetObjectHandle('sceneObjectName#1') always retrieves the handle of object 'sceneObjectName#1'
    -- ...
    --
    -- Refer also to simGetCollisionhandle, simGetDistanceHandle, simGetIkGroupHandle, etc.
    --
    -- Following 2 instructions might also be useful: simGetNameSuffix and simSetNameSuffix

end


-- if (sim_call_type==sim_childscriptcall_actuation) then

--     -- Put your main ACTUATION code here

--     rob_pos=simGetObjectPosition(rebot_handle,-1)
--     path_pos=simGetObjectPositionOnPath(path_handle,pos_on_path)

--     m=simGetObjectMatrix(m)
--     m=simGetInvertedMatrix(m)
--     path_pos=simMultiplyVector(m,path_pos)
--     distance=math.sqrt(path_pos[1]^2+path_pos[2]^2)
--     phi=math.atan2(path_pos[2],path_pos[1])

--     if(pos_on_path<1) then
--         v_des=0.1
--         w_des=0.8*phi
--     else
--         v_des=0
--         w_des=0
--     end

--     wheel_separation=0.208
--     v_r=v_des+(wheel_separation/2)*w_des
--     v_l=v_des-(wheel_separation/2)*w_des

--     wheel_diameter=0.0701
--     wheel_radius=wheel_diameter/2

--     w_r=v_r/wheel_radius
--     w_l=v_l/wheel_radius

--     simSetJointTargetVelocity(motorLeft,w_l)
--     simSetJointTargetVelocity(motorRight,w_r)

--     if(distance<0.1) then
--         pos_on_path=pos_on_path+0.01
--     end

--     -- For example:
--     --
--     -- local position=simGetObjectPosition(handle,-1)
--     -- position[1]=position[1]+0.001
--     -- simSetObjectPosition(handle,-1,position)

-- end