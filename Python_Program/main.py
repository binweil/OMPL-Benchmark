# This example illustrates how to use the path/motion
# planning functionality from a remote API client.
#
# Load the demo scene 'motionPlanningServerDemo.ttt' in V-REP 
# then run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import vrep
import numpy as np
import time
import sqlite3
import uuid

def listToStringWithoutBrackets(list1):
    return str(list1).replace('[','').replace(']','')


# Initialize some variables
start = {}
goal = {}
config = {}
Map = {}
base_name = "/home/lamy/Desktop/OMPL_Compare_Task/VREP_Test_Maps/"
path_file = "/home/lamy/Desktop/OMPL_Compare_Task/Path_files/"
db_file = base_name + 'scenarios.db'

# Connect to Database and retrieve Data
connect = sqlite3.connect(db_file)
cursor = connect.cursor()
cursor.execute('SELECT scenario_id FROM scenarios')
tmp = cursor.fetchall()
scenario_ids = list(map(lambda x: x[0], tmp))
for i in scenario_ids:
    cursor.execute('SELECT start_state FROM scenarios WHERE scenario_id = '+str(i))
    startdb = cursor.fetchall()
    tmp = startdb[0][0]
    tmp = tmp.replace("[","").replace("]","")
    start[i] = list(np.fromstring(tmp,dtype='float',sep=' '))

    cursor.execute('SELECT goal_state FROM scenarios WHERE scenario_id = '+str(i))
    goaldb = cursor.fetchall()
    tmp = goaldb[0][0]
    tmp = tmp.replace("[","").replace("]","")
    goal[i] = list(np.fromstring(tmp,dtype='float',sep=' '))

    cursor.execute('SELECT map_name FROM scenarios WHERE scenario_id = '+str(i))
    VrepMap = cursor.fetchall()
    VrepMap = "{}{}.stl".format(base_name, VrepMap[0][0])
    Map[i] = VrepMap
connect.close()

# Connect to V-rep
print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, -500000, 5)  # Connect to V-REP, set a very large time-out for blocking commands
if clientID != -1:
    print('Connected to remote API server')

    emptyBuff = bytearray()

    # Start the simulation:
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    # Load a robot instance: res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/v_rep/qrelease/release/test.ttm'],emptyBuff,vrep.simx_opmode_oneshot_wait)
    #    robotHandle=retInts[0]
    
    # Retrieve some handles:
    res, robotHandle = vrep.simxGetObjectHandle(clientID,'UR5#',vrep.simx_opmode_oneshot_wait)
    res, target1 = vrep.simxGetObjectHandle(clientID,'testPose1#',vrep.simx_opmode_oneshot_wait)

    # Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
    res,retInts,target1Pose, retStrings,retBuffer=vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'getObjectPose', [target1], [], [], emptyBuff, vrep.simx_opmode_oneshot_wait)

    res, retInts, robotInitialState, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript,'getRobotState',[robotHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    # Some parameters:
    approachVector = [0, 0, 0]  # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
    maxConfigsForDesiredPose = 100  # we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
    maxTrialsForConfigSearch = 300 # a parameter needed for finding appropriate goal states
    searchCount = 1  # how many times OMPL will run for a given task
    minConfigsForPathPlanningPath=400  # interpolation states for the OMPL path
    minConfigsForIkPath = 100  # interpolation states for the linear approach path
    collisionChecking = 1  # whether collision checking is on or off
    algorithm = 30019  # RRT=30018 RRTConnect=30019 SBL=30021
    if algorithm == 30018:
        algorithm_name = "RRT"
    elif algorithm == 30019:
        algorithm_name = "RRTConnect"
    elif algorithm == 30021:
        algorithm_name = "SBL"
    else:
        algorithm_name = "unknown"

    inInts=[robotHandle,collisionChecking,minConfigsForPathPlanningPath,searchCount,algorithm]
    res, retInts, robotCurrentConfig, retStrings ,retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'getRobotState',[robotHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    for i in scenario_ids:
        map_length = len(Map[i])
        res, retInts, path, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer',
                                                                                vrep.sim_scripttype_childscript,
                                                                                'UpdateMap', [map_length],
                                                                                [], Map[i], emptyBuff,
                                                                                vrep.simx_opmode_oneshot_wait)
        config[i] = start[i] + goal[i]
        print("Start Configuration is ",start[i])
        print("Goal Configuration is ",goal[i])
        start_time = time.time()
        res,retInts,path,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'findPath_goalIsState',inInts,config[i],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        Calculation_time = time.time() - start_time
        print("Calculation time is", Calculation_time)
        print("Path array is ",path)
        print("")

        # Create path files
        n = 1
        file_id = uuid.uuid4()
        f = open(path_file + str(file_id) + '.txt', 'a')
        for m in range(0, len(path), 1):
            if n >= 7:
                n = 1
                f.write('\n')
                output = listToStringWithoutBrackets(path[m] * 180 / 3.14159)
                f.write(output + ',')
            else:
                output = listToStringWithoutBrackets(path[m] * 180 / 3.14159)
                f.write(output + ',')
            n = n + 1
        f.close()

        # Write Data to Database
        connect = sqlite3.connect(db_file)
        cursor = connect.cursor()
        cursor.execute('INSERT INTO Lamy_results (File_ID,Calculation_Time,Algorithm_Name,File_name) VALUES (?,?,?,?)', (str(file_id),Calculation_time, algorithm_name, "{}.txt".format(str(file_id))))
        connect.commit()
        connect.close()

        if (res == 0) and len(path) > 0:
            # Visualize the path:
            res, retInts, retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'visualizePath',[robotHandle,255,0,255],path,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
            lineHandle = retInts[0]
            

            # Make the robot follow the path:
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'runThroughPath',[robotHandle],path,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
            
            # Wait until the end of the movement:
            runningPath=True
            while runningPath:
                res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'isRunningThroughPath',[robotHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
                runningPath=retInts[0]==1
                
            # Clear the path visualization:
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'removeLine',[lineHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)

    # Stop simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
