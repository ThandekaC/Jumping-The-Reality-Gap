import numpy as np
import pybullet as p
import pybullet_data
import time
import pickle
import csv

# import legged data
# angles
file = open('final_angles_data_pyomo', 'rb')
pickled_angles = pickle.load(file)
file.close()
print(pickled_angles)

# angles
thf1_out = pickled_angles[0] #remember that in pyomo, the front legs are actually the hind legs
thf2_out = pickled_angles[1]
thh1_out = pickled_angles[2]
thh2_out = pickled_angles[3]

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(False)#need to take steps
planeId = p.loadURDF("plane.urdf")
robot = p.loadURDF("Cheetah.urdf",[0,0,0.5], [0,0,0,1])


num_joints =p.getNumJoints(robot)
#print(num_joints)

#num_joints = p.getNumJoints(robot) # Rather use total angles
#jointLimitForce
LF_JOINT2=[]
LF_JOINT3=[]
RF_JOINT2=[]
RF_JOINT3=[]
LB_JOINT2=[]
LB_JOINT3=[]
RB_JOINT2=[]
RB_JOINT3=[]


for j in range(30):

    start_time = time.perf_counter()

    #position
    p.setJointMotorControlArray(robot,[1,2,4,5,7,8,10,11],p.POSITION_CONTROL,targetPositions =[thh1_out[j], thh2_out[j], thh1_out[j], thh2_out[j], thf1_out[j],thf2_out[j],thf1_out[j],thf2_out[j]])
    #p.calculateInverseKinematics(robot,1,targetPosition=[0.10, 0, 0])
    LF_JOINT2.append(p.getJointStates(robot,[1])[0][0])
    LF_JOINT3.append(p.getJointStates(robot,[2])[0][0])
    RF_JOINT2.append(p.getJointStates(robot,[4])[0][0])
    RF_JOINT3.append(p.getJointStates(robot,[5])[0][0])
    LB_JOINT2.append(p.getJointStates(robot,[7])[0][0])
    LB_JOINT3.append(p.getJointStates(robot,[8])[0][0])
    RB_JOINT2.append(p.getJointStates(robot,[10])[0][0])
    RB_JOINT3.append(p.getJointStates(robot,[11])[0][0])
    #print()

    p.stepSimulation()

    end_time = time.perf_counter()
    time.sleep(0.1)
    #time.sleep(max(1/240 - end_time + start_time, 0))

new_angles = zip(LF_JOINT2,LF_JOINT3,RF_JOINT2,RF_JOINT3,LB_JOINT2,LB_JOINT3,RB_JOINT2,RB_JOINT3)  
header =['LF_JOINT2','LF_JOINT3','RF_JOINT2','RF_JOINT3','LB_JOINT2','LB_JOINT3','RB_JOINT2','RB_JOINT3']

with open('Mini_Cheetah_angle_tracking.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f, delimiter='\t')    
    writer.writerow(header)    
    writer.writerows(new_angles)
