import pybullet as p
import pybullet_data
import time
import math
import cv2
import numpy
p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.03]

car = p.loadURDF("AUTO.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
	print(p.getJointInfo(car, joint))
targetVel = 20  #rad/s
targetVelRev = -1
maxForce = 30 #Newton
#p.applyExternalForce(car,3,[100,0,0],)
targetVel1 = 4
targetVel2 = -4
targetVelS = 0


width = 512                               #Setting parameters for the camera image
height = 512

fov = 60
aspect = width / height
near = 0.02                                     #Near plane
far = 5                                         #Far plane

for joint in [0,1]:
	p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =32,force = maxForce)
p.setJointMotorControl2(car, 7, p.VELOCITY_CONTROL, targetVelocity =100,force = maxForce)


while(1):
	for i in range(1000):
		p.setJointMotorControl2(car, 3, p.POSITION_CONTROL, targetPosition =0.5,force = maxForce)
		p.stepSimulation()
		time.sleep(1./240.)
		print(p.getJointState(car, 3))
'''
	for i in range(500):
		p.setJointMotorControl2(car, 3, p.POSITION_CONTROL, targetPosition =1,force = maxForce)
		p.stepSimulation()
		time.sleep(1./240.)
'''
		

