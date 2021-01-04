
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
carpos = [0, 0, 0.1]

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
p.setJointMotorControl2(car, 7, p.VELOCITY_CONTROL, targetVelocity =50,force = maxForce)


while (1):
    keys = p.getKeyboardEvents()
    print(p.getLinkState(car, 0))
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in [0,1]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
           
            p.stepSimulation()
            time.sleep(1./240.)
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in [0,1]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVelS,force = maxForce)
            
            p.stepSimulation()
            time.sleep(1./240.)          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in [0,1]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVelRev,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in [0,1]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVelS,force = maxForce)
            p.stepSimulation()
            time.sleep(1./240.)           
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in [2,4]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel2,force = maxForce)
            for joint in [3,5]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel1,force = maxForce)
            p.stepSimulation()
            time.sleep(1./240.)
            
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVelS,force = maxForce)
            
            p.stepSimulation()
            time.sleep(1./240.)
            
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in [2,4]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel1,force = maxForce)
            for joint in [3,5]:
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel2,force = maxForce)
            p.stepSimulation()
            time.sleep(1./240.)

            
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVelS,force = maxForce)
            
            p.stepSimulation()
            time.sleep(1./240.)



        if (k == ord('r') and (v & p.KEY_IS_DOWN)):
            p.applyExternalTorque(car, -1, [0,0,180], flags = p.LINK_FRAME)
            p.stepSimulation()
            time.sleep(1./240.)
        	
            
            
        if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            p.applyExternalTorque(car, -1, [0,0,0], flags = p.LINK_FRAME)
            p.stepSimulation()
            time.sleep(1./240.)
            
        if (k == ord('a') and (v & p.KEY_IS_DOWN)):
            targetVel = targetVel + 1
            targetVel1 = targetVel1 + 0.5
            targetVel2 = targetVel2 - 0.5
            targetVelRev = targetVelRev - 1
            while(1):
                br = 0
                keys = p.getKeyboardEvents()
                for k, v in keys.items():
                    if(k == ord('a') and (v & p.KEY_WAS_RELEASED)):
                        br = 1
                        break
                if(br == 1):
                    break
                


        if (k == ord('c') and (v & p.KEY_WAS_RELEASED)):
            pos = p.getBasePositionAndOrientation(car)                              #Getting the position and orientation of the car
            cord = pos[0]
            ori = p.getEulerFromQuaternion(pos[1])
            unit = [0.3*math.cos(ori[2]),0.3*math.sin(ori[2]),0.35]                #Calculating a unit vector in direction of the car. The anle depends only upon YAW(z) as car is in xy plane
            cam = [sum(x) for x in zip(cord,unit)]                                 #Fixing the camera a little forward from the center of the car
            unit2 = [1*math.cos(ori[2]),1*math.sin(ori[2]),0.20] 
            see = [sum(x) for x in zip(cord,unit2)]                                #Setting the looking direction in direction of the car
            view_matrix = p.computeViewMatrix(cam, see , [0,0,1])                  #Calculating the position of the camera
            pro_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)      #Getting the resolution matrix based on camera properties
            pre_image = p.getCameraImage(width, height, view_matrix, pro_matrix, shadow = True, renderer=p.ER_BULLET_HARDWARE_OPENGL)  #Getting the image from the camera
            image = cv2.cvtColor(pre_image[2], cv2.COLOR_RGB2BGR)             #Image returned is in RGB format but opencv works with BGR so converting the colo scheme
            cv2.imshow('Husky', image)                                        #Displaying the image
            cv2.waitKey(0)
            cv2.destroyAllWindows()              



p.getContactPoints(car)

p.disconnect()
