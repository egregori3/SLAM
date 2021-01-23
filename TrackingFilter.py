import math
import random
import LidarBotModel as model
import algorithm1 as alg

#==============================================================================
# Tracking Filter
#==============================================================================
def particleFilter(state, heading, lidar_data, dtext):
    prediction = (0,0,0)
    dtext = list()

    if state == 'init':
        # Place initial particles
        pf = alg.ParticleFilter(parameters, robot)
        dtext.append("Place particles")
        state = 'updateSimulation'
        (done, nump, avgw, keep, prediction) = pf.pfData()




    # Wait here until the robot sends data
    heading, lidar_data = rw.getRobotData(parameters)
    if state == 'addRobot':
        print("Placing robot")
        robot = rw.PlaceRobot(parameters)
        dtext.append("Placing robot")
        state = 'moveRobot'
        current_object = robot
    elif state == 'moveRobot':
        print("Move robot")
        if robot.valid_weight() == False:
            robot.move()
            dtext.append("Move robot to valid weight region")
        else:
            state = 'placeParticles'
        current_object = robot
    elif state == 'placeParticles':
        print("Place particles")
        current_object = pf
    elif state == 'updateSimulation':
        rw.updateRobot()
        print("Iteration %d: robot(%d, %d)"%(iteration,robot.x,robot.y), end=" ")
        (done, nump, avgw, keep, prediction) = pf.update()
        current_object = pf
    else:
        print("!ERROR! Illegal state")
        break

    if type(current_obect) == alg.ParticleFilter:
        fields = ["p=%d,%d"%(nump,keep), "wavg=%f"%avgw]
        if prediction[2] > 0:
            fields.append("L=%d,%d"%(prediction[0],prediction[1]))
        for field in fields:
            print(field, end=" ")
            dtext.append(field)

