# Hexbug simulator using particle filter
# written by Eric Gregori
import getopt
import sys
import os
import math
import random
import helpers
import LidarBotModel as model
import algorithm1 as alg
import arena
import cv2
import numpy as np
import display
import DataLogger
from ast import literal_eval


#==============================================================================
#
# Init parameters
#
#==============================================================================
def Config(parameters):
    parameters['plotGraphics']                    = False
    parameters['plotSamples']                     = False
    parameters['verbose']                         = False
    parameters['simulations']                     = 1        # Number of simulations
    parameters['displayDynamicText']              = list()
    parameters['arenaRegion']                     = 10
    parameters['numberOfParticles']               = 0

    # Robot model parameters
    parameters['measurementSigmaNoise']           = 2.0      # Sigma for measurement noise
    parameters['headingSigmaNoise']               = 0.15     # Sigma for turning noise
    parameters['distanceSigmaNoise']              = 0.003    # Sigma for distance noise
    parameters['distanceDivider']                 = 10
    parameters['lidarSamples']                    = 36
    parameters['distanceIteraton']                = 30       # Move 10 pixels per iteration
    parameters['lidarMaxDistance']                = 50       # Max distance in pixels

def dumpConfig(parameters):
    print('Configuration')
    print(parameters)

#==============================================================================
#
# OutputData
#
#==============================================================================
def OutputData(parameters, index, pf):
    if parameters['plotGraphics']:
        filename = parameters['outputPath']+"/iteration" + str(index)
        if type(pf) == alg.ParticleFilter:
            wmax = pf.maxWeight()
        else:
            wmax = 1
        print("Output %s"%(filename))
        image = display.Display(parameters, pf, wmax, filename)
    if parameters['plotSamples']:
        filename = parameters['outputPath']+"/lidar" + str(index)
        parameters['dataLoggerObject'].plotSamples(pf.particles, filename)

#==============================================================================
#
# Place the robot into the arena
#
#==============================================================================
def PlaceRobot(parms):
    parms['robotPath'] = []
    robot = model.Particle(parms=parms)
    if 'robotX' not in parms:
        robot.place({})
    else:
        try:
            data = (parms['robotX'], parms['robotY'], parms['robotH'])
        except KeyError:
            print("!ERROR! You must specify y and h with x")
            usage()
        p = {'setPosition':data}
        robot.place(p)
    print("Robot initial state (x = %d, y = %d, h = %f"%(robot.x, robot.y, robot.heading))
    return robot

#==============================================================================
#
# main
#
#==============================================================================
def usage():
    print("!ERROR! Illegal parameter")
    print("Required: -a arenaFilename -i iterations ")
    print("Optional strings:  [-o outputPath] [-d dataFilename] [-m plot samples] ")
    print("Optional ints: [-x -y -h Initial robot position] [-w Initial weight threshold] [-p numberOfParticles]")
    print("Optional Flags: [-v verbose] [-g output graphics]")
    print("-o must be specified with -d")
    print("-o must be specified with -m")
    print("-o must be specified with -g")
    sys.exit(1)

def main(argv):
    parameters = dict()
    Config(parameters)
    try:
        opts, args = getopt.getopt(argv, "vgi:o:a:d:p:m:x:y:h:w:")
    except getopt.GetoptError:
        usage()
    for opt, arg in opts:
        if opt in ("-v"):  # -v verbose
            parameters['verbose']           = True
        elif opt in ("-o"):
            parameters['outputPath']        = arg
        elif opt in ("-a"):
            parameters['arenaFilename']     = arg
        elif opt in ("-i"):
            parameters['iterations']        = int(arg)      # Number of times the robot is moved
        elif opt in ("-d"):
            parameters['dataFilename']      = arg
        elif opt in ("-p"):
            parameters['numberOfParticles'] = int(arg)
        elif opt in ("-m"):
            parameters['plotSamples']       = True
        elif opt in ("-g"):
            parameters['plotGraphics']      = True
        elif opt in ("-x"):
            parameters['robotX']            = int(arg)
        elif opt in ("-y"):
            parameters['robotY']            = int(arg)
        elif opt in ("-h"):
            parameters['robotH']            = float(arg)
        elif opt in ("-w"):
            parameters['initialWeightThres']= float(arg)


    # Test command line parameters
    try:
        print("arenaFilename       = "+parameters['arenaFilename'])
        if 'outputPath' in parameters:
            print("outputPath          = "+parameters['outputPath'])
        print("iterations          = "+str(parameters['iterations']))
        if parameters['plotGraphics']: 
            print("Graphics output enabled")
        if parameters['plotSamples']: 
            print("Sample plotting output enabled")
    except KeyError:
        usage()
    print()
    print()
    if 'dataFilename' in parameters and not 'outputPath' in parameters:
        print("!ERROR! -o must be specified with -d")
        usage()
    if parameters['plotSamples'] and not 'outputPath' in parameters:
        print("!ERROR! -o must be specified with -m")
        usage()
    if parameters['plotGraphics'] and not 'outputPath' in parameters:
        print("!ERROR! -o must be specified with -g")
        usage()

    parameters['displayStaticText'] = ["p="+str(parameters['numberOfParticles'])]
    parameters['arena'] = arena.Arena(parameters)
    parameters['dataLoggerObject'] = DataLogger.DataLogger(parameters)
    if 'dataFilename' in parameters: 
        parameters['dataLoggerObject'].startDataLogger()

    print('Particle Filter Simulation')
    # Brute force simulation of multiple possible parameters
    for sim in range(1,parameters['simulations']+1):
        print('Simulation #', sim)

    state = 'addRobot'
    iteration = 0
    while(1):
        if state == 'addRobot':
            print("Placing robot")
            pf_or_robot = PlaceRobot(parameters)
            state = 'moveRobot'
        elif state == 'moveRobot':
            print("Move robot")
            if pf_or_robot.valid_weight() == False:
                pf_or_robot.move()
            else:
                state = 'placeParticles'
                robot = pf_or_robot
        elif state == 'placeParticles':
            print("Place particles")
            pf_or_robot = alg.ParticleFilter(parameters, robot)
            state = 'updateSimulation'
        elif state == 'updateSimulation':
            print("Iteration %d: robot(%d, %d)"%(iteration, int(robot.x), int(robot.y)), end=" ")
            result = pf_or_robot.update()
            wmax = pf_or_robot.maxWeight()
            wavg = pf_or_robot.avgWeight()
            predicted_x, predicted_y = pf_or_robot.avgXY()
            distance = helpers.distance_between((predicted_x, predicted_y),(robot.x,robot.y))
            print("wmax=%f, wavg=%f"%(wmax, wavg), end=" ")
            print("pred(%d,%d)"%(predicted_x, predicted_y), end=" ")
            print("dist=%d" % distance, end=" ")
            parameters['displayDynamicText'].append("wavg=%.2f" % wavg)
            parameters['displayDynamicText'].append("pred(%d,%d)" % (predicted_x, predicted_y))
            parameters['displayDynamicText'].append("dist=%d" % distance)
        else:
            print("!ERROR! Illegal state")
            break

        OutputData(parameters, iteration, pf_or_robot)
        iteration += 1
        if iteration > parameters['iterations']:
            break

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
