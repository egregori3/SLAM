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

    # Robot model parameters
    parameters['measurementSigmaNoise']           = 2.0      # Sigma for measurement noise
    parameters['headingSigmaNoise']               = 0.15     # Sigma for turning noise
    parameters['distanceSigmaNoise']              = 0.003    # Sigma for distance noise
    parameters['distanceDivider']                 = 10
    parameters['lidarSamples']                    = 720
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
        wmax = pf.maxWeight()
        print("Output %s"%(filename))
        image = display.Display(parameters, pf, wmax, filename)
    if parameters['plotSamples']:
        filename = parameters['outputPath']+"/lidar" + str(index)
        parameters['dataLoggerObject'].plotSamples(pf.particles, filename)


#==============================================================================
#
# Plot sample data
#
#==============================================================================
def PlotParticleSamples(parms):
    #plotSamples(self, particle_list, title)
    pass

#==============================================================================
#
# Particle Filter based prediction
#
#==============================================================================
def ParticleFilterSimulation(parms, robot):
    print('Particle Filter Simulation')
    # Brute force simulation of multiple possible parameters
    for sim in range(1,parms['simulations']+1):
        print('Simulation #', sim)

        # Instantiate particle filter with N particles and inital position
        pf = alg.ParticleFilter(parms, robot)
        OutputData(parms, 0, pf)
        return
        for i in range(1, parms['iterations']):
            print("Iteration %d: robot(%d, %d)"%(i, int(robot.x), int(robot.y)), end=" ")
            result = pf.update()
            wmax = pf.maxWeight()
            wavg = pf.avgWeight()
            predicted_x, predicted_y = pf.avgXY()
            distance = helpers.distance_between((predicted_x, predicted_y),(robot.x,robot.y))
            print("wmax=%f, wavg=%f"%(wmax, wavg), end=" ")
            print("pred(%d,%d)"%(predicted_x, predicted_y), end=" ")
            print("dist=%d" % distance, end=" ")
            parms['displayDynamicText'].append("wavg=%.2f" % wavg)
            parms['displayDynamicText'].append("pred(%d,%d)" % (predicted_x, predicted_y))
            parms['displayDynamicText'].append("dist=%d" % distance)
            OutputData(parms, i, pf)
            print("\n")
            if result:
                break

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
        # Drive robot to valid weight
        while(robot.valid_weight() == False):
            robot.move()
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
    print("-a arenaFilename -i iterations -p numberOfParticles")
    print("[-v verbose] [-o outputPath] [-d dataFilename] [-m plot samples] [-g output graphics]")
    print("-o must be specified with -d")
    print("-o must be specified with -m")
    print("-o must be specified with -g")
    sys.exit(1)

def main(argv):
    parameters = dict()
    Config(parameters)
    try:
        opts, args = getopt.getopt(argv, "vgi:o:a:d:p:m:x:y:h:")
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

    # Test command line parameters
    try:
        print("arenaFilename       = "+parameters['arenaFilename'])
        if 'outputPath' in parameters:
            print("outputPath          = "+parameters['outputPath'])
        print("Number of particles = "+str(parameters['numberOfParticles']))
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

    robot = PlaceRobot(parameters)
    dumpConfig(parameters)
    ParticleFilterSimulation(parameters, robot)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
