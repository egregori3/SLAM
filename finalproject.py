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
    # Simulation parameters
    parameters['outputPath']                      = ""
    parameters['arenaFilename']                   = ""
    parameters['dataFilename']                    = ""
    parameters['plotFilename']                    = ""

    parameters['plotGraphics']                    = False
    parameters['verbose']                         = False
    parameters['simulations']                     = 1        # Number of simulations
    parameters['iterations']                      = 10       # Number of times the robot is moved
    parameters['displayDynamicText']              = list()

    # Particle filter parameters
    parameters['numberOfParticles']               = 100      # Number of particles

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
# Display partices
#
#==============================================================================
def DisplayParticles(parameters, filename, pf):
    if parameters['plotGraphics']:
        wmax = pf.maxWeight()
        filename = parameters['outputPath']+"/"+filename
        image = display.Display(parameters, pf, wmax, filename)

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
        for i in range(parms['iterations']):
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
            DisplayParticles(parms,"/looking" + str(i) + ".jpg", pf)
            print("\n")
            if result:
                break

#==============================================================================
#
# Place the robot into the arena
#
#==============================================================================
def PlaceRobot(parameters):
    parameters['robotPath'] = []
    robot = model.Particle(parms=parameters)
    robot.place(place_robot=True)
    try:
        parameters['dataLoggerObject'].plotSamples([robot], "Robot")
    except KeyError:
        pass
    return robot

#==============================================================================
#
# main
#
#==============================================================================
def usage():
    print("!ERROR! Illegal parameter")
    print("-a arenaFilename -i iterations -p numberOfParticles")
    print("[-v verbose] [-o outputPath] [-d dataFilename] [-m plotFilename] [-g output graphics]")
    print("-o must be specified with -d")
    print("-o must be specified with -m")
    print("-o must be specified with -g")
    sys.exit(1)

def main(argv):
    parameters = dict()
    Config(parameters)
    try:
        opts, args = getopt.getopt(argv, "vgi:o:a:d:p:m:")
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
            parameters['plotFilename']      = arg
        elif opt in ("-g"):
            parameters['plotGraphics']      = True

    # Test command line parameters
    try:
        print("arenaFilename       = "+parameters['arenaFilename'])
        print("Number of particles = "+str(parameters['numberOfParticles']))
        print("iterations          = "+str(parameters['iterations']))
        if parameters['plotGraphics']: 
            print("Graphics output enabled")
        if 'plotFilename' in parameters: 
            print("Plotting output enabled")
    except KeyError:
        usage()
    print()
    print()
    if'dataFilename' in parameters and not 'outputPath' in parameters:
        print("!ERROR! -o must be specified with -d")
        usage()
    if'plotFilename' in parameters and not 'outputPath' in parameters:
        print("!ERROR! -o must be specified with -m")
        usage()
    if'plotGraphics' in parameters and not 'outputPath' in parameters:
        print("!ERROR! -o must be specified with -g")
        usage()

    parameters['displayStaticText'] = ["p="+str(parameters['numberOfParticles'])]
    parameters['arena'] = arena.Arena(parameters)
    parameters['dataLoggerObject'] = DataLogger.DataLogger(parameters)
    if parameters['dataFilename']: parameters['dataLoggerObject'].startDataLogger()

    robot = PlaceRobot(parameters)
    dumpConfig(parameters)
    ParticleFilterSimulation(parameters, robot)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
