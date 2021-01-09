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
from ast import literal_eval


#==============================================================================
#
# Init parameters
#
#==============================================================================
def Config(parameters):
    # Simulation parameters
    parameters['outputPath']                      = "Particles"
    parameters['arenaFilename']                   = ""
    parameters['dataFilename']                    = "data.txt"
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
    wmax = pf.maxWeight()
    image = display.Display(parameters['arena'].GetImage(),
                            parameters['robotPath'],
                            (parameters['displayStaticText'], parameters['displayDynamicText'])
    image.addParticles(pf, wmax)
    filename = parameters['outputPath']+"/"+filename
    image.saveImage(filename)

#==============================================================================
#
# Dump data to a csv file
#
#==============================================================================

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
            wmax = pf.maxWeight()
            wavg = pf.avgWeight()
            print("Iteration %d: robot %d, %d, %f, %f"%(i, int(robot.x), int(robot.y), wmax, wavg))
            parms['displayDynamicText'].append(" wmax="+str(wmax))
            parms['displayDynamicText'].append(" wavg="+str(wavg))
            result = pf.update()
            DisplayParticles(parms,"/looking" + str(i) + ".jpg", pf)
            if result:
                break

def PlaceRobot(parameters):
    parameters['robotPath'] = []
    robot = model.Particle(parms=parameters)
    robot.place(place_robot=True)
    return robot

def main(argv):
    parameters = dict()
    Config(parameters)
    print(__doc__)
    try:
        opts, args = getopt.getopt(argv, "vi:o:a:d:p:")
    except getopt.GetoptError:
        print("!ERROR! Illegal parameter")
        sys.exit(1)
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

    parameters['displayStaticText'] = ["p="+str(parameters['numberOfParticles'])]
    parameters['arena'] = arena.Arena(parameters)
    robot = PlaceRobot(parameters)
    dumpConfig(parameters)
    ParticleFilterSimulation(parameters, robot)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
