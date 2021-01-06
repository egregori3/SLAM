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
    parameters['simulations']                     = 1        # Number of simulations

    # Particle filter parameters
    parameters['numberOfParticles']               = 1000     # Number of particles

    # Robot model parameters
    parameters['measurementSigmaNoise']           = 2.0      # Sigma for measurement noise
    parameters['headingSigmaNoise']               = 0.15     # Sigma for turning noise
    parameters['distanceSigmaNoise']              = 0.003    # Sigma for distance noise
    parameters['distanceDivider']                 = 10
    parameters['lidarSamples']                    = 720
    parameters['distanceIteraton']                = 50       # Move 10 pixels per iteration
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
    wmax = 0.0
    for p in pf.particles:
        if p.weight > wmax:
            wmax = p.weight

    image = display.Display(parameters['arena'].GetImage(),parameters['robotPath'])
    image.AddParticles(pf, wmax)
    filename = parameters['outputPath']+"/"+filename
    image.SaveImage(filename)

#==============================================================================
#
# Particle Filter based prediction
#
#==============================================================================
def ParticleFilterSimulation(parameters):
    print('Particle Filter Simulation')
    dumpConfig(parameters)
    # Brute force simulation of multiple possible parameters
    for sim in range(1,parameters['simulations']+1):
        tracking                = []

        if parameters['simulations'] > 1:
                turningSigma    = random.random()*parameters['turningSigmaNoise']
                distanceSigma   = random.random()*parameters['distanceSigmaNoise']
        else:
                turningSigma    = parameters['turningSigmaNoise']
                distanceSigma   = parameters['distanceSigmaNoise']

        print('Simulation #', sim, ' -', turningSigma, distanceSigma)
        alg.dumpPFConfig()

        # Instantiate particle filter with N particles and inital position
        pf = alg.ParticleFilter(parameters, turningSigma, distanceSigma)

        frameNum = 0
        for i in range(predictionSteps):
                prediction = pf.update(prediction,steps=1,curr_pos_valid=False)
                DisplayParticles(parameters,"/looking" + str(frameNum) + ".jpg", pf)
                frameNum += 1

def PlaceRobot(parms):
    parms['robotPath'] = []
    robot = model.Particle(parms=parms)
    robot.place()
    return robot

class test:
    def __init__(self, robot):
        self.particles = [robot]

def main(argv):
    parameters = {}
    parameters['outputPath'] = "Particles"
    parameters['arenaFilename'] = ""
    parameters['verbose'] = False
    print(__doc__)
    try:
        opts, args = getopt.getopt(argv, "vi:o:a:")
    except getopt.GetoptError:
        sys.exit(1)
    for opt, arg in opts:
        if opt in ("-v"):  # -v verbose
            parameters['verbose'] = True
        elif opt in ("-o"):
            parameters['outputPath'] = arg
        elif opt in ("-a"):
            parameters['arenaFilename'] = arg

    Config(parameters)
    parameters['arena'] = arena.Arena(parameters)
    robot = PlaceRobot(parameters)
    pf = test(robot)
    for i in range(50):
        DisplayParticles(parameters, "test"+str(i)+".jpg", pf)
        robot.move()

#    ParticleFilterSimulation(parameters)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
