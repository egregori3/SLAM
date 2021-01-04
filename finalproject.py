# Hexbug simulator using particle filter
# written by Eric Gregori
import getopt
import sys
import os
import math
import random
import helpers
import HexBugModel as model
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
    parameters['enableImages']                    = True
    parameters['enableLowPass']                   = True         # True to enable low pass filter
    parameters['numberOfParticles']               = 1000         # Number of particles
    parameters['measurementSigmaNoise']           = 2.0          # Sigma for measurement noise
    parameters['turningSigmaNoise']               = 0.15         # Sigma for turning noise
    parameters['distanceSigmaNoise']              = 0.003        # Sigma for distance noise
    parameters['trackingEnd']                     = 60           # Number of measurements to track
    parameters['step']                            = 1            # Input measurement step
    parameters['simulations']                     = 1            # Number of simulations
    parameters['predictionSteps']                 = 60           # Number of steps to predict

    # Debugging Variables
    parameters['enableTrackingParticleDisplay']   = True         # Display particles
    parameters['enablePathDisplay']               = True         # Display path
    parameters['enableCalcTrackingError']         = False        # Tracking error calculation
    parameters['enablePredictionDisplay']         = True         # Display prediction path
    parameters['enableTrainingMode']              = False        # False is normal mode, True is training mode
    parameters['enableTrajectorySearch']          = False        # True to enable trajectory search 

def dumpConfig(parameters):
    print('Configuration')
    print(parameters)

#==============================================================================
#
# Load/Parse input File of X,Y points
#
#==============================================================================
def LoadXYPoints(parameters):
    input_file = open(parameters['startFilename'], 'r')

    #==========================================================================
    #
    # Load test measurements and training measurements from input file
    #
    #==========================================================================
    # Load input_file
    input_data = [literal_eval(l.strip()) for l in input_file.readlines()][:]

    # Extract input data
    measurements = []
    for idindex in range(len(input_data)):
        if input_data[idindex][0] > 0 and input_data[idindex][1] > 0:
                measurements.append(input_data[idindex])
        else:
                break

    input_file.close()

    # extract training data
    parameters['measurements'] = measurements

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
    print('Particle Filter generated predictions')
    measurements = parameters['measurements'] # save some typing
    if parameters['enableTrainingMode'] == True:
        predictiontest = measurements[trackingEnd:]
        measurements = measurements[:trackingEnd]

    if parameters['enableLowPass']:   # Low Pass Filter - Filter measurements
        measurements = helpers.lowPassFilter(measurements)

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
        dumpConfig(parameters)
        alg.dumpPFConfig()

        # Instantiate particle filter with N particles and inital position
        pf = alg.ParticleFilter(parameters, measurements[0], turningSigma, distanceSigma)
        for particle in pf.particles:
            particle.dump()
        DisplayParticles(parameters,"/init.jpg", pf)
        stop

        # Tracking measurements
        prevP           = measurements[0]
        frameNum        = 0
        for i in range(1,len(measurements),parameters['step']):
                prediction = pf.update(measurements[i], steps=parameters['step'], curr_pos_valid=True)
#                prediction = pf.update(prediction,steps=1,curr_pos_valid=False)
                if prediction[0] < 0 or prediction[1] < 0: break
                tracking.append(prediction)
                print('Tracking',measurements[i], prediction)
                if parameters['enableTrackingParticleDisplay'] == True:
                    DisplayParticles(parameters,"/tracking" + str(frameNum) + ".jpg", pf)
                frameNum += 1
                prevP = prediction

        if prediction[0] < 0 or prediction[1] < 0: continue

#==============================================================================
#
# Place robot at random location in arena
#
#==============================================================================
def PlaceRobot(parameters):
    arena = parameters['arena']
    while(1):
        x = int(random.random()*parameters['arenaWidth'])
        y = int(random.random()*parameters['arenaHeight'])
        if not arena.CheckXY(x,y):
            break
    parameters['robotPath'] = [(x,y)]
    print("Robot installed at %d, %d"%(x,y))

def main(argv):
    parameters = {}
    parameters['startFilename'] = ""
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
        elif opt in ("-i"):
            parameters['startFilename'] = arg
        elif opt in ("-o"):
            parameters['outputPath'] = arg
        elif opt in ("-a"):
            parameters['arenaFilename'] = arg

    Config(parameters)
    parameters['arena'] = arena.Arena(parameters)
    PlaceRobot(parameters)
    LoadXYPoints(parameters)
    ParticleFilterSimulation(parameters)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
