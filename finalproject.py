# Experiments in robot localization using particle filters (algorithm and simulator)
# Copyright (C) <2021>  Eric Gregori

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

import getopt
import sys
import os
import math
import random
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
    parameters['verbose']                         = False
    parameters['arenaRegion']                     = 10
    parameters['numberOfParticles']               = 0
    parameters['distanceDivider']                 = 10
    parameters['distanceIteraton']                = 30       # Move 10 pixels per iteration

    # Display parameters
    parameters['plotGraphics']                    = False
    parameters['plotSamples']                     = False
    parameters['simulations']                     = 1        # Number of simulations
    parameters['displayTextX']                    = 50
    parameters['displayTextY']                    = 50

    # Robot model parameters
    parameters['measurementSigmaNoise']           = 2.0      # Sigma for measurement noise
    parameters['headingSigmaNoise']               = 0.15     # Sigma for turning noise
    parameters['distanceSigmaNoise']              = 0.003    # Sigma for distance noise
    parameters['lidarSamples']                    = 36
    parameters['lidarMaxDistance']                = 50       # Max distance in pixels

def dumpConfig(parameters):
    print('Configuration')
    print(parameters)

#==============================================================================
#
# OutputData
#
#==============================================================================
def OutputData(parameters, index, pf, text, prediction):
    if parameters['plotGraphics']:
        filename = parameters['outputPath']+"/iteration" + str(index)
        print("Output %s"%(filename))
        image = display.Display(parameters, pf, filename, text, prediction)
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
    print("Robot initial state (x = %d, y = %d, h = %f)"%(robot.x, robot.y, robot.heading))
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
        prediction = (0,0,0)
        dtext = list()
        if state == 'addRobot':
            print("Placing robot")
            pf_or_robot = PlaceRobot(parameters)
            dtext.append("Placing robot")
            robot = pf_or_robot
            state = 'moveRobot'
        elif state == 'moveRobot':
            print("Move robot")
            if pf_or_robot.valid_weight() == False:
                pf_or_robot.move()
                dtext.append("Move robot to valid weight region")
            else:
                state = 'placeParticles'
            robot = pf_or_robot
        elif state == 'placeParticles':
            print("Place particles")
            pf_or_robot = alg.ParticleFilter(parameters, robot)
            dtext.append("Place particles")
            state = 'updateSimulation'
            (done, nump, avgw, keep, prediction) = pf_or_robot.pfData()
        elif state == 'updateSimulation':
            print("Iteration %d: robot(%d, %d)"%(iteration,robot.x,robot.y), end=" ")
            (done, nump, avgw, keep, prediction) = pf_or_robot.update()
        else:
            print("!ERROR! Illegal state")
            break

        dtext.append("r=%d,%d" % (robot.x, robot.y))
        if type(pf_or_robot) == alg.ParticleFilter:
            fields = ["p=%d,%d"%(nump,keep), "wavg=%f"%avgw]
            if prediction[2] > 0:
                fields.append("L=%d,%d"%(prediction[0],prediction[1]))
            for field in fields:
                print(field, end=" ")
                dtext.append(field)

        OutputData(parameters, iteration, pf_or_robot, dtext, prediction)
        iteration += 1
        if iteration > parameters['iterations']:
            break

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
