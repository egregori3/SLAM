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
import time                         # Used for performance measuring
import arena
import display
import DataLogger
import TrackingFilter
import Configuration as Config
import RobotServer as rs

#==============================================================================
#
# usage
#
#==============================================================================
def usage():
    print("!ERROR! Illegal parameter")
    print("Required: -a arenaFilename -i iterations ")
    print("Optional strings:  [-o outputPath] [-d dataFilename] [-m plot samples] [-s server IP]")
    print("Optional ints: [-x -y -h Initial robot position] [-w Initial weight threshold] [-p numberOfParticles]")
    print("Optional Flags: [-v verbose] [-g output graphics]")
    print("-o must be specified with -d")
    print("-o must be specified with -m")
    print("-o must be specified with -g")
    sys.exit(-1)

#==============================================================================
#
# main
#
#==============================================================================
def main(argv):
    parameters = dict()
    config = Config.Configuration(parameters)
    try:
        opts, args = getopt.getopt(argv, "vgi:o:a:d:p:m:x:y:h:w:s:")
    except getopt.GetoptError:
        usage()
    for opt, arg in opts:
        if opt in ("-v"):  # -v verbose
            parameters['verbose']           = True
        elif opt in ("-m"):
            parameters['plotSamples']       = True
        elif opt in ("-g"):
            parameters['plotGraphics']      = True
        elif opt in ("-o"):
            parameters['outputPath']        = arg
        elif opt in ("-a"):
            parameters['arenaFilename']     = arg
        elif opt in ("-d"):
            parameters['dataFilename']      = arg
        elif opt in ("-s"):
            parameters['serverIP']          = arg           # Server IP address fro getting data from robot
        elif opt in ("-i"):
            parameters['iterations']        = int(arg)      # Number of times the robot is moved
        elif opt in ("-p"):
            parameters['numberOfParticles'] = int(arg)
        elif opt in ("-x"):
            parameters['robotX']            = int(arg)
        elif opt in ("-y"):
            parameters['robotY']            = int(arg)
        elif opt in ("-h"):
            parameters['robotH']            = float(arg)
        elif opt in ("-w"):
            parameters['initialWeightThres']= float(arg)

    if config.verify(parameters):
        usage()

    print("Initializing arena")
    parameters['arena'] = arena.Arena(parameters)

    datalogger_object = DataLogger.DataLogger()
    if 'dataFilename' in parameters:
        print("Initializing datalogger")
        datalogger_object.startDataLogger(parameters)

    prediction = (0,0,0)
    robot_server = rs.RobotServer(parameters)
    if robot_server.start(parameters) < 0:
        print("Failure starting the server")
        sys.exit(-2)
    for iteration in range(parameters['iterations']):
        start_time = time.perf_counter()
        dtext = [("Iteration %d"%iteration,False)] # Only output to console
        state = robot_server.getDataFromRobot(parameters, dtext)
        prediction = TrackingFilter.particleFilter(parameters, state, dtext)
        elapsed_time = time.perf_counter() - start_time
        dtext.append(("Time: %f"%elapsed_time,False))
        for field in dtext:
            print(field[0], end=" ")
        print()
        if parameters['plotGraphics']:
            filename = parameters['outputPath']+"/iteration" + str(iteration)
            display.Display(parameters, filename, dtext, prediction)
        if parameters['plotSamples']:
            self.datalogger_object.plotSamples(parameters, parameters['outputPath']+"/lidar" + str(iteration))

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
