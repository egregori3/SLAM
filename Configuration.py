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

class Configuration:
    def __init__(self, parameters):
        # Simulation parameters
        parameters['verbose']                         = False
        parameters['arenaRegion']                     = 10

        # Particle filter parameters
        parameters['numberOfParticles']               = 0

        # Display parameters
        parameters['plotGraphics']                    = False
        parameters['plotSamples']                     = False
        parameters['displayTextX']                    = 50
        parameters['displayTextY']                    = 50

        # Robot model parameters
        parameters['measurementSigmaNoise']           = 2.0      # Sigma for measurement noise
        parameters['headingSigmaNoise']               = 0.15     # Sigma for turning noise
        parameters['distanceSigmaNoise']              = 0.003    # Sigma for distance noise

        # Robot physical parameters
        parameters['lidarSamples']                    = 36
        parameters['lidarMaxDistance']                = 50       # Max distance in pixels
        parameters['distanceToPixel']                 = 3        # Robot count/pixel ratio
                                                                 # So if the robot moves 1000 counts
                                                                 # for every 10 pixels, this should be 10
        # Simulated robot parameters
        parameters['robotDistBetUpdates']             = 21       # The simulated robot moves X pixels for each update
        parameters['robotTurnAngle']                  = int(parameters['lidarSamples']/6)

    def dump(self, parameters):
        print('Configuration')
        print(parameters)

    def verify(self, parameters):
        # Test command line parameters
        usage = False
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
            usage = True
        print()
        print()
        if 'dataFilename' in parameters and not 'outputPath' in parameters:
            print("!ERROR! -o must be specified with -d")
            usage = True
        if parameters['plotSamples'] and not 'outputPath' in parameters:
            print("!ERROR! -o must be specified with -m")
            usage = True
        if parameters['plotGraphics'] and not 'outputPath' in parameters:
            print("!ERROR! -o must be specified with -g")
            usage = True
        return usage

