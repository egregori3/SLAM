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

import math
# import random
# import numpy as np

#==============================================================================
#
# Particle - LidarBot model
#
#            Lidarbot is a differential drive robot with a 360 degree lidar.
#            The robots static charactericts are: X, Y, heading
#            The robots dynamic characteristics are: turning slip and forward slip.
#
#            The lidar sensor provides a vector of 720 distances from the robot center.
#            The magnetometer sensor provides a heading.
#
#            robot_object should point to the robot object.
#            robot_object should be None if the particle is a robot.
#
#==============================================================================
class Particle:
        def __init__(self, parms, myid = 0, text):
            # Init static aspects of the particle
            self.id                 = myid                           # Particle ID for debugging
            self.lidar_max_dist     = parms['lidarMaxDistance']
            self.robot_path         = parms['robotPath']
            self.lidar_samples      = parms['lidarSamples']
            self.arena              = parms['arena']

            # Noise
            self.measNoise          = parms['measurementSigmaNoise'] # Measurement noise - sigma
            self.headingNoise       = parms['headingSigmaNoise']     # Heading noise - sigma  
            self.distanceNoise      = parms['distanceSigmaNoise']    # Distance noise - sigma 
            self.text               = "P"+str(myid)

        # After placement, the weight is valid
        def place(self, x, y, robot_heading):
            self.x                  = int(x)                         # Initial position
            self.y                  = int(y)
            self.weight             = 0.0                            # Initial weight
            self.samples            = [0] * self.lidar_samples       # List of samples representing distances
            self.samples = self.arena.readLidar(self.x, self.y, 
                                        robot_heading, self.samples) # Update Lidar after placement
            if self.robot_object != None:
                self.__setWeight()                                   # Update particle weights
            return self.weight

        # Move particle
        # Scan Lidar
        # For particles, update weights
        # After moving, the weight is valid
        def move(self, robot_distance_in_pixels, robot_heading):
            collision = False
            for i in range(robot_distance_in_pixels):
                self.__readLidar(front_only=True)
                if self.samples[0] <= 1:
                    collision = True
                    break
                # no collision
                self.x += math.cos(robot_heading)
                self.y += math.sin(robot_heading)
            self.x = int(self.x)
            self.y = int(self.y)
            self.__readLidar(robot_heading)
            self.__setWeight()
            return collision

        def isThisARobot(self):
            return (self.robot_object == None)

        # if the lidar data is all max, the weight is not valid
        def validWeight(self):
            for sample in self.samples:
                if sample < self.lidar_max_dist-2:
                    return True
            return False

        # Calculate a weight by comparing this particles lidar data with the robot's lidar data
        # If the robot is in a deadzone, the calculated weight is invalid.
        # It is important that the place() function does not try to place a particle
        # in an invalid location.
        def __setWeight(self):
            # If the robot sensor data is not valid do not change the weight
            if self.robot_object.valid_weight():
                if self.valid_weight() == False:
                    self.weight = 0.0
                    return 0.0
                result = np.corrcoef(np.array(self.robot_object.samples), np.array(self.samples))
                if result[0][1] == result[1][0]:
                    if result[0][1] > 0:
                        self.weight = result[0][1]
                    else:
                        self.weight = 0
                    return self.weight
#                   px = self.Gaussian(meas[0], self.meas_noise, self.x)
#                   py = self.Gaussian(meas[1], self.meas_noise, self.y)

        # Gaussian - from robot class
        def __gaussian(self,mu, sigma, x):
                # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
                return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * math.pi * (sigma ** 2))


