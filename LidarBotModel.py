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
import random
import numpy as np

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
        def __init__(self, parms, myid = 0, robot_object = None):
            # Init static aspects of the particle
            self.id                 = myid                      # Particle ID for debugging
            self.distance_divider   = parms['distanceDivider']  # Used to break down distance into steps
            self.distance           = parms['distanceIteraton'] # Move this far per iteration
            self.lidar_max_dist     = parms['lidarMaxDistance']
            self.robot_path         = parms['robotPath']
            self.lidar_samples      = parms['lidarSamples']
            self.arena              = parms['arena']

            # Noise
            self.measNoise          = parms['measurementSigmaNoise'] # Measurement noise - sigma
            self.headingNoise       = parms['headingSigmaNoise']     # Heading noise - sigma  
            self.distanceNoise      = parms['distanceSigmaNoise']    # Distance noise - sigma 
            self.robot_object       = robot_object              # None = this is a robot
            if robot_object == None:
                self.text = "Robot"
            else:
                self.text = "P"+str(myid)

        # After placement, the weight is valid
        def place(self, cmd_dict):
            # Dynamic aspects of the particle
            if 'constrainedRandom' in cmd_dict:
                (px,py,w,h) = cmd_dict['constrainedRandom']
                x, y, heading       = self.__constrainedLocation(w, h, px, py)
            elif 'setPosition' in cmd_dict:
                (x,y,heading) = cmd_dict['setPosition']
            else:
                x, y, heading       = self.__randomLocation()

            self.x                  = int(x)                    # Initial position
            self.y                  = int(y)
            if self.this_is_a_robot():
                self.heading        = heading                   # Set robot
            else:
                self.heading        = self.robot_object.heading # Partical is always robot heading
            self.weight             = 0.0                       # Initial weight
            self.samples            = [0] * self.lidar_samples # List of samples representing distances
            self.__readLidar()                                  # Update Lidar after placement
            if self.this_is_a_robot():
                self.robot_path.append((int(x),int(y)))
            else:
                self.__setWeight()                              # Update particle weights
            return self.weight

        # Move particle
        # Scan Lidar
        # For particles, update weights
        # After moving, the weight is valid
        def move(self):
            if not self.this_is_a_robot():                      # This is a particle
                self.heading = self.robot_object.heading        # Set particle heading to robot heading
            d = max(1,self.distance/self.distance_divider)
            for i in range(self.distance_divider):
                # Move particle
                tx, ty = self.__move(self.x,self.y,d,self.heading)
                if self.__getSample(tx, ty, self.heading) > self.lidar_max_dist/4:
                    # Particle did NOT hit wall, so let it move
                    self.x = tx
                    self.y = ty
                else:
                    # If robot, select a new heading for the next iteration
                    if self.this_is_a_robot():
                        self.__setNewHeading()
                    break
            # After motion, update state information
            self.x = int(self.x)
            self.y = int(self.y)
            self.__readLidar()
            if self.this_is_a_robot():
                self.robot_path.append((self.x,self.y))
            else:
                self.__setWeight()

        def this_is_a_robot(self):
            return (self.robot_object == None)

        # if the lidar data is all max, the weight is not valid
        def valid_weight(self):
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

        def __setNewHeading(self):
            max_distance = self.lidar_max_dist
            samples = len(self.samples)
            angle_sample = (2 * math.pi)/samples
            for right in range(int(samples/4)):
                if self.samples[right] > (3*max_distance/4):
                    break
            for left in range(samples-1, samples-int(samples/4), -1):
                if self.samples[left] > (3*max_distance/4):
                    left = samples - left
                    break
            if left < right:
                self.heading = self.__angletrunc(self.heading-(left*angle_sample))
            else:
                self.heading = self.__angletrunc(self.heading+(right*angle_sample))

        def __getSample(self, px, py, heading):
            # send pulse from x,y towards heading, stop when hitting wall
            for d in range(self.lidar_max_dist):
                x,y = self.__move(px, py, d, heading)
                if self.arena.CheckXY(x,y):
                    break
            return d

        def __readLidar(self):
            # Measure distance while incrementing heading.
            # Start scan at robot heading.
            scan = self.heading
            angle_adder = (2 * math.pi)/self.lidar_samples
            for r in range(self.lidar_samples):
                self.samples[r] = self.__getSample(self.x, self.y, scan)
                scan = self.__angletrunc(scan + angle_adder)

        # Width, Height, x, y 
        def __constrainedLocation(self, w, h, x, y):
            while(1):
                x += int((random.random()*w) - w/2)
                y += int((random.random()*h) - h/2)
                x = max(0,x)
                y = max(0,y)
                x = min(x,self.arena.width-1)
                y = min(y,self.arena.height-1)
                if not self.arena.CheckXY(x,y):
                    break
            h = random.random()*(2 * math.pi)
            return x,y,h

        def __randomLocation(self):
            while(1):
                x = int(random.random()*self.arena.width)
                y = int(random.random()*self.arena.height)
                h = random.random()*(2 * math.pi)
                if not self.arena.CheckXY(x,y):
                    break
            return x,y,h

        # Truncate angle - from robot class
        def __angletrunc(self,a):
                while a < 0.0:
                        a += math.pi * 2
                return ((a + math.pi) % (math.pi * 2)) - math.pi

        # Gaussian - from robot class
        def __gaussian(self,mu, sigma, x):
                # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
                return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * math.pi * (sigma ** 2))

        def __move(self,x,y,d,heading):
                x += d * math.cos(heading)
                y += d * math.sin(heading)
                return x,y


