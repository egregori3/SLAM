# Written by Eric Gregori
from math import *
import random


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
            self.parms              = parms                     # Parameters
            self.id                 = myid                      # Particle ID for debugging
            self.distance_divider   = parms['distanceDivider']  # Used to break down distance into steps
            self.distance           = parms['distanceIteraton'] # Move this far per iteration

            # Noise
            self.measNoise          = parms['measurementSigmaNoise'] # Measurement noise - sigma
            self.headingNoise       = parms['headingSigmaNoise']     # Heading noise - sigma  
            self.distanceNoise      = parms['distanceSigmaNoise']    # Distance noise - sigma 
            self.robot_object       = robot_object              # None = this is a robot

        def place(self):
            # Dynamic aspects of the particle
            x, y, heading           = self.__randomLocation()
            self.x                  = x                         # Initial position
            self.y                  = y
            self.heading            = heading                   # Initial heading
            self.weight             = 0.0                       # Initial weight
            self.samples            = [0] * self.parms['lidarSamples'] # List of samples representing distances
            if self.this_is_a_robot():
                self.parms['robotPath'].append((int(x),int(y)))

        def move(self):
            if not self.this_is_a_robot():                      # This is a particle
                self.heading = self.robot_object.heading        # Set particle heading to robot heading
            d = max(1,self.distance/self.distance_divider)
            for i in range(self.distance_divider):
                # Move particle
                tx, ty = self.__move(self.x,self.y,d,self.heading)
                if self.__getSample(tx, ty, self.heading) > self.parms['lidarMaxDistance']/4:
                    # Particle did NOT hit wall, so let it move
                    self.x = tx
                    self.y = ty
                else:
                    # If robot, select a new heading for the next iteration
                    if self.this_is_a_robot():
                        self.__setNewHeading()
                    break
            if self.this_is_a_robot():
                self.parms['robotPath'].append((int(self.x),int(self.y)))
            else:
                self.__readLidar()

        # Calculate a weight by comparing this particles lidar data with the robot's lidar data
        def weight(self, robot_particle):
            self.__readLidar()

#               px = self.Gaussian(meas[0], self.meas_noise, self.x)
#               py = self.Gaussian(meas[1], self.meas_noise, self.y)

        def this_is_a_robot(self):
            return (self.robot_object == None)

        def __setNewHeading(self):
            max_distance = self.parms['lidarMaxDistance']
            samples = len(self.samples)
            angle_sample = (2*pi)/samples
            self.__readLidar()
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
            print(left,right,self.heading)

        def __getSample(self, px, py, heading):
            # send pulse from x,y towards heading, stop when hitting wall
            arena = self.parms['arena']
            for d in range(self.parms['lidarMaxDistance']):
                x,y = self.__move(px, py, d, heading)
                if arena.CheckXY(x,y):
                    break
            return d

        def __readLidar(self):
            # Measure distance while incrementing heading.
            # Start scan at robot heading.
            scan = self.heading
            angle_adder = (2*pi)/self.parms['lidarSamples']
            for r in range(self.parms['lidarSamples']):
                self.samples[r] = self.__getSample(self.x, self.y, scan)
                scan = self.__angletrunc(scan + angle_adder)

        def __randomLocation(self):
            arena = self.parms['arena']
            while(1):
                x = int(random.random()*self.parms['arenaWidth'])
                y = int(random.random()*self.parms['arenaHeight'])
                h = random.random()*(2 * pi)
                if not arena.CheckXY(x,y):
                    break
            return x,y,h

        # Truncate angle - from robot class
        def __angletrunc(self,a):
                while a < 0.0:
                        a += pi * 2
                return ((a + pi) % (pi * 2)) - pi

        # Gaussian - from robot class
        def __gaussian(self,mu, sigma, x):
                # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
                return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

        def __move(self,x,y,d,heading):
                x += d * cos(heading)
                y += d * sin(heading)
                return x,y


