# Written by Eric Gregori
from math import *
import random


#==============================================================================
#
# Particle - HexBug model
#            Based on Prof. Thrun robot class
#            Differential drive model
#
#==============================================================================
class particle:
        def __init__(self, parms, x = 0.0, y = 0.0, heading = 0.0, turn = 1000, dist = 1000, weight = 0.0, myid = 0):
                print("input x=%d, y=%d"%(x,y))
                self.x                  = x
                self.y                  = y
                self.heading            = heading
                self.id                 = myid                  # Particle ID for debugging
                self.parms              = parms

                # Noise
                self.meas_noise         = 2.0                   # Measurement noise - sigma
                self.turning_noise      = 0.028                 # Turning noise - sigma  
                self.distance_noise     = 0.0028                # Distance noise - sigma 

                # Initial turning angle
                if turn < 1000:                              
                        self.turning    = turn                  # Required for deep copy
                else:
                        self.turning    = random.gauss(0.0, self.turning_noise)

                # Initial distance
                if dist < 1000:                             
                        self.distance   = dist                  # Required for deep copy
                else:
                        self.distance   = random.gauss(7.2,self.distance_noise)

                self.weight             = weight

                self.distance_divider   = 10                    # Used to break down distance into steps

        # Set noise
        def setnoise(self, meas_noise, turning_noise, distance_noise):
                self.meas_noise         = meas_noise
                self.turning_noise      = turning_noise
                self.distance_noise     = distance_noise

        # Truncate angle - from robot class
        def angle_trunc(self,a):
                while a < 0.0:
                        a += pi * 2
                return ((a + pi) % (pi * 2)) - pi

        # Gaussian - from robot class
        def Gaussian(self,mu, sigma, x):
                # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
                return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
      
        # Calculate bounce angle when hitting a wall
        # Direction below are from center
        # inputAngle    0.0 outputAngle         pi/2 outputAngle
        # pi/4          -pi/4                   3pi/4           (2.0*wallAngle)-inputAngle
        # pi/2          -pi/2                   pi/2            (2.0*wallAngle)-inputAngle
        # 3pi/4         -3pi/4                  pi/4            (2.0*wallAngle)-inputAngle
        # 0.0           0.0                     -pi,pi          (2.0*wallAngle)-inputAngle
        # -pi,pi        -pi,pi                  0.0             (2.0*wallAngle)-inputAngle
        # -pi/4         pi/4                    -3pi/4          (2.0*wallAngle)-inputAngle
        # -pi/2         pi/2                    -pi/2           (2.0*wallAngle)-inputAngle

        #               -pi/2
        #       -3pi/4          -pi/4
        #
        # -pi,pi                        0
        #
        #       3pi/4           pi/4
        #               pi/2
        #
        def bounce(self, incomingAngle, wallAngle):
                outgoingAngle = (2.0*wallAngle)-incomingAngle
                return outgoingAngle

        def nextPosition(self,x,y,d,heading):
                x += d * cos(self.heading)
                y += d * sin(self.heading)
                return x,y

        # Move the particle
        def move(self, turning=1000, distance=1000, max_turning_angle = pi):
                if turning == 1000:  turning = self.turning
                if distance == 1000: distance = self.distance

                turning = random.gauss(turning, self.turning_noise)
                distance = random.gauss(distance, self.distance_noise)

                # truncate to fit physical limitations
                turning = max(-max_turning_angle, turning)
                turning = min( max_turning_angle, turning)
                distance = max(0.0, distance)

                # Execute motion - heading - 0=3:00, pi/4=4:30, pi/2=6:00, 3pi/4=7:30, -pi,pi=9:00, -3pi/4=10:30, -pi/2=12:00
                self.heading += turning
                self.heading = self.angle_trunc(self.heading)
                x = self.x
                y = self.y
                d = distance/self.distance_divider
                for i in range(self.distance_divider):
                        self.nextPosition(x,y,d,self.heading)
                        if self.parms['arena'].CheckXY(x,y):
                                test = self.angle_trunc(self.bounce(self.heading,0.0))
                                tx,ty = self.nextPosition(x,y,d,test)
                                if self.parms['arena'].CheckXY(tx,ty) == False:
                                    test = self.angle_trunc(self.bounce(self.heading,0.0))
                                    x = tx
                                    y = ty
                                self.heading = test

#                x += ((self.distance_divider-i-1)*d) * cos(self.heading)
#                y += ((self.distance_divider-i-1)*d) * sin(self.heading)
                self.x = x
                self.y = y
                return [self.x,self.y]

        # Sense the particle position as a probability
        def sense(self, meas):
                px = self.Gaussian(meas[0], self.meas_noise, self.x)
                py = self.Gaussian(meas[1], self.meas_noise, self.y)
                return px*py

        # Absolute particle position - for dynamics testing
        def getXY(self):
                return [self.x,self.y]

        # Print particle parameters to screen
        def dump(self):
                print('particle',self.id, int(self.x), int(self.y))
                print(self.heading, self.turning, self.distance, self.weight)
                print(self.meas_noise, self.turning_noise, self.distance_noise)
                

