# Written by Eric Gregori

from math import *
import random
from helpers import *
from HexBugModel import *

#==============================================================================
#
# Configuration Variables
#
#==============================================================================
enableResampleWithInvalidMeasurement    = False        # False to disable resampling with invalid measurement
percentageOfParticleToReplace           = 0.2           # Percentage of particle to replace 1.0 = 100%
history                                 = 5             # Length of history tails used for particle updating

# Debugging Variables
enableParticleDump                      = False         # Dump particles to Console

def dumpPFConfig():
        print('Particle Filter Configuration')
        print(enableResampleWithInvalidMeasurement)     # Enable resampling during predict only
        print(percentageOfParticleToReplace)            # Percentage of particle to replace 1.0 = 100%
        print(history)                                  # Length of history tails used for particle updating
        print(enableParticleDump)

#==============================================================================
#
# Particle Filter
#
#==============================================================================
class ParticleFilter:
    def __init__(self, parms, start, turning_noise, distance_noise ):
        self.meas               = []
        self.particles          = []
        self.N                  = parms['numberOfParticles']
        self.id                 = 0
        self.meas_noise         = parms['measurementSigmaNoise']
        self.turning_noise      = turning_noise
        self.distance_noise     = distance_noise
        self.parms              = parms
        heading_adder = (2.0*3.14159)/self.N
        heading = 0.0
        for i in range(self.N):
            tx,ty = self.PlaceParticle()
            self.particles.append(particle(parms,x=tx,y=ty,heading=heading,myid=self.id))
            self.particles[-1].setnoise(self.meas_noise, self.turning_noise, self.distance_noise)
            heading += heading_adder 
            self.id += 1
        self.meas.append(start)
        return

    def PlaceParticle(self):
        arena = self.parms['arena']
        while(1):
            x = int(random.random()*self.parms['arenaWidth'])
            y = int(random.random()*self.parms['arenaHeight'])
            if not arena.CheckXY(x,y):
                break
        return x,y

    # move particles
    # if predictOnly == 0
    #   calculate weights 
    #   resample particles
    # normalize weights
    # prediction returned as weighted average
    # curr_pos_valid = True -  curr_pos is a measurement
    # curr_pos_valid = False - curr_pos is a prediction
    def update(self, curr_pos, steps=1, curr_pos_valid=True):
        self.meas.append(curr_pos)

        # move particles
        for s in range(steps):
                for r in self.particles:
                        r.move()

        # calculate weights - resample - replace stale particles
        # calculate weights
        w = []
        for p in self.particles:
                w.append(p.sense(curr_pos))

        if curr_pos_valid == True or enableResampleWithInvalidMeasurement == True:
                # resample particles
                # Use prof Thurn's wheel to resample particles
                np=[]
                index=int(random.random()*self.N)
                beta=0.0
                mw = max(w)
                nw = []
                for i in range(self.N):
                        beta+=random.random()*2.0*mw
                        while beta > w[index]:
                                beta -= w[index]
                                index=(index+1) % self.N
                        np.append(self.particles[index])
                        nw.append(w[index])

                # deep copy particles
                self.particles = []
                n = int(len(np)*percentageOfParticleToReplace)
                # Learn heading and distance
                # Deep copy 100%-n% of the particles - X,Y,heading, distance are preserved - turning = 0.0 - weight is replaced
                for z in range(len(np)-n):
                        self.particles.append(particle(self.parms,x=np[z].x,y=np[z].y,heading=np[z].heading,turn=0.0,dist=np[z].distance,weight=nw[z],myid=np[z].id))
                        self.particles[-1].setnoise(self.meas_noise, self.turning_noise, self.distance_noise)

                # Replace a n% of the particles based on one of two strategies
                # This completely random particle has a weight of 0 since it has not been compared against a measurement yet.
                # Strategy 1 - Randomly pick new turning angle and distance (preserving heading to provide momentum)
                # Strategy 2 - Average history of heading, random distance
                if history > 0 and len(self.meas) >= history:
                        aa = 0.0
                        c = 0
                        for mi in range(len(self.meas)-history,len(self.meas)-1):
                                aa += get_heading(self.meas[mi], self.meas[mi+1])
                                c += 1
                        n = z
                        # X,Y preserved - heading=average - turning,distance=random, weight=0
                        for i in range(n+1,len(np)):
                                self.particles.append(particle(self.parms,x=np[i].x,y=np[i].y,heading=aa/c,weight=0.0,myid=np[i].id)) 
                                self.particles[-1].setnoise(self.meas_noise, self.turning_noise, self.distance_noise)
                else:
                        # Strategy 1 - Randomly pick new heading and distance (preserving heading to provide momentum)
                        # X,Y, heading preserved - turning,distance=average, weight=0
                        n = z
                        self.id += 1
                        for i in range(n+1,len(np)):
                                self.particles.append(particle(self.parms,x=np[i].x,y=np[i].y,heading=np[i].heading, weight=0.0,myid=self.id))
                                self.particles[-1].setnoise(self.meas_noise, self.turning_noise, self.distance_noise)

        # Dump particles to console
        if enableParticleDump == True:
                print('curr_pos', curr_pos, curr_pos_valid)
                for p in self.particles:
                        p.dump();

        # normalize weights
        wsum = 0.0    
        maxw = 0.0
        maxwi = 0
        for i in range(len(self.particles)):
                wsum += self.particles[i].weight 
                if self.particles[i].weight > maxw:
                        maxw = self.particles[i].weight
                        maxwi = i

#        if curr_pos_valid == False: # Prediction only phase
#                return self.particles[maxwi].getXY()                


        # prediction returned as weighted average
        x = 0.0
        y = 0.0
        if wsum > 0.0:
                for p in self.particles:
                        pos = p.getXY()
                        x += (pos[0] * (p.weight/wsum))
                        y += (pos[1] * (p.weight/wsum))
        else:
                print("ZERO Weights - LOST TRACK")
                x = -1
                y = -1

        return [x,y]

