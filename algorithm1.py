# Written by Eric Gregori

from math import *
import random
from helpers import *
import LidarBotModel as model
import numpy as np

#==============================================================================
#
# Particle Filter
#
#==============================================================================
class ParticleFilter:
    def __init__(self, parms, robot):
        self.meas               = []
        self.particles          = []
        self.robot              = robot
        self.numparticles       = parms['numberOfParticles']
        myid                    = 0
        self.particles.append(robot)
        for i in range(self.numparticles):
            self.particles.append(model.Particle(parms, myid=myid, robot_object=robot))
            self.particles[-1].place()
            myid += 1

    # move particles
    def update(self):
        # move particles
        for p in self.particles:
            p.move()

        # refresh dead particles
        weights = [p.weight for p in self.particles]
        keep_indexes = [i for i in range(len(weights)) if weights[i] > 0.75]
        numkeep = len(keep_indexes)
        if numkeep > 0:
            dump_indexes = [i for i in range(len(weights)) if weights[i] <= 0.75]
            numdump = len(dump_indexes)
            i = keep_indexes[0]
            c = 0
            for d in dump_indexes:
                self.particles[d].place(px=self.particles[i].x, py=self.particles[i].y)
                c += 1
                if c > (numdump+numkeep)/numkeep:
                    i += 1
                    c = 0

            if numkeep > numdump:
                print("keep = %d, dump = %d"%(numkeep, numdump))
                return True

        else:
            for p in self.particles:
                p.place()

        return False

    def maxWeight(self):
        wmax = 0.0
        for p in self.particles:
            if p.weight > wmax:
                wmax = p.weight
        return wmax

    def avgWeight(self):
        avg = 0.0
        for p in self.particles:
            avg += p.weight
        return avg / len(self.particles)

