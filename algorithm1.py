# Written by Eric Gregori

import LidarBotModel as model

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
        keepThreashold = 0.9
        weights = [p.weight for p in self.particles]
        keep_indexes = [i for i in range(len(weights)) if weights[i] > keepThreashold]
        numkeep = len(keep_indexes)
        print("keep=%d"%(numkeep), end=" ")
        if numkeep > 0:
            dump_indexes = [i for i in range(len(weights)) if weights[i] <= keepThreashold]
            numdump = len(dump_indexes)
            print("dump=%d"%(numdump), end=" ")
            keep = 0
            dump = 0
            r = 0
            for d in dump_indexes:
                if keep < numkeep:
                    ki = keep_indexes[keep]
                    x=self.particles[ki].x
                    y=self.particles[ki].y
                    w=self.particles[ki].weight
                    self.particles[d].place(px=x, py=y)
                    if dump == 0: print("(%d,%d,%3f)"%(x,y,w), end="")
                    dump += 1
                    if dump >= int((numdump+numkeep)*0.1):
                        keep += 1
                        print("=%d"%(dump), end=" ")
                        dump = 0
                else:
                    print("=%d"%(dump), end=" ")
                    self.particles[d].place()
                    r += 1

            print("rand=%d"%(r), end=" ")
            if numkeep > int((numdump+numkeep)*0.9):
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

