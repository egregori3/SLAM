# Written by Eric Gregori

import random
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
        self.width              = parms['arenaWidth']
        self.height             = parms['arenaHeight']
        self.arena              = parms['arena']
        pid                     = 0
        self.particles.append(robot) # The robot MUST move first
        particles = []
        return
        for i in range(self.numparticles):
            particles.append(model.Particle(parms, myid=i, robot_object=robot))
        print("Placing initial particles")
        index = 0
        while(pid < self.numparticles):
            position = self.arena.valid_regions[index]
            p = particles[pid]
            if self.placeByRegion(p,position[0],position[1], 0.5):
                self.particles.append(p)
                print("*", end="", flush=True)
                pid += 1
            index = (index + 1) % (len(self.arena.valid_regions)-1)
        print()
        print("Placed %d particles"%self.numparticles)

    def placeByRegion(self, particle, x, y, thres):
        cmd = {'constrainedRandom':(x,y,50,50)}
        particle.place(cmd)
        if particle.weight > thres:
            return True
        return False

    def placeByWeight(self, particle):
        weight = 0.0
        while(weight < 0.75):
            particle.place({})
            weight = particle.weight

    def placeNextTo(self, keep_particle, relocate_particle):
        x=keep_particle.x
        y=keep_particle.y
        w=keep_particle.weight
        # x,y is the location of the keep particle
        # Place the dump particle in a random location around the keep particle
        data = (x, y, 50, 50)
        relocate_particle.place({'constrainedRandom':data})
        return x,y,w

    # move particles
    # return True to end simulation
    def update(self):
        # move particles
        for p in self.particles:
            p.move()

        if not self.robot.valid_weight(): return False

        # refresh dead particles
        keepThreashold = 0.9
        particles = [p for p in self.particles if not p.this_is_a_robot()]
        # keep_particles is a list of particles with weights > keepThreashold
        keep_particles = [p for p in particles if p.weight > keepThreashold]
        numkeep = len(keep_particles)
        print("keep=%d"%(numkeep), end=" ")
        # The object is to keep particles with weights above keepThreshold
        dump_particles = [p for p in particles if p.weight <= keepThreashold]
        numdump = len(dump_particles)
        print("dump=%d"%(numdump), end=" ")
        if numkeep == 0:
            # if we do not have any good hypothesus (particles) we need to guess
            for d in dump_particles:
                i = int(random.random() * (len(self.arena.valid_regions)-1))
                p =self.arena.valid_regions[i]
                self.placeByRegion(d, p[0], p[1], 75)
        else:
            # Place low weight particles next to high weight particles
            keep = 0
            dump = 0
            r = 0
            # Redistribute particles with weights below keepThreshold
            # next to particles with weights above keepThreshold
            for relocate_particle in dump_particles:
                if keep < numkeep:
                    # for each keep particle assign X dump particles
                    x,y,w = self.placeNextTo(keep_particles[keep], relocate_particle)
                    if dump == 0: print("(%d,%d,%3f)"%(x,y,w), end="")
                    dump += 1
                    # We assigne 10% of the dump particles to each keep particle
                    if dump >= int((numdump+numkeep)*0.1):
                        keep += 1
                        print("=%d"%(dump), end=" ")
                        dump = 0
                else:
                    # We have assigne 10 dump particles to each keep particle but we have
                    # extra dump particles
                    k = 0
        return False

    def maxWeight(self):
        wmax = 0.0
        for p in self.particles:
            if not p.this_is_a_robot():
                if p.weight > wmax:
                    wmax = p.weight
        return wmax

    def avgWeight(self):
        avg = 0.0
        n  = 0
        for p in self.particles:
            if not p.this_is_a_robot():
                avg += p.weight
                n += 1
        return avg / n

    def avgXY(self):
        x = 0.0
        y = 0.0
        n = 0
        for p in self.particles:
            if not p.this_is_a_robot():
                x += (p.weight * p.x)
                y += (p.weight * p.y)
                n += 1
        return int(x/n), int(y/n)

