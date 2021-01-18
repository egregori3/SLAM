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

import random
import LidarBotModel as model
import math

#==============================================================================
#
# Particle Filter
#
#==============================================================================
class ParticleFilter:
    def __init__(self, parms, robot):
        self.parms              = parms
        self.meas               = []
        self.particles          = []
        self.robot              = robot
        self.width              = parms['arenaWidth']
        self.height             = parms['arenaHeight']
        self.arena              = parms['arena']
        self.numb_of_particles  = parms['numberOfParticles']
        self.lidar_max_dist     = parms['lidarMaxDistance']
        self.init_weight_thres  = parms['initialWeightThres']

        self.particles.append(robot) # The robot MUST move first
        self.__resetParticles()

    def __resetParticles(self):
        # For each valid region calculate a weight.
        print("Scanning valid regions")
        tester = model.Particle(self.parms, robot_object=self.robot)
        xy_list = self.__getXYByWeight(tester, self.init_weight_thres)
        print("Placing initial particles - weight > "+str(self.init_weight_thres))
        if self.numb_of_particles == 0:
            print("Number of particles set by algorithm")
            i = 0
            for (x,y) in xy_list:
                p = model.Particle(self.parms, myid=i, robot_object=self.robot)
                i += 1
                self.__placePosition(p,x,y)
                self.particles.append(p)
                print("*", end="", flush=True)
        print()
        print("Placed %d particles"%len(self.particles))

    def __placePosition(self,p,x,y):
        weight = p.place({'setPosition':(x,y,0)})
        return weight

    # Returns list (x,y) tuples based on weight
    def __getXYByWeight(self, particle, threshold):
        return [(particle.x,particle.y) \
            for position in self.arena.valid_regions \
                if self.__placePosition(particle, position[0], position[1]) > threshold]

    # Computes distance between point1 and point2. Points are (x, y) pairs.
    def __DistanceBetween(self, point1, point2):
        (x1, y1) = point1
        (x2, y2) = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Place the relocate particle in a random location around the keep particle
    # within distance.
    def __placeNextTo(self, keep_particle, relocate_particle, check_map, dist):
        # Place the relocate particle within distance from x,y
        # Width, Height, x, y 
        dist = int(dist)
        for x in range(dist):
            for y in range(dist):
                if self.arena.CheckXY(x,y):
                    check_map[y][x] = True

        for tries in range(dist*dist):
            x = keep_particle.x + int((random.random()*dist) - dist/2)
            y = keep_particle.y + int((random.random()*dist) - dist/2)
            x = max(0,x)
            y = max(0,y)
            x = min(x,self.arena.width-1)
            y = min(y,self.arena.height-1)
            if check_map[y][x] == False:
                if self.__placePosition(relocate_particle,x,y) >= 0.5:
                    check_map[y][x] = True
                    return True
        return False # We could not place the particle

    def pfData(self, keep=0, done=False):
        nump = len(self.particles)
        avgw = self.__avgWeight()
        return (done, nump, avgw, keep)

    # move particles
    # return True to end simulation
    def update(self):
        # move particles
        for p in self.particles:
            p.move()

        # If the robot is in a deadzone, we do not have enough data to 
        # make a decision on how to place particles
        if not self.robot.valid_weight(): return self.pfData()

        # Create list of all particles (not the robot)
        particles = [p for p in self.particles if not p.this_is_a_robot()]

        # refresh dead particles
        keepThreashold = 0.9
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
            self.__resetParticles()
        else:
            # Place low weight particles next to high weight particles
            keep = 0
            dump = 0
            r = 0
            check_map = [[False for c in range(self.width)] for r in range(self.height)]
            for p in keep_particles:
                check_map[p.y][p.x] = True
            # Redistribute particles with weights below keepThreshold
            # next to particles with weights above keepThreshold
            for relocate_particle in dump_particles:
                if keep < numkeep:
                    # for each keep particle assign X dump particles
                    if self.__placeNextTo(keep_particles[keep], relocate_particle, check_map, \
                            self.lidar_max_dist/2) == False:
                        self.particles.remove(relocate_particle)
                    if dump == 0:
                        x = keep_particles[keep].x
                        y = keep_particles[keep].y
                        w = keep_particles[keep].weight
                        print("(%d,%d,%3f)"%(x,y,w), end="")
                    dump += 1
                    # We assigne 10% of the dump particles to each keep particle
                    if dump >= int(numdump/numkeep):
                        keep += 1
                        print("=%d"%(dump), end=" ")
                        dump = 0
                else:
                    # We have assigne 10 dump particles to each keep particle but we have
                    # extra dump particles
                    k = 0
        return self.pfData(keep=len(keep_particles))

    def __avgWeight(self):
        avg = 0.0
        n  = 0
        for p in self.particles:
            if not p.this_is_a_robot():
                avg += p.weight
                n += 1
        return avg / n

    def __avgXY(self):
        x = 0.0
        y = 0.0
        n = 0
        for p in self.particles:
            if not p.this_is_a_robot():
                x += (p.weight * p.x)
                y += (p.weight * p.y)
                n += 1
        return int(x/n), int(y/n)

#
# These functions do not use valid regions and are deprecated
#
    # deprecated
    def __DEPRECATED_placeByRegion(self, particle, x, y, thres):
        cmd = {'constrainedRandom':(x,y,50,50)}
        particle.place(cmd)
        if particle.weight > thres:
            return True
        return False

    # deprecated
    def __DEPRECATED_placeByWeight(self, particle, thres):
        particle.place({})  # Place randomly
        weight = particle.weight
        if weight > thres:
            return True
        return False

    # deprecated
    def __DEPRECATED_placeNextTo(self, keep_particle, relocate_particle):
        x=keep_particle.x
        y=keep_particle.y
        w=keep_particle.weight
        # x,y is the location of the keep particle
        # Place the dump particle in a random location around the keep particle
        data = (x, y, 50, 50)
        relocate_particle.place({'constrainedRandom':data})
        return x,y,w


