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
import math
import LidarBotModel as model

class RobotServer:
    def __init__(self, parms):
        self.arena                   = parms['arena']
        parameters['robotLidarData'] = [0] * parms['lidarSamples']  # List of samples representing distances
        parms['robotPath']           = []
        if 'serverIP' in parms:
            self.get_robot_data = self.__getDataFromRealRobot
            print("Connecting to server: "+parms['serverIP'])
        else:
            print("Using simulated robot")
            self.get_robot_data = self.__getDataFromSimulatedRobot
            self.robot_on = False

    def start(self, parameters):
        return 0

    #==============================================================================
    #
    # Get data from robot
    #
    # 1) At powerup, the robot connects to the server
    # 2) The robot does a Lidar scan and reads its magnetometer
    # 3) The robot sends the "init" command followed by the magnetometer and Lidar data
    # 4) The robot moves distance x in direction z then takes a Lidar reading
    # 5) The robot sends the "update" command followed by x, z, and the Lidar data
    # 6) Goto 4
    #
    # If the robot hits an obstruction in step 4, it should stop, send an update,
    # then turn to a new heading and goto step 4.
    #==============================================================================
    def getDataFromRobot(self, parameters, dtext):
        return self.get_robot_data(parameters, dtext)

    def __getDataFromRealRobot(self, parameters, dtext):
        pass

    def __getDataFromSimulatedRobot(self, parameters, dtext):
        if self.robot_on == False:
            #==============================================================================
            # For simulation we need to place a robot in the arena in order to get the
            # Lidar data. 
            #==============================================================================
            # 1) At powerup, the robot connects to the server
            # 2) The robot does a Lidar scan and reads its magnetometer
            # 3) The robot sends the "init" command followed by the magnetometer and Lidar data
            robot_object   = model.Particle(parms, 'Simulated Robot')
            robot_heading  = 0
            if 'robotX' not in parms:
                x, y = random.choice(parms['arena'].valid_regions)
                robot_heading = random.random()*(2 * math.pi)
                self.robot.place(x, y, robot_heading)
            else:
                try:
                    self.robot.place(parms['robotX'], parms['robotY'], parms['robotH'])
                    robot_heading = parms['robotH']
                except KeyError:
                    print("!ERROR! You must specify y and h with x")
            print("Simulated robot initial state (x = %d, y = %d, h = %f)"%(self.robot.x, self.robot.y, robot_heading))
            parms['robotObject'] = robot_object
            parms['robotPath'].append((self.robot.x, self.robot.y))
            self.robot_on                   = True
            status                          = 0
            parameters['robotHeading']      = robot_heading
            parameters['robotLidarData']    = self.arena.readLidar(self.robot.x, self.robot.y, \
                                                parameters['robotHeading'], parameters['robotLidarData'])
            dtext.append("init r=%d,%d" % (self.robot.x, self.robot.y))
            return 'init'
        else:
            # send "update" data
            # Since we do no have a physical robot, we use a particle to simulate
            # the robot.
            # 4) The robot moves distance x in direction z then takes a Lidar reading
            update_dist    = parms['robotDistBetUpdates']
            turn_angle     = parms['robotTurnAngle']
            heading = parameters['robotHeading']
            lidar_samples = parameters['robotLidarData']
            collision = False
            for step in range(update_dist):
                lidar_data = self.arena.readLidar(self.robot.x, self.robot.y, heading, lidar_samples)
                if min(lidar_data[:turn_angle]) > 5 and min(lidar_data[-turn_angle:]) > 5:
                    self.robot.x += math.cos(heading)
                    self.robot.y += math.sin(heading)
                else:
                    collision = True
                    break

            self.robot.x = int(self.robot.x)
            self.robot.y = int(self.robot.y)
            parameters['robotDistance'] = step
            dtext.append("sr=%d,%d" % (self.robot.x, self.robot.y))
            parameters['robotPath'].append((self.robot.x, self.robot.y))
            lidar_data = self.arena.readLidar(self.robot.x, self.robot.y, heading, lidar_samples)
            # If the robot hits an obstruction in step 4, it should stop, send an update,
            # then turn to a new heading and goto step 4.
            if collision:
                heading = self.__setNewHeading(heading, lidar_data, dtext)
            parameters['robotLidarData'] = lidar_data
            parameters['robotHeading']   = heading
            return 'update'

    def __setNewHeading(self, heading, lidar_data, dtext):
        samples = len(lidar_data)
        turn_angle = (math.pi/8)+random.random()*(math.pi/2)
        left  = max(lidar_data[-1 * self.turn_angle:])
        right = max(lidar_data[:self.turn_angle])
        if left == right:
            heading = self.__angletrunc(turn_angle + random.random()*math.pi)
        elif left > right:
            heading = self.__angletrunc(heading-turn_angle)
        else:
            heading = self.__angletrunc(heading+turn_angle)
        return heading

    # Truncate angle - from robot class
    def __angletrunc(self,a):
        return (a+(2*math.pi)) % (2*math.pi)
