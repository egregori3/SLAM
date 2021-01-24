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
        parms['robotLidarData']      = [0] * parms['lidarSamples']  # List of samples representing distances
        if 'serverIP' in parms:
            self.get_robot_data      = self.__getDataFromRealRobot
            print("Connecting to server: "+parms['serverIP'])
        else:
            print("Using simulated robot")
            self.turn_offset         = parms['robotTurnOffset']
            self.get_robot_data      = self.__getDataFromSimulatedRobot
            self.robot_on            = False

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

    def __getDataFromSimulatedRobot(self, parms, dtext):
        if self.robot_on == False:
            #==============================================================================
            # For simulation we need to place a robot in the arena in order to get the
            # Lidar data. 
            #==============================================================================
            # 1) At powerup, the robot connects to the server
            # 2) The robot does a Lidar scan and reads its magnetometer
            # 3) The robot sends the "init" command followed by the magnetometer and Lidar data
            robot_heading  = 0
            if 'robotX' not in parms:
                x, y = random.choice(parms['arena'].valid_regions)
                robot_heading = random.random()*(2 * math.pi)
            else:
                try:
                    x                       = parms['robotX']
                    y                       = parms['robotY']
                    robot_heading           = parms['robotH']
                except KeyError:
                    print("!ERROR! You must specify y and h with x")
            print("Simulated robot initial state (x = %d, y = %d, h = %f)"%(x, y, robot_heading))
            self.robot_on                   = True
            status                          = 0
            parms['robotHeading']           = robot_heading
            parms['robotValidLidar']        = self.arena.readLidar(x, y, parms['robotLidarData'])
            parms['simulatedRobotPath']     = [(x,y)]
            dtext.append(("init r=%d,%d" % (x, y), True)) # Display to console and overlay
            return 'init'
        else:
            # send "update" data
            # Since we do no have a physical robot, we use a particle to simulate
            # the robot.
            # 4) The robot moves distance x in direction z then takes a Lidar reading
            update_dist                     = parms['robotDistBetUpdates']
            lidar_data                      = parms['robotLidarData']
            x,y                             = parms['simulatedRobotPath'][-1]
            valid                           = self.arena.readLidar(x, y, lidar_data)
            if lidar_data[0] < update_dist/2:
                dtext.append(("New heading", False))
                self.__setNewHeading(parms, lidar_data, update_dist, dtext)
            valid                           = self.arena.readLidar(x, y, lidar_data)
            step = min(lidar_data[0]-3, update_dist)
            if step < 0: raise Exception("Negative Step")
            x += step * math.cos(parms['robotHeading'])
            y += step * math.sin(parms['robotHeading'])
            parms['robotDistance']          = step
            dtext.append(("sr=%d,%d" % (int(x), int(y)), True)) # Display to console and overlay
            parms['simulatedRobotPath'].append((int(x), int(y)))
            parms['robotValidLidar']        = self.arena.readLidar(int(x), int(y), parms['robotLidarData'])
            return 'update'

    def __setNewHeading(self, parms, lidar_data, update_dist, dtext):
        heading = self.__angletrunc(parms['robotHeading'] + math.pi) 
        turn_angle = (2*math.pi)/len(lidar_data)
        for i in range(int(len(lidar_data)/2)):
            if lidar_data[i] > update_dist:
                heading = self.__angletrunc(parms['robotHeading'] + (2+i)*turn_angle)
                break
            if lidar_data[-i] > update_dist:
                heading = self.__angletrunc(parms['robotHeading'] - (2+i)*turn_angle)
                break
        parms['robotHeading'] = heading

    # Truncate angle - from robot class
    def __angletrunc(self,a):
        return (a+(2*math.pi)) % (2*math.pi)
