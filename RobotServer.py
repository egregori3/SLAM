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
import lcm
import LidarBotModel as model
from sensorlcm import sensors_t

class RobotServer:
    def __init__(self, parms):
        self.arena                   = parms['arena']
        parms['robotLidarData']      = [0] * parms['lidarSamples']  # List of samples representing distances
        self.robot_on                = False
        if 'lcmChannel' in parms:
            self.get_robot_data      = self.__getDataFromRealRobot
            print("Listening on LCM channel: "+parms['lcmChannel'])
            self.lc = lcm.LCM()
            subscription = self.lc.subscribe(parms['lcmChannel'], self.__lcmSensorHandler)
        else:
            print("Using simulated robot")
            self.turn_offset         = parms['robotTurnOffset']
            self.get_robot_data      = self.__getDataFromSimulatedRobot

    def start(self, parameters):
        return 0

    def __lcmSensorHandler(self, channel, data):
        msg = sensors_t.decode(data)
        print("Received message on channel \"%s\"" % channel)
        self.robot_heading  = msg.orientation
        self.robot_distance = msg.distance
        self.robot_lidar    = msg.ranges
        self.robot_msg_id   = msg.id

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

    def __getDataFromRealRobot(self, parms, dtext):
        self.lc.handle()
        print("Waiting for data from robot")
        parms['robotHeading']    = self.robot_heading
        parms['robotDistance']   = self.robot_distance
        parms['robotValidLidar'] = self.robot_lidar
        dtext.append(("id=%d" % (self.robot_msg_id), True)) # Display to console and overlay
        if self.robot_on == False:
            self.robot_on = True
            return 'init'
        else:
            return 'update'

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
            valid                           = self.arena.readLidar(x, y, lidar_data, 4)
            if lidar_data[0] <= 1:
                dtext.append(("New heading", False))
                valid                       = self.arena.readLidar(x, y, lidar_data)
                self.__setNewHeading(parms, lidar_data, update_dist, dtext)
            # try to move
            for step in range(update_dist):
                tx = x + step * math.cos(parms['robotHeading'])
                ty = y + step * math.sin(parms['robotHeading'])
                valid = self.arena.readLidar(tx, ty, lidar_data, 4)
                if lidar_data[0] <= 1:
                    break
            x = tx
            y = ty
            valid                           = self.arena.readLidar(x, y, lidar_data)
            parms['robotDistance']          = int(round(random.normalvariate(step, \
                                                parms['distanceSigmaNoise'])))
            dtext.append(("sr=%d,%d" % (int(x), int(y)), True)) # Display to console and overlay
            parms['simulatedRobotPath'].append((int(x), int(y)))
            parms['robotValidLidar']        = self.arena.readLidar(int(x), int(y), \
                                                parms['robotLidarData'])
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
        parms['robotHeading'] = random.normalvariate(heading,parms['headingSigmaNoise'])

    # Truncate angle - from robot class
    def __angletrunc(self,a):
        return (a+(2*math.pi)) % (2*math.pi)
