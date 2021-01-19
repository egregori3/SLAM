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

import cv2
import LidarBotModel as model
import algorithm1 as alg

#==============================================================================
#
# display - Use openCV to display the data in files
#
#==============================================================================
class Display:

    def __init__(self, parms, p_or_pf, filename, text_list, point):
        self.text_list      = text_list
        self.robot_color    = (255,255,0)
        self.textx          = parms['displayTextX']
        self.texty          = parms['displayTextY']
        self.__image        = parms['arena'].GetImage().copy()
        self.__addText()
        if type(p_or_pf) == alg.ParticleFilter:
            # We were passed a particle filter object
            self.__addParticles(p_or_pf, point)
        else:
            # We were passed a robot object
            self.__addRobot(p_or_pf)

        self.__addLine(parms['robotPath'])
        self.__saveImage(filename)

    def __colorMap(self, weight):
        if weight >= 0.9: return (0,255,0) # Green
        if weight < 0.5: return (0,0,255)  # Red
        return (0,255,255) # Yellow

    def __addLine(self, points):
        if len(points) > 1:
            for i in range(len(points)-1):
                cv2.line(self.__image,(points[i][0],points[i][1]),(points[i+1][0],points[i+1][1]),(128,128,128),2)

    def __addRobot(self, particle):
        cv2.circle(self.__image, (particle.x, particle.y), 6, self.robot_color, -1)

    def __addParticles(self, pf, point):
        # The robot mUST be the first particle because it needs to move first
        for p in pf.particles:
            if not p.this_is_a_robot():
                color = self.__colorMap(p.weight)
                cv2.circle(self.__image, (p.x, p.y), 4, color, -1)
        # Plot robot last so that it is on top of the particles
        self.__addRobot(pf.particles[0])
        if point[2] > 0:
            cv2.circle(self.__image, (point[0], point[1]), point[2], (255,0,255), 1)

    def __addText(self,):
        text = ""
        for field in self.text_list:
            text += (field+" ")
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (self.textx, self.texty)
        fontScale = 1
        color = (196, 196, 196) 
        thickness = 1
        image = cv2.putText(self.__image, text, org, font,
                   fontScale, color, thickness, cv2.LINE_AA) 

    def __saveImage(self,filename):
        cv2.imwrite(filename+".jpg", self.__image)
