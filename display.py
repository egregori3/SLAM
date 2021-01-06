# Written by Eric Gregori

import cv2
import helpers

#==============================================================================
#
# display - Use openCV to display the data in files
#
#==============================================================================
class Display:

    def __init__(self, image, robot_path):
        self.__image = image.copy()
        self.AddLine(robot_path)

    def AddLine(self, points):
        for i in range(len(points)-1):
            cv2.line(self.__image,(points[i][0],points[i][1]),(points[i+1][0],points[i+1][1]),(128,128,128),2)

    def AddParticles(self, pf, wmax):
        for p in pf.particles:
            if p.this_is_a_robot:
                color = (255,0,0)
            else:
                color = (0,0,255)
                if wmax > 0:
                    color = helpers.colorBetween((0,0,255), (0,255,0), p.weight/wmax)
            cv2.circle(self.__image, (int(p.x), int(p.y)), 4, color, -1)

    def SaveImage(self,filename):
        cv2.imwrite(filename, self.__image)
