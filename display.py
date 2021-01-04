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
        self.AddRobot(robot_path[-1])

    def AddLine(self, points):
        for i in range(len(points)-1):
            cv2.line(self.__image,(points[i][0],points[i][1]),(points[i+1][0],points[i+1][1]),(128,128,128),2)

    def AddRobot(self,point):
        cv2.circle(self.__image, point, 4, (255,0,0), -1)

    def AddParticles(self, pf, wmax):
        for p in pf.particles:
            print("display x=%d, y=%d"%(p.x, p.y))
            pxy = p.getXY()
            print("pxy: "+str(pxy))
            color = (0,0,255)
            if wmax > 0:
                color = helpers.colorBetween((0,0,255), (0,255,0), p.weight/wmax)
            cv2.circle(self.__image, (int(pxy[0]), int(pxy[1])), 4, color, -1)

    def SaveImage(self,filename):
        cv2.imwrite(filename, self.__image)
