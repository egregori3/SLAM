# Written by Eric Gregori

import cv2
import helpers

#==============================================================================
#
# display - Use openCV to display the data in files
#
#==============================================================================
class Display:

    def __init__(self, image, robot_path, text_array):
        self.__image = image.copy()
        self.addLine(robot_path)
        self.addText(text_array)

    def addLine(self, points):
        if len(points) > 1:
            for i in range(len(points)-1):
                cv2.line(self.__image,(points[i][0],points[i][1]),(points[i+1][0],points[i+1][1]),(128,128,128),2)

    def addParticles(self, pf, wmax):
        for p in pf.particles:
            if p.this_is_a_robot():
                color = (255,0,0)
            else:
                color = (0,0,255)
                if wmax > 0:
                    color = helpers.colorBetween((0,0,255), (0,255,0), p.weight/wmax)
            cv2.circle(self.__image, (int(p.x), int(p.y)), 4, color, -1)

    def addText(self, text_array):
        text = ""
        for field in text_array[0]: # Static text
            text += (field+" ")
        for field in text_array[1]: # Dynamic text
            text += (field+" ")
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (50, 50)
        fontScale = 1
        color = (255, 0, 0) 
        thickness = 2
        image = cv2.putText(self.__image, text, org, font,
                   fontScale, color, thickness, cv2.LINE_AA) 
        text_array[1] = "" # Reset dynamic text

    def saveImage(self,filename):
        cv2.imwrite(filename, self.__image)
