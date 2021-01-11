# Written by Eric Gregori

import cv2
import helpers

#==============================================================================
#
# display - Use openCV to display the data in files
#
#==============================================================================
class Display:

    def __init__(self, parameters, pf, wmax, filename):
        self.parms = parameters
        self.__image = parameters['arena'].GetImage().copy()
        self.addText()
        self.addParticles(pf, wmax)
        self.addLine(parameters['robotPath'])
        self.saveImage(filename)

    def addLine(self, points):
        if len(points) > 1:
            for i in range(len(points)-1):
                cv2.line(self.__image,(points[i][0],points[i][1]),(points[i+1][0],points[i+1][1]),(128,128,128),2)

    def addParticles(self, pf, wmax):
        # The robot mUST be the first particle because it needs to move first
        robot_color = (255,255,0)
        default_particle_color = (0,0,255)
        for p in pf.particles:
            if p.this_is_a_robot():
                color = robot_color
            else:
                color = default_particle_color
                if wmax > 0:
                    color = helpers.colorBetween((0,0,255), (0,255,0), p.weight/wmax)
            cv2.circle(self.__image, (int(p.x), int(p.y)), 4, color, -1)
        cv2.circle(self.__image, (int(pf.particles[0].x), int(pf.particles[0].y)), 6, robot_color, -1)

    def addText(self):
        text = ""
        for field in self.parms['displayStaticText']:
            text += (field+" ")
        for field in self.parms['displayDynamicText']:
            text += (field+" ")
        self.parms['displayDynamicText'] = list()
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (50, 50)
        fontScale = 1
        color = (196, 196, 196) 
        thickness = 1
        image = cv2.putText(self.__image, text, org, font,
                   fontScale, color, thickness, cv2.LINE_AA) 

    def saveImage(self,filename):
        cv2.imwrite(filename+".jpg", self.__image)
