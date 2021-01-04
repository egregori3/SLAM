# Written by Eric GRegori

import cv2

#==============================================================================
#
# arena - Define position contraints
#
#==============================================================================
class Arena:

    def __init__(self, parameters):
        print("Loading arena: ", parameters['arenaFilename'])
        self.__image = cv2.imread(parameters['arenaFilename'])
        try:
            height, width, channels = self.__image.shape
            parameters['arenaHeight'] = height
            parameters['arenaWidth']  = width
            print("height = %d, width = %d, channels = %d" % (height, width, channels))
        except Exception as e:
            print("Failure loading arena: "+e)

    def GetImage(self): return self.__image

    # If True than collision
    def CheckXY(self, x, y):
        pos = self.__image[int(y),int(x)]
        if pos[0] == 255 and pos[1] == 255 and pos[2] == 255:
            return True
        return False


