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
            self.width = width
            self.height = height
            parameters['arenaHeight'] = height
            parameters['arenaWidth']  = width
            self.region = parameters['arenaRegion']
            self.valid_regions = self.placeByPosition()
            print("height = %d, width = %d, channels = %d" % (height, width, channels))
        except Exception as e:
            print("Failure loading arena: "+e)

    def GetImage(self): return self.__image

    # If True than collision
    def CheckXY(self, x, y):
        if x >= self.width: return  True
        if y >= self.height: return True
        pos = self.__image[int(y),int(x)]
        if pos[0] == 255 and pos[1] == 255 and pos[2] == 255:
            return True
        return False

    # If any portion of the region is valid, add it to self.valid_regions
    def placeByPosition(self):
        i = 0
        x = 0
        y = 0
        valid_regions = []
        while(1):
            ul = self.CheckXY(x,y)
            ll = self.CheckXY(x,y+self.region-1)
            ur = self.CheckXY(x+self.region-1,y)
            lr = self.CheckXY(x+self.region-1,y+self.region-1)
            if not (ul and ll and ur and lr):
                valid_regions.append((x+int(self.region/2),y+int(self.region/2)))
                print(x+int(self.region/2),y+int(self.region/2))
            x = int((i * 50) % self.width)
            y = int((i * 50) / self.width)*50
            i += 1
            if y >= self.height: break
        return valid_regions


