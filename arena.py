# Written by Eric GRegori

import cv2
import math

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
            print("height = %d, width = %d, channels = %d" % (height, width, channels))
        except Exception as e:
            print("Failure loading arena: "+e)
        #  (0,0)1 2(?,0)
        #  (0,?)4 8(?,?)
                               # 1 = open, 0 = collision
        self.cad = [( -1,-1 ), # 0000 - do not place
                    (  0,0  ), # 0001 - put into 1
                    (  1,0  ), # 0010 - put into 2
                    (0.5,0  ), # 0011 - between 1-2
                    (  0,1  ), # 0100 - put into 4
                    (  0,0.5), # 0101 - between 1-4
                    ( -1,-1 ), # 0110 - do not place
                    (  0,0  ), # 0111 - put into 1
                    (  1,1  ), # 1000 - put into 8
                    ( -1,-1 ), # 1001 - do not place
                    ( -1,-1 ), # 1010 - do not place
                    (  1,0  ), # 1011 - put into 2
                    (0.5,1  ), # 1100 - between 4-8
                    (  0,1  ), # 1101 - put into 4
                    (  1,1  ), # 1110 - put into 8
                    (0.5,0.5)] # 1111 - put into middle
        self.outer_scan = [self.__boundary(parameters['lidarMaxDistance'],angle) \
                for angle in range(0,360,10)]
        self.valid_regions = self.__getValidRegions()

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
    #  (0,0)1 2(?,0)
    #  (0,?)4 8(?,?)
    def __getValidRegions(self):
        print("Finding valid regions:", end=" ")
        i = 0
        x = 0
        y = 0
        valid_regions = []
        while(1):
            if not self.CheckXY(x,y): # center is open
                for offset in self.outer_scan:
                    xa = min(offset[0]+x, self.width)
                    ya = min(offset[1]+y, self.height)
                    if self.CheckXY(xa,ya): # Scan hit wall
                        cv2.circle(self.__image, (x, y), 1, (0,0,255), -1)
                        valid_regions.append((x, y))
            x = int((i * self.region) % self.width)
            y = int((i * self.region) / self.width)*self.region
            i += 1
            if y >= self.height: break
        print()
        return valid_regions

    def __boundary(self,d,heading):
            x = d * math.cos(heading*(2*math.pi/360))
            y = d * math.sin(heading*(2*math.pi/360))
            return (int(x), int(y))


