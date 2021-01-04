# Written by Eric Gregori
from math import *

#==============================================================================
#
# Helper Functions
#
#==============================================================================
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_trunc(a):
        while a < 0.0:
                a += pi * 2
        return ((a + pi) % (pi * 2)) - pi

def get_heading(prev_pos, curr_pos):
    prev_pos_x = prev_pos[0]
    prev_pos_y = prev_pos[1]
    curr_pos_x = curr_pos[0]
    curr_pos_y = curr_pos[1]
    heading = atan2(curr_pos_y - prev_pos_y, curr_pos_x - prev_pos_x)
    heading = angle_trunc(heading)
    return heading

def lowPassFilter(measurements):
        output = []
        for i in range(4,len(measurements)):
                x = measurements[i-3][0]+measurements[i-2][0]+measurements[i-1][0]+measurements[i][0]
                y = measurements[i-3][1]+measurements[i-2][1]+measurements[i-1][1]+measurements[i][1]
                output.append([x/4.0,y/4.0])       
        return(output)        

def colorBetween(startColor, endColor, percentage):
    percentage = min(1., max(0., percentage)) # limit percentage to 0-1 
    ib = int(endColor[0] * percentage + startColor[0] * (1 - percentage))
    ig = int(endColor[1] * percentage + startColor[1] * (1 - percentage))
    ir = int(endColor[2] * percentage + startColor[2] * (1 - percentage))
    return (ib,ig,ir)

def error(l1, l2):
    return sum((c - a)**2 + (d - b)**2 for ((a, b), (c, d)) in zip(l1, l2))**0.5


