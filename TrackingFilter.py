import math
import random
import LidarBotModel as model
import Algorithm as alg

#==============================================================================
# Tracking Filter
#==============================================================================
def particleFilter(parameters, state, dtext):
    prediction = (0,0,0)
    if state == 'init':
        # Place initial particles
        parameters['pfObject'] = alg.ParticleFilter(parameters)
        dtext.append(("Place particles", True)) # Display to console and overlay
        (done, nump, avgw, keep, prediction) = parameters['pfObject'].pfData()
    else:
        (done, nump, avgw, keep, prediction) = parameters['pfObject'].update(parameters, dtext)
        dtext.append(("p=%d,%d"%(nump,keep), True))
        dtext.append(("wavg=%2f"%avgw, True))
        if prediction[2] > 0:
            dtext.append(("L=%d,%d"%(prediction[0],prediction[1]), True))
    return prediction
