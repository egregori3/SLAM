
class DataLogger:

    def __init__(self, parms):
        self.fp = open(parms[dataFilename], "w")
        fp.write("rx, ry, ")
