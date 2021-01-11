import matplotlib.pyplot as plt

class DataLogger:

    def __init__(self, parameters):
        self.parms = parameters

    def startDataLogger(self):
        self.fp = open(self.parms[dataFilename], "w")
        fp.write("rx, ry, ")

    def plotSamples(self, particle_list, filename):
        filename = filename+".jpg"
        fig, ax = plt.subplots(facecolor=(.18, .31, .31))
        ax.set_facecolor('#eafff5')
        ax.set_xlabel('Sample', color='c')
        ax.set_ylabel('Distance', color='peachpuff')
        t = range(len(particle_list[0].samples))
        ax.set_title("LIDAR Samples", color='0.7')
        for s in particle_list:
            ax.plot(t, s.samples, label=s.text)
        ax.tick_params(labelcolor='tab:orange')
        plt.legend()
        plt.savefig(filename)
