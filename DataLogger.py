# Experiments in robot localization using particle filters (algorithm and simulator)
# Copyright (C) <2021>  Eric Gregori

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

import matplotlib.pyplot as plt

class DataLogger:

    def startDataLogger(self, parms):
        self.fp = open(self.parms[dataFilename], "w")
        fp.write("rx, ry, ")

    def plotSamples(self, parms, filename):
        pf = parms['pfObject']
        filename = filename+".jpg"
        fig, ax = plt.subplots(facecolor=(.18, .31, .31))
        ax.set_facecolor('#eafff5')
        ax.set_xlabel('Sample', color='c')
        ax.set_ylabel('Distance', color='peachpuff')
        t = range(len(pf.particles[0].samples))
        ax.set_title("LIDAR Samples", color='0.7')
        for s in pf.particles:
            ax.plot(t, s.samples, label=s.text)
        ax.plot(t, parms['robotLidarData'], label="Robot")
        ax.tick_params(labelcolor='tab:orange')
        plt.legend()
        plt.savefig(filename)
