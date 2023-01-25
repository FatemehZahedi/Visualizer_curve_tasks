from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from matplotlib.path import Path
from matplotlib.patches import PathPatch

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        self.graphWidget.setBackground('w')
        self.Scatter = pg.ScatterPlotItem(size=15)

        #hour = [1,2,3,4,5,6,7,8,9,10]
        #temperature = [30,32,34,32,33,31,29,32,35,45]
        self.Splinecurve()


        # plot data: x, y values
        #pen = pg.mkPen(color=(0, 0, 255), width=0, style=QtCore.Qt.NoLine)
        self.graphWidget.plot(self.out[0], self.out[1])
        #self.graphWidget.plot(self.data[0,:], self.data[1,:], pen=pen, symbol='o', symbolSize=15)
        self.Scatter.addPoints(self.data[0,:], self.data[1,:])
        self.graphWidget.addItem(self.Scatter)

        #err = 1

        #self.draw_error_band(self.graphWidget, self.out[0], self.out[1], err=1)

    def Splinecurve(self):

        x = (0, 6, -5, 8, -3, 0)
        y = (0, 8, -7, -9, 10, 0)

        self.data = np.array((x,y))
        tck,u = interpolate.splprep(self.data, s=0)

        unew = np.arange(0, 1.01, 0.01)

        self.out = interpolate.splev(unew, tck)

    def draw_error_band(ax, x, y, err, **kwargs):
        # Calculate normals via centered finite differences (except the first point
        # which uses a forward difference and the last point which uses a backward
        # difference).
        dx = np.concatenate([[x[1] - x[0]], x[2:] - x[:-2], [x[-1] - x[-2]]])
        dy = np.concatenate([[y[1] - y[0]], y[2:] - y[:-2], [y[-1] - y[-2]]])
        l = np.hypot(dx, dy)
        nx = dy / l
        ny = -dx / l

        # end points of errors
        xp = x + nx * err
        yp = y + ny * err
        xn = x - nx * err
        yn = y - ny * err

        vertices = np.block([[xp, xn[::-1]],
                            [yp, yn[::-1]]]).T
        codes = np.full(len(vertices), Path.LINETO)
        codes[0] = codes[len(xp)] = Path.MOVETO
        path = Path(vertices, codes)
        ax.add_patch(PathPatch(path, **kwargs))


def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()