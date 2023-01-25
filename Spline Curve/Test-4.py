import sys
import matplotlib
matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from matplotlib.path import Path
from matplotlib.patches import PathPatch

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot()
        super(MplCanvas, self).__init__(fig)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        sc = MplCanvas(self, width=6, height=5, dpi=100)
        self.setCentralWidget(sc)
        self.Splinecurve()

        sc.axes.plot(self.out[0], self.out[1],color='orange')
        sc.axes.plot(self.data[0,:], self.data[1,:], 'ob', markerSize=10)

        err = 1
        self.draw_error_band(self.out[0], self.out[1], err)
        
        sc.axes.add_patch(PathPatch(self.path, facecolor="gray", edgecolor="none", alpha=.1))

        #self.draw_error_band(sc.axes.plot, self.out[0], self.out[1], err,
        #            facecolor="gray", edgecolor="none", alpha=.1)
        
        self.show()

    def Splinecurve(self):

        x = (0, 6, -5, 8, -3, 0)
        y = (0, 8, -7, -9, 10, 0)

        self.data = np.array((x,y))
        tck,u = interpolate.splprep(self.data, s=0)

        unew = np.arange(0, 1.01, 0.01)

        self.out = interpolate.splev(unew, tck)

    def draw_error_band(self, x, y, err):#, **kwargs):
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
        self.path = Path(vertices, codes)
        #ax.add_patch(PathPatch(path, **kwargs))


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
app.exec_()
