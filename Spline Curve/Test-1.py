import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from scipy.io import loadmat
import scipy.io

#x = (0,5,10,15,30,35,40,50,55,60)

#y = (100,90,65,85,70,30,40,45,20,0)

#x = (0, 0.06, -0.05, 0.08, -0.03, 0)
#y = (0, 0.08, -0.07, -0.09, 0.10, 0)

#x = (0, -0.08, 0.08, -0.03, -0.09, 0)
#y = (0, -0.09, -0.02,  0.075, -0.01, 0)

#path generated from matlab for test

#y = (0,	0.051,	-0.012,	0.097,	-0.09,	0)
#y = (0,	0.09,	0.012,	-0.068,	0.092,	0)
#y = (0,	-0.081,	0.072,	-0.029,	-0.097,	0)
#y = (0,	-0.085,	0.064,	0.095,	-0.070,	0)
#y = (0,	-0.089,	0.063,	-0.077,	-0.098,	0)
#y = (0,	-0.096,	-0.001,	-0.095,	0.06,	0)
#y = (0,	-0.095,	-0.054,	0.066,	-0.089,	0)

#x = (0,	0.085,	-0.033,	0.040,	-0.092,	0)
#x = (0,	-0.083,	0.069,	-0.046,	0.052,	0)
#x = (0,	0.081,	-0.027,	0.063,	-0.079,	0)
#x = (0,	0.078,	-0.077,	0.004,	0.095,	0)
#x = (0,	-0.078,	0.051,	-0.06,	0.061,	0)
#x = (0,	-0.085,	-0.019,	0.093,	-0.072,	0)
#x = (0,	0.10,	-0.054,	0.040,	-0.056,	0)

#path generated from matlab (real path)

#y = (0,	0.051,	-0.012,	0.097,	-0.09,	0)
# y = (0,	0.09,	0.012,	-0.068,	0.092,	0)
#y = (0,	-0.081,	0.072,	-0.029,	-0.097,	0)
#y = (0,	-0.085,	0.064,	0.095,	-0.070,	0)
#y = (0,	-0.085,	0.064,	0.095,	-0.070,	0)
#y = (0,	-0.085,	0.064,	0.095,	-0.070,	0)
#y = (0,	-0.095,	-0.054,	0.066,	-0.089,	0)
#y = (0,	-0.085,	0.064,	0.095,	-0.070,	0)

#x = (0,	-0.083,	0.069,	-0.046,	0.052,	0)
# x = (0,	-0.083,	0.069,	-0.046,	0.052,	0)
#x = (0,	0.081,	-0.027,	0.063,	-0.079,	0)
#x = (0,	0.078,	-0.077,	0.004,	0.095,	0)
#x = (0,	-0.078,	0.051,	-0.06,	0.061,	0)
#x = (0,	-0.085,	-0.019,	0.093,	-0.072,	0)
#x = (0,	-0.083,	0.069,	-0.046,	0.052,	0)
#x = (0,	-0.083,	0.069,	-0.046,	0.052,	0)

#--- Final Path-----
# x = (0, 0.06, -0.05, 0.08, -0.01, 0)
x = (0, 0.057, -0.022, 0.031, -0.09, 0)
# x = (0, 0.096, 0.002, -0.07, 0.034, 0)
# x = (0, 0.073, 0.021, -0.04, 0.087, 0)
# x = (0, -0.096, -0.002, 0.07, -0.034, 0)
# x = (0, 0.092, -0.054, 0.004, 0.054, 0)
# x = (0, -0.064, -0.011, 0.047, -0.089, 0)
# x = (0, -0.099, -0.036, -0.086, 0.051, 0)


# y = (0, 0.08, -0.07, -0.09, 0.08, 0)
y = (0, 0.098, 0.085, -0.10, 0.052, 0)
# y = (0, 0.084, -0.038, -0.069, 0.097, 0)
# y = (0, 0.056, -0.096, 0.087, 0.098, 0)
# y = (0, 0.082, -0.07, -0.061, 0.098, 0)
# y = (0, 0.068, -0.058, 0.084, -0.04, 0)
# y = (0, 0.056, -0.096, 0.087, 0.098, 0)
# y = (0, 0.082, -0.07, -0.061, 0.098, 0)

#plt.plot(x, y, 'o')

#plt.show()

data = np.array((x,y))
tck,u = interpolate.splprep(data, s=0)
#print(tck)


unew = np.arange(0, 1.0001, 0.0001)

out = interpolate.splev(unew, tck)

fig, ax = plt.subplots()

ax.plot(out[0], out[1], color='orange')


ax.plot(data[0,:], data[1,:], 'ob')
print(fig)

#plt.show()

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


draw_error_band(ax, out[0], out[1], err=0.01,
                    facecolor="gray", edgecolor="none", alpha=.1)

#scipy.io.savemat('/home/fzahedi1/git/NewStudy/Spline Curve/lineout5.mat', {'lineout5':out})
#scipy.io.savemat('/home/fzahedi1/git/NewStudy/Spline Curve/target5.mat', {'target5':data})


plt.xlim([-0.12, 0.12])
plt.ylim([-0.12, 0.12])
plt.xlabel("x (cm)")
plt.ylabel("y (cm)")
plt.show()


