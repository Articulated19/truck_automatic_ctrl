import numpy as np
from scipy import interpolate
from matplotlib import pyplot as plt
import sys

x = []
y = []

file = sys.argv[1]

with open(file,'r') as f:
    for line in f:
        line = line.replace("\n", "")
        l = line.split(" ")
        x.append(float(l[0]))
        y.append(float(l[1]))
    
    
# append the starting x,y coordinates
x.append(x[0])
y.append(y[0])
x = np.array(x)
y = np.array(y)

smoothness = float(sys.argv[2])

# fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
# is needed in order to force the spline fit to pass through all the input points.
tck, u = interpolate.splprep([x, y], s=smoothness, per=True)

# evaluate the spline fits for 1000 evenly spaced distance values
xi, yi = interpolate.splev(np.linspace(0, 1, 1000), tck)

with open(file + '_splined_' + str(smoothness),'w') as f:
    for xx,yy in zip(xi,yi):
        f.write(str(xx) + " " + str(yy) + "\n")

# plot the result
fig, ax = plt.subplots()
a = ax.plot(x, y, 'o')
b = ax.plot(x, y , '-b')
c = ax.plot(xi, yi, '-r')
plt.setp(b, linewidth=0.6)
plt.setp(c, linewidth=0.6)
plt.setp(a,c="black", ms=4, alpha=0.5)


plt.axis('equal')

plt.show()

