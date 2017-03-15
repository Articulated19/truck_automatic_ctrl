import numpy as np
from scipy import interpolate


def spline(positions): #positions = [(x1, y1), (x2, y2), (x3, y3)]
    if len(positions) <= 3:
        return positions
    
    print "spline: positions = ", positions
    
    xs,ys = zip(*positions)
    xs = np.array(xs)
    ys = np.array(ys)
    
    
    tck, u = interpolate.splprep([xs, ys], s=0, per=False)
    xi, yi = interpolate.splev(np.linspace(0, 1, len(xs) * 7), tck)
    return zip(xi,yi)
    
    
