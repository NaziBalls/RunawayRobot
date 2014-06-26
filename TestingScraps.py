__author__ = 'nagpalk'
from math import *

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    x3 = measurement[0]
    y3 = measurement[1]
    x1 = OTHER[0][0]
    y1 = OTHER[0][1]
    x2 = OTHER[1][0]
    y2= OTHER[1][1]

    if(x2-x1)==0:
        HA1 = 0;
    else:
        HA1 = atan( (y2-y1)/(x2-x1) )

    if (x2<x1):
        HA1 += pi

    if (x3-x2)==0:
        HA2 = 0;
    else:
        HA2 = atan( (y3-y2)/(x3-x2) )
    if (x3<x2):
        HA2 += pi

    delHA = HA2 - HA1

    HA3 = (HA2+delHA)%(2*pi)

    print(HA1, HA2, HA3)
    #r = distance_between(measurement, OTHER[1])

    #xest = x3 + r*cos(HA3)
    #yest = y3 + r*sin(HA3)

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #return xy_estimate, OTHER


measurement = [1+sqrt(2),sqrt(2)]
OTHER = [ [0,0], [sqrt(2),sqrt(2)]]
estimate_next_pos(measurement,OTHER)
