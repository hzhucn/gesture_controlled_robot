#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np

@np.vectorize
def get_ir_from_dist(d, min_dist, half_dist, max_dist, order):
    if(d <= min_dist):
        return 1023
    elif(d >= max_dist):
        return 0.0
    else:
        alpha = (half_dist - min_dist) / (np.exp(-np.log(0.5)/order) - 1.0)
        return 1023. / ((d - min_dist)/alpha + 1)**order

@np.vectorize
def get_dist_from_ir(ir, min_dist, half_dist, max_dist, order):
    if(ir >= 1023):
        return min_dist
    else:
        alpha = (half_dist - min_dist) / (np.exp(-np.log(0.5)/order) - 1.0)
        d = (np.exp(1.0/order * np.log(1023./ir)) - 1) * alpha + min_dist
        if(d >= max_dist):
            return np.nan
        else:
            return d
        

min_dist = 50 # mm
half_dist = 75 # mm, where the response is 512
max_dist = 250 # mm
order = 5

dvalues = np.linspace(0, 300, 1000) # mm
IR_value = get_ir_from_dist(dvalues, min_dist, half_dist, max_dist, order)

IR_values = np.linspace(0.01, 1023, 1000)
distances = get_dist_from_ir(IR_values, min_dist, half_dist, max_dist, order)

plt.figure()
plt.subplot(1,2,1)
plt.plot(dvalues, IR_value)
plt.xlabel("distance to the wall (mm)")
plt.ylabel("Measured value")
plt.ylim([0, 1024])
plt.subplot(1,2,2)
plt.plot(IR_values, distances)
plt.xlabel("Measured value")
plt.ylabel("distance to the wall (mm)")
plt.ylim([0, max_dist])
plt.show()
