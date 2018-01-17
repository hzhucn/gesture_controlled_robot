#!/usr/bin/python

#### Very stupid bad way of doing the job
#### visually fit a curve to the one of the manual , do not trust this too much !


import matplotlib.pyplot as plt
import numpy as np

@np.vectorize
def get_ir_from_dist(d, cut_dist, half_dist, max_dist, order):
    # d (mm)
    if(d <= cut_dist):
        return 1023 + d * (1000.-1023.)/30
    elif(d >= max_dist):
        return 0
    else:
        alpha = (half_dist - cut_dist) / (np.exp(-np.log(0.5)/order) - 1.0)
        return 1000. / ((d - cut_dist)/alpha + 1)**order

@np.vectorize
def get_dist_from_ir(ir, cut_dist, half_dist, max_dist, order):
    if(ir >= 1000):
        return (ir - 1023) * cut_dist / (1000 - 1023.)
    else:
        alpha = (half_dist - cut_dist) / (np.exp(-np.log(0.5)/order) - 1.0)
        d = (np.exp(1.0/order * np.log(1000./ir)) - 1) * alpha + cut_dist
        if(d >= max_dist):
            return np.nan
        else:
            return d
        

cut_dist = 30 # mm
max_dist = 200 # mm
half_dist = 50 # mm, where the response is 512
order = 5

dvalues = np.linspace(0, 250, 1000) # mm
IR_value = get_ir_from_dist(dvalues, cut_dist, half_dist, max_dist, order)

IR_values = np.linspace(0.01, 1023, 1000)
distances = get_dist_from_ir(IR_values, cut_dist, half_dist, max_dist, order)

plt.figure()
plt.subplot(1,2,1)
plt.plot(dvalues/10, IR_value)
plt.xlabel("distance to the wall (cm)")
plt.ylabel("Measured value")
plt.ylim([0, 1024])
plt.xlim([0, max_dist/10])
plt.subplot(1,2,2)
plt.plot(IR_values, distances/10)
plt.xlabel("Measured value")
plt.ylabel("distance to the wall (cm)")
plt.xlim([0, 1024])
plt.ylim([0, max_dist/10])
plt.show()
