import numpy as np
import matplotlib.pyplot as plt
import math

# PARAMETERS
laser_min_range = 0.04
laser_max_range = 4.00
sampling_resolution_mm = 0.01
desired_scanning_resolution = 0.05

# SAMPLING
d = np.arange (laser_min_range, laser_max_range+sampling_resolution_mm, sampling_resolution_mm)
angle = np.arccos(1.0 - ((desired_scanning_resolution**2)/ (2*d**2) ))

# FIT CURVE (d has reciphoral properties. So fitting 1/d against angle gives nearly linear result)
# (1) angle = f(d)
# (2)       = p(1/d)
# (3)       = (1/d)*coef[0] + coef[1]
coefficients = np.polyfit(1.0/d, angle, 1)
curve_model = np.poly1d(coefficients)

print("Coefficients:")
print(coefficients)

# PLOT
plt.scatter(1.0/d, angle)
plt.plot(1.0/d, curve_model(1.0/d))
plt.show()
