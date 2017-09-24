import ikpy
import numpy as np
a = -3 * np.pi / 4
b = 5 * np.pi / 6
print(a, b)
print(ikpy.geometry_utils.angle_difference(b, a))
