import matplotlib.pyplot as plt
import numpy as np

with open('path.txt', 'r') as f:
    lines = f.readlines()
    x = [float(line.split(',')[0]) for line in lines]
    y = [float(line.split(',')[1]) for line in lines]
    z = [float(line.split(',')[2]) for line in lines]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.set_aspect('equal')
plt.show()

