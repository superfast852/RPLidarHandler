from matplotlib import pyplot as plt
import numpy as np

datapoints = np.linspace(0, 2*np.pi, 360)
points = np.array([np.cos(datapoints), np.sin(datapoints), datapoints])
x, y, c = points
plt.scatter(x, y, c=c, cmap='plasma')
plt.show()