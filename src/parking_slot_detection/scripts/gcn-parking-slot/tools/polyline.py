import random
import numpy as np

num_points=20
points=np.random.uniform(0, 100, size=(num_points,2))

print("Point:")
for i, (x,y) in enumerate(points):
    print(f"point {i+1}:({x: .2f},{y: .2f})")

