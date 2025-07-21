import random
import numpy as np

num_points=20
points=np.random.uniform(0, 100, size=(num_points,2))

def callength(points):
    diffs=np.diff(points,axis=0)
    distance=np.linalg.norm(diffs,axis=1)
    length_total=np.sum(distance)
    return length_total

polyline_length=callength(points)
print(f"")

print("Point:")
for i, (x,y) in enumerate(points):
    print(f"point {i+1}:({x: .2f},{y: .2f})")


