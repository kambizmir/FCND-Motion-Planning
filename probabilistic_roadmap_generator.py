import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid , probabilistic_a_star
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import math
from bresenham import bresenham

TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

SAMPLE_NUMBER =35

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)            
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

print("random x" , np.random.choice(grid.shape[0],SAMPLE_NUMBER))
print("random y" , np.random.choice(grid.shape[1],SAMPLE_NUMBER))


print("Generating random points")
random_points = [ (x,y) for x in np.random.choice(grid.shape[0],SAMPLE_NUMBER) for y in np.random.choice(grid.shape[1],SAMPLE_NUMBER)]

print("random_points = ",random_points)
print("length of random_points = ",len(random_points))

print("Filtering to free space points")
free_space_random_points = [ p for p in random_points if grid[p[0],p[1]] == 0 ]
print("free_space_random_points = " , free_space_random_points)
print("length of free_space_random_points = " , len(free_space_random_points))   

print("Generating valid actions map")
valid_actions_map ={}
for p in free_space_random_points:
    valid_actions_map[p] = []

for p in free_space_random_points:
    for q in free_space_random_points:
        if p != q:
            pathClear = True
            cells = list(bresenham(p[0], p[1], q[0], q[1]))
            for cell in cells:
                if grid[ cell[0],cell[1] ] == 1:
                    pathClear = False
                    break
            if pathClear:
                valid_actions_map[p].append( [ q[0]-p[0] , q[1]-p[1] , np.sqrt( (q[0]-p[0])**2 +  (q[1]-p[1])**2  ) ] )


print(free_space_random_points)
print(valid_actions_map)


import pickle

f = open("free_space_random_points.pkl","wb")
pickle.dump(free_space_random_points,f)
f.close()


f = open("valid_actions_map.pkl","wb")
pickle.dump(valid_actions_map,f)
f.close()