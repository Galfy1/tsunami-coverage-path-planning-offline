from collections import deque as queue
import numpy as np
import math
from shapely.geometry import Point, LineString, Polygon

from shared_tools.custom_cell_tools import is_cell_valid, dx_4way, dy_4way, dx_8way, dy_8way

# BASED ON https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/

# Function to perform the BFS traversal
# grid: 2D numpy array where 1 = flyable, 0 = no-fly zone
# start_y, start_x: starting coordinates for the BFS (lat, lon)
def breadth_first_traversal(grid, start_cell, allow_diagonal = False):

    start_y = start_cell[0]
    start_x = start_cell[1]

    # Error check
    if (grid[start_y][start_x] == 0):
        raise ValueError("Starting point must be within the polygon and outside no-fly zones")

    # Declare the visited array
    vis = np.full((grid.shape[0], grid.shape[1]), False)

    # Result list to store the BFS traversal order
    result = []

    # Stores indices of the matrix cells
    q = queue()

    # Mark the starting cell as visited
    # and push it into the queue
    q.append(( start_y, start_x ))
    vis[start_y][start_x] = True

    # Iterate while the queue
    # is not empty
    while (len(q) > 0):
        cell = q.popleft()
        x = cell[1]
        y = cell[0]
        result.append((y, x))

        # If diagonal movement is allowed, check the diagonal cells
        if allow_diagonal == False:
            for i in range(4):
                adjx = x + dx_4way[i]
                adjy = y + dy_4way[i]
                if (is_cell_valid(grid, vis, adjx, adjy)):
                    q.append((adjy, adjx ))
                    vis[adjy][adjx] = True
        else:
            for i in range(8):
                adjx = x + dx_8way[i]
                adjy = y + dy_8way[i]
                if (is_cell_valid(grid, vis, adjx, adjy)):
                    q.append((adjx, adjx))
                    vis[adjy][adjx] = True

    return result


