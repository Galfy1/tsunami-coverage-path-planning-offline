from collections import deque as queue
import numpy as np
import math
from shapely.geometry import Point, LineString, Polygon

# BASED ON https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/

# Direction vectors
dRow_4way = [ -1, 0, 1, 0]
dCol_4way = [ 0, 1, 0, -1]

dRow_8way = [ -1, -1, 0, 1, 1, 1, 0, -1]
dCol_8way = [ 0, 1, 1, 1, 0, -1, -1, -1]


# Function to check if a cell
# is be visited or not
def is_valid(grid, vis, x, y):
  
    # If cell lies out of bounds
    if (x < 0 or y < 0 or x >= grid.shape[1] or y >= grid.shape[0]):
        return False

    # If cell is already visited
    if (vis[y][x]):
        return False
    
    # If cell is not traversable (i.e. "no fly zone")
    if (grid[y][x] == 0):
        return False

    # Otherwise
    return True

# Function to perform the BFS traversal
# grid: 2D numpy array where 1 = flyable, 0 = no-fly zone
# start_y, start_x: starting coordinates for the BFS (lat, lon)
def breadth_first_traversal(grid, start_y, start_x, allow_diagonal = False):

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
                adjx = x + dRow_4way[i]
                adjy = y + dCol_4way[i]
                if (is_valid(grid, vis, adjx, adjy)):
                    q.append((adjy, adjx ))
                    vis[adjy][adjx] = True
        else:
            for i in range(8):
                adjx = x + dRow_8way[i]
                adjy = y + dCol_8way[i]
                if (is_valid(grid, vis, adjx, adjy)):
                    q.append((adjx, adjx))
                    vis[adjy][adjx] = True

    return result


# Order that make sure next cell is a neighbor of the current cell (breadth first traversal does not guarantee this)
def single_drone_traversal_order_bft(grid, start_y, start_x, allow_diagonal_in_bft=False, allow_diagonal_in_path=False):
    bft = breadth_first_traversal(grid, start_y, start_x, allow_diagonal=allow_diagonal_in_bft)

    result = []
    visited = [False for _ in range(len(bft))]

    result.append(bft[0])
    visited[0] = True
    current_cell = bft[0]

    while len(result) < len(bft):  # while result list is not full
        found_neighbor = False

        # try to find a neighbor of current_cell that hasn't been visited
        for i in range(len(bft)):
            if not visited[i]:
                cell = bft[i]
                dx = abs(cell[1] - current_cell[1])
                dy = abs(cell[0] - current_cell[0])

                if (allow_diagonal_in_path and max(dx, dy) == 1) or (not allow_diagonal_in_path and dx + dy == 1):
                    # neighbor found!
                    result.append(cell)
                    visited[i] = True
                    current_cell = cell
                    found_neighbor = True
                    break

        if not found_neighbor:
            # no neighbor found — jump to the *closest* unvisited cell
            min_dist = float("inf")
            next_index = None

            for i in range(len(bft)):
                if not visited[i]:
                    cell = bft[i]
                    dx = abs(cell[1] - current_cell[1])
                    dy = abs(cell[0] - current_cell[0])

                    # Calculate euclidean distance
                    dist = math.sqrt(dx**2 + dy**2)

                    if dist < min_dist:
                        min_dist = dist
                        next_index = i

            if next_index is not None:
                result.append(bft[next_index])
                visited[next_index] = True
                current_cell = bft[next_index]

    return result




def _find_closest_cell(grid, current_cell, visited_cells):
    min_dist = float("inf")
    closest_cell = None
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if(is_valid(grid, visited_cells, x, y)):
                dx = abs(x - current_cell[1])
                dy = abs(y - current_cell[0])
                dist = math.sqrt(dx**2 + dy**2)  # Euclidean distance
                if dist < min_dist:
                    min_dist = dist
                    closest_cell = (y, x)
    return closest_cell # closest unvisited cell. returns None if no more valid cells are left


def _find_next_cell_centroid(grid, current_cell, visited_cells, centroid_line_angle: float, allow_diagonal_in_path = True, directional = "unidirectional"):

    neighbor_with_smallest_angle_diff = None  # angle diff compared to centroid line direction

    x = current_cell[1]
    y = current_cell[0]

    # TODO: !!! ALTERNATIV STRATEGI: BARE VÆLG DEN FØRSTE NEIBOR DER ER FRI. (start med venstre, så op, så højre, så ned, eller sådan noget). så holder dronen altid til venstre
    # TODO: ALTERNATIV STATEGRI: DRONEN VÆLGER DEN NABO DER HAR EN VINKEL TÆTTEST PÅ DENS NUVÆRENDE RETNING (så vi prøver at undgå skarpe sving)
        # MÅSKE SKIP DEN HER... VED IKKE HVOR GOD DEN ER. SE DEN NEDENFOR I STEDET
    # TODO: !!! ALTERNATIV: EN HYBRID AF CENTROID OG NUVÆRENDE RETNING. DEN UDREGNER HER EN DESIRED RETNING BASERET PÅ CENTROID OG NUVÆRENDE RETNING. OG VÆGTER DEM BEGGE TO LIDT. F.EKS. 70% CENTROID, 30% NUVÆRENDE RETNING.
        # WUPS, SÅ SKAL DET VÆRE BIDIRECITONEL CENTROID. DEN VI HAR NU ER UNIDIRECTIONEL
        # NÆVN I RAPPORTEN PROBLEMER MED BIDIRECTIONEL CENTOID (konstant retningsskift), OG AT DENNE HYBRID METODE PRØVER AT FIKSE DET PROBLEM

    # If diagonal movement is allowed, check the diagonal cells
    # if allow_diagonal_in_path == False: # not sure how much sense it makes to not allow diagonal here...
    #     for i in range(4):
    #         adjx = x + dRow_4way[i]
    #         adjy = y + dCol_4way[i]
    #         if (is_valid(grid, vis, adjx, adjy)):
    #             current_cell = (adjx, adjy)
    #             result.append((adjx, adjy))
    #             vis[adjx][adjy] = True
    # else:
    for i in range(8):
        adjx = x + dRow_8way[i]
        adjy = y + dCol_8way[i]
        if (is_valid(grid, visited_cells, adjx, adjy)):
            neighbor_cell = (adjy, adjx)

            # Calculate angle from current_cell to neighbor_cell
            angle_to_neighbor = math.atan2(neighbor_cell[0] - current_cell[0], neighbor_cell[1] - current_cell[1])  # angle in radians

            # Calculate angle difference to centroid line angle
            # "Unidirectional" angle difference (0 to po). "Bidirectional" would be (0 to pi/2), but does not seem to work very well (constantly shifting direction) 
            angle_diff_rad = abs(angle_to_neighbor - centroid_line_angle)    # raw difference, but could be anywhere from 0 to 2π.
            angle_diff_rad = min(angle_diff_rad, 2*math.pi - angle_diff_rad) # ensure in [0, pi]. This step ensures we are measuring the shorter way around the circle (e.g. 350° → 10°).
            if directional == "bidirectional":
                angle_diff_rad = min(angle_diff_rad, math.pi - angle_diff_rad)   # ensure in [0, pi/2]. Folds any obtuse angle (>90°) back into an acute one, giving [0, pi/2].

            # Check if this neighbor has the smallest angle difference so far
            if (neighbor_with_smallest_angle_diff is None) or (angle_diff_rad < neighbor_with_smallest_angle_diff[1]):
                neighbor_with_smallest_angle_diff = (neighbor_cell, angle_diff_rad)

    if neighbor_with_smallest_angle_diff != None:
        # neighbor found!
        return neighbor_with_smallest_angle_diff[0] # (neighbor_cell, angle_diff_rad)
    else:
        # No unvisited neighbor is found. Find the closest unvisited cell
        print("No unvisited neighbor found. Finding closest unvisited cell...")
        return _find_closest_cell(grid, current_cell, visited_cells) # closest unvisited cell. returns None if no more valid cells are left


# Alternative ways to do traversal order path planning
def single_drone_traversal_order_alt(grid, start_y, start_x, polygon: Polygon, allow_diagonal_in_path = True, method = "centroid"): 

    result = []
    result.append((start_y, start_x))

    # If centroid method is chosen, we need the "centroid line" angle
    centroid = polygon.centroid  # Point(lon, lat)
    centroid_line_angle = math.atan2(abs(centroid.y - start_y), abs(centroid.x - start_x))  # angle in radians
    
    # Declare the visited array
    vis = np.full((grid.shape[0], grid.shape[1]), False)

    while(True):
        if method == "centroid":
            next_cell = _find_next_cell_centroid(grid, result[-1], vis, centroid_line_angle, allow_diagonal_in_path)
        elif method == "ASD":
            #next_cell = _find_next_cell_default(grid, result[-1], vis, allow_diagonal_in_path)

        if next_cell is None:
            break

        result.append(next_cell)
        vis[next_cell[0]][next_cell[1]] = True

    return result

# # Test Code
# if __name__ == '__main__':
  
#     # Given input matrix
#     grid = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
#                     [0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
#                     [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0],
#                     [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0],
#                     [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
#                     [0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
#                     [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
#                     [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
#                     [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1],
#                     [0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1],
#                     [0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#                     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1]])



#     # vis, False, sizeof vis)

#     print(traversal_order(grid, 10, 10))