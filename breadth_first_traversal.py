from collections import deque as queue
import numpy as np
import math
from shapely.geometry import Point, LineString, Polygon

# BASED ON https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/

# TODO DER ER ET FUNDEMENTALT ISSUE HER... BFS ER POITNSE IKKE NØDVENDLIGVIS VED SIDEN AF HINANDEN
    # I DERES PAPER: er hver point ved siden af hidnanden fordi dronen vælger de points der er tættest på

# Direction vectors
dRow_4way = [ -1, 0, 1, 0]
dCol_4way = [ 0, 1, 0, -1]

dRow_8way = [ -1, -1, 0, 1, 1, 1, 0, -1]
dCol_8way = [ 0, 1, 1, 1, 0, -1, -1, -1]


# TODO PRØV AT LAV DIAGONASL I DET HER. så skal vi måske også passe metadata til tsunami om om det er med eller uden diagnonal (så kan den mirror valget for sin nabo finding halløj)
# Man kan også ændre på rækkefælgen af dRow og dCol så den f.eks. altid prøver at gå højre først, så ned, så venstre, så op (eller sådan noget)

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
        #print(grid[x][y], end = " ")
        result.append((y, x))

        #q.pop()

        # # Go to the adjacent cells (not diagonal)
        # for i in range(4):
        #     adjx = x + dRow[i]
        #     adjy = y + dCol[i]
        #     if (is_valid(grid, vis, adjx, adjy)):
        #         q.append((adjx, adjy))
        #         vis[adjx][adjy] = True

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
# def traversal_order(grid, start_row, start_col, allow_diagonal = False):
#     bft = breadth_first_traversal(grid, start_row, start_col)

#     result = []
#     visited = [False for _ in range(len(bft))]

#     result.append(bft[0])
#     visited[0] = True
#     current_cell = bft[0]

#     while (len(result) < len(bft)): # while result list is not full
#         # we now want to find the neighbor cell of current_cell that is "closest" in the bft list and has not yet been visited
#         #print("Current cell:", current_cell)
#         for i in range(len(bft)):   # iterate through all cells in bft from "closest" to "farthest"
#             if (visited[i] == False):
#                 cell = bft[i]
#                 # Check if cell is neighbor to current_cell
#                 dx = abs(cell[0] - current_cell[0])
#                 dy = abs(cell[1] - current_cell[1])
#                 if (allow_diagonal and max(dx, dy) == 1) or (not allow_diagonal and dx + dy == 1):
#                     # neighbor found!
#                     result.append(cell)
#                     visited[i] = True
#                     current_cell = cell
#                     break
#             if (i == len(bft) - 1):
#                 # no neighbor found... Instead, we add the closest unvisited cell to current_cell
#                 closest_cell = (float('inf'), float('inf'))
#                 for j in range(len(bft)):
#                     if (visited[j] == False):
#                         # Check if this cell is closer than the current closest
#                         if (abs(bft[j][0] - current_cell[0]) + abs(bft[j][1] - current_cell[1]) < abs(closest_cell[0] - current_cell[0]) + abs(closest_cell[1] - current_cell[1])):
#                             closest_cell = bft[j]
#                 # Add the closest unvisited cell to the result
#                 # if (closest_cell != (float('inf'), float('inf'))):
#                 #     result.append(bft[closest_cell_index])
#                 #     visited[closest_cell_index] = True
#                 #     current_cell = bft[closest_cell_index]
#                 # else:
#                 #     raise ValueError("No unvisited cells left, but result list is not full. This should never happen.")
                
#     return result


# Order that make sure next cell is a neighbor of the current cell (breadth first traversal does not guarantee this)
def single_drone_traversal_order(grid, start_y, start_x, allow_diagonal_in_bft=False, allow_diagonal_in_path=False):
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







def find_next_cell_centroid(grid, current_cell, visited_cells, centroid_line_angle: float, allow_diagonal_in_path = True):

    neighbor_with_smallest_angle_diff = None  # angle diff compared to centroid line direction

    #waypoint_count = np.sum(grid == 1) # amount of waypoints to be visited

    x = current_cell[1]
    y = current_cell[0]


    neighbor_found = False

    # HUSK DET YDRE LOOP SKAL TAGE HØJDE FOR AT ANDRE KAN ÆNDRE VISISTED ARRAAYED (så vi nemmere kan implimerentere det i simmen)
    # KIG LIGE PÅ HVORDAN VI HAR GJORT DET HER I SIMMEN.
    # ligesom i find_next_cell


    # TODO: ALTERNATIV STRATEGI: BARE VÆLG DEN FØRSTE NEIBOR DER ER FRI. (start med venstre, så op, så højre, så ned, eller sådan noget). så holder dronen altid til venstre


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
            angle_diff_rad = abs(angle_to_neighbor - centroid_line_angle)    # raw difference, but could be anywhere from 0 to 2π.
            angle_diff_rad = min(angle_diff_rad, 2*math.pi - angle_diff_rad) # ensure in [0, pi]. This step ensures we are measuring the shorter way around the circle (e.g. 350° → 10°).
            angle_diff_rad = min(angle_diff_rad, math.pi - angle_diff_rad)   # ensure in [0, pi/2]. Folds any obtuse angle (>90°) back into an acute one, giving [0, pi/2].

            # Check if this neighbor has the smallest angle difference so far
            if (neighbor_with_smallest_angle_diff is None) or (angle_diff_rad < neighbor_with_smallest_angle_diff[1]):
                neighbor_with_smallest_angle_diff = (neighbor_cell, angle_diff_rad)

    if neighbor_with_smallest_angle_diff != None:
        # neighbor found!
        return neighbor_with_smallest_angle_diff[0] # (neighbor_cell, angle_diff_rad)
    else:
        # No unvisited neighbor is found. Find the closest unvisited cell
        min_dist = float("inf")
        closest_cell = None
        for cell in grid:
            if(is_valid(grid, visited_cells, cell[1], cell[0])):
                dx = abs(cell[1] - current_cell[1])
                dy = abs(cell[0] - current_cell[0])
                dist = math.sqrt(dx**2 + dy**2)  # Euclidean distance
                if dist < min_dist:
                    min_dist = dist
                    closest_cell = cell
        return closest_cell # closest unvisited cell. returns None if no more valid cells are left

    


def single_drone_traversal_order_centroid(grid, start_y, start_x, polygon: Polygon, allow_diagonal_in_path = True):

    centroid = polygon.centroid  # Point(lon, lat)
    centroid_line = LineString([Point(start_x, start_y), centroid]) # Point(lon, lat)

    result = []
    result.append((start_y, start_x))

    centroid_line_angle = math.atan2(centroid.y - start_y, centroid.x - start_x)  # angle in radians
    
    # Declare the visited array
    vis = np.full((grid.shape[0], grid.shape[1]), False)

    while(True):
        next_cell = find_next_cell_centroid(grid, result[-1], vis, centroid_line_angle, allow_diagonal_in_path)
        if next_cell is None:
            break

        result.append(next_cell)
        vis[next_cell[0]][next_cell[1]] = True

    return result

# Test Code
if __name__ == '__main__':
  
    # Given input matrix
    grid = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
                    [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
                    [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
                    [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1],
                    [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1],
                    [0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1],
                    [0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1]])



    # vis, False, sizeof vis)

    print(traversal_order(grid, 10, 10))