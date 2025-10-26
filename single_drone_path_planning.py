from collections import deque as queue
import numpy as np
import math
from shapely.geometry import Point, LineString, Polygon
from breadth_first_traversal import breadth_first_traversal
from custom_cell_tools import is_cell_valid, dRow_4way, dCol_4way, dRow_8way, dCol_8way


# Order that make sure next cell is a neighbor of the current cell (breadth first traversal does not guarantee this)
def single_drone_traversal_order_bft(grid, start_cell, allow_diagonal_in_bft=False, allow_diagonal_in_path=False):
    bft = breadth_first_traversal(grid, start_cell, allow_diagonal=allow_diagonal_in_bft)

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


# TODO: !!! ALTERNATIV STRATEGI: BARE VÆLG DEN FØRSTE NEIBOR DER ER FRI. (start med venstre, så op, så højre, så ned, eller sådan noget). så holder dronen altid til venstre


def _find_closest_cell(grid, current_cell, visited_cells):
    min_dist = float("inf")
    closest_cell = None
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if(is_cell_valid(grid, visited_cells, x, y)):
                dx = abs(x - current_cell[1])
                dy = abs(y - current_cell[0])
                dist = math.sqrt(dx**2 + dy**2)  # Euclidean distance
                if dist < min_dist:
                    min_dist = dist
                    closest_cell = (y, x)
    return closest_cell # closest unvisited cell. returns None if no more valid cells are left

def _find_centroid_angle_diff_of_neighbors(grid, current_cell, visited_cells, centroid_line_angle: float, 
                                                 directional: str, allow_diagonal_in_path = True):
    #neighbor_with_smallest_angle_diff = None  # angle diff compared to centroid line direction
    x = current_cell[1]
    y = current_cell[0]
    result = []

    # TODO LIGE NU ER DET KUN 8 WAY. AKA allow_diagonal_in_path GØR INGENTING... IMPLIMENTER 4 WAY OGSÅ?
    # MÅSKE LAV EN WARNING MED AT DET IKKE GIVER SUPER MEGET MENEING MED allow_diagonal_in_path = false for centroid stuff?

    for i in range(8):
        adjx = x + dRow_8way[i]
        adjy = y + dCol_8way[i]
        if (is_cell_valid(grid, visited_cells, adjx, adjy)):
            neighbor_cell = (adjy, adjx)

            # Calculate angle from current_cell to neighbor_cell
            angle_to_neighbor = math.atan2(neighbor_cell[0] - current_cell[0], neighbor_cell[1] - current_cell[1])  # angle in radians

            # Calculate angle difference to centroid line angle
            
            angle_diff_rad = abs(angle_to_neighbor - centroid_line_angle)    # raw difference, but could be anywhere from 0 to 2π.
            angle_diff_rad = min(angle_diff_rad, 2*math.pi - angle_diff_rad) # ensure in [0, pi]. This step ensures we are measuring the shorter way around the circle (e.g. 350° → 10°).
            if directional == "bidirectional":
                angle_diff_rad = min(angle_diff_rad, math.pi - angle_diff_rad)   # ensure in [0, pi/2]. Folds any obtuse angle (>90°) back into an acute one, giving [0, pi/2].

            #print(f"Neighbor {i}, angle diff to centroid: {angle_diff_rad}")

            result.append((neighbor_cell, angle_diff_rad))

    return result  # list of (neighbor_cell, angle_diff_rad)

    #         # Check if this neighbor has the smallest angle difference so far
    #         if (neighbor_with_smallest_angle_diff is None) or (angle_diff_rad < neighbor_with_smallest_angle_diff[1]):
    #             neighbor_with_smallest_angle_diff = (neighbor_cell, angle_diff_rad)

    # return neighbor_with_smallest_angle_diff  # (neighbor_cell, angle_diff_rad) or None if no valid neighbor found


# "unidirectional" angle difference (0 to pi). "bidirectional" would be (0 to pi/2). 
# "bidirectional" does not seem to work very well for the "pure centroid" method (will sometimes do really sharp turns in direction)
def _find_next_cell_centroid(grid, current_cell, visited_cells, centroid_line_angle: float, 
                             directional = "unidirectional", allow_diagonal_in_path = True, angle_offset_rad = 0):

    centroid_angle_diff_of_neighbors = _find_centroid_angle_diff_of_neighbors(grid, current_cell, visited_cells, centroid_line_angle+angle_offset_rad,
                                                                              directional, allow_diagonal_in_path)
    #print(f"centroid angle: {centroid_angle_diff_of_neighbors}")

    if centroid_angle_diff_of_neighbors: # if list is not empty
        # Find the neighbor with the smallest angle difference
        # centroid_angle_diff_of_neighbors is a list of (neighbor_cell, angle_diff_rad)
        neighbor_with_smallest_angle_diff = min(centroid_angle_diff_of_neighbors, key=lambda x: x[1])
        #print(f"Next cell chosen with angle diff: {neighbor_with_smallest_angle_diff[1]}")
        return neighbor_with_smallest_angle_diff[0] 
    else:
        # No unvisited neighbor is found. Find the closest unvisited cell
        #print("No unvisited neighbor found. Finding closest unvisited cell...")
        return _find_closest_cell(grid, current_cell, visited_cells) # closest unvisited cell. returns None if no more valid cells are left

# "Unidirectional" angle difference (0 to pi). "Bidirectional" would be (0 to pi/2). 
#  For this hybrid approach, we want "Bidirectional" (the issues of bidirectional is what we are trying to fix by taking into account current direction as well)
def _find_next_cell_hybrid(grid, current_cell, visited_cells, centroid_line_angle: float, current_direction_angle: float, 
                           weight_centroid, directional = "bidirectional", allow_diagonal_in_path = True, angle_offset_rad = 0):

    centroid_angle_diff_of_neighbors = _find_centroid_angle_diff_of_neighbors(grid, current_cell, visited_cells, centroid_line_angle+angle_offset_rad, directional, allow_diagonal_in_path)

    if centroid_angle_diff_of_neighbors: # if list is not empty

        # Find the neighbor with the smallest weighted angle difference
        # centroid_angle_diff_of_neighbors is a list of (neighbor_cell, angle_diff_rad)
        min_weighted_angle_diff = float("inf")
        best_neighbor = None

        # (here, angles are in radians)
        for neighbor_cell, angle_diff_to_centroid in centroid_angle_diff_of_neighbors:
            # Calculate angle from current_cell to neighbor_cell
            angle_to_neighbor = math.atan2(neighbor_cell[0] - current_cell[0], neighbor_cell[1] - current_cell[1])  # angle in radians

            # Calculate angle difference to current direction (we always want this to be unidirectional, i.e. [0, pi]. cus we want to penalize sharp turns)
            angle_diff_to_current_dir = abs(angle_to_neighbor - current_direction_angle)
            angle_diff_to_current_dir = min(angle_diff_to_current_dir, 2*math.pi - angle_diff_to_current_dir) # ensure in [0, pi]
        
            # Calculate weighted angle difference
            weighted_angle_diff = (weight_centroid * angle_diff_to_centroid) + ((1 - weight_centroid) * angle_diff_to_current_dir)

            if weighted_angle_diff < min_weighted_angle_diff:
                min_weighted_angle_diff = weighted_angle_diff
                best_neighbor = neighbor_cell

        return best_neighbor


    else:
        # No unvisited neighbor is found. Find the closest unvisited cell
        #print("No unvisited neighbor found. Finding closest unvisited cell...")
        return _find_closest_cell(grid, current_cell, visited_cells) # closest unvisited cell. returns None if no more valid cells are left

# Alternative ways to do traversal order path planning
def single_drone_traversal_order_alt(grid, start_cell, start_gps, polygon: Polygon,
                                     method, allow_diagonal_in_path = True, hybrid_centroid_weight = 0.5): 

    start_cell_y = start_cell[0]
    start_cell_x = start_cell[1]
    start_coord_y = start_gps[0]
    start_coord_x = start_gps[1]

    result = []
    result.append((start_cell_y, start_cell_x))

    # If "centroid" or "hybrid" method is chosen, we need the "centroid line" angle
    centroid = polygon.centroid  # Point(lon, lat)

    # Calculate the centroid line angle (remember, centroid is in coordinates, so we need start_gps)
    centroid_line_angle = math.atan2(centroid.y - start_coord_y, centroid.x - start_coord_x)  # angle in radians

    #print(f"Centroid line angle: {centroid_line_angle}")

    # For "hybrid" method, we also need the current direction angle
    current_direction_angle = 0.0  # initial direction angle (radians). Could be set to any value, as it will be updated after the first move.
    
    # Declare the visited array
    vis = np.full((grid.shape[0], grid.shape[1]), False)

    while(True):
        current_cell = result[-1]
        if method == "centroid":
            next_cell = _find_next_cell_centroid(grid, current_cell, vis, centroid_line_angle, allow_diagonal_in_path=allow_diagonal_in_path)
        elif method == 'centroid90':
            next_cell = _find_next_cell_centroid(grid, current_cell, vis, centroid_line_angle, allow_diagonal_in_path=allow_diagonal_in_path, angle_offset_rad=math.pi/2)
        elif method == 'centroid180':
            next_cell = _find_next_cell_centroid(grid, current_cell, vis, centroid_line_angle, allow_diagonal_in_path=allow_diagonal_in_path, angle_offset_rad=math.pi)
        elif method == "centroid_hybrid":
            next_cell = _find_next_cell_hybrid(grid, current_cell, vis, centroid_line_angle, current_direction_angle, 
                                               hybrid_centroid_weight, allow_diagonal_in_path=allow_diagonal_in_path)
            # Update current direction angle
            if next_cell is not None:
                current_direction_angle = math.atan2(next_cell[0] - current_cell[0], next_cell[1] - current_cell[1]) 
        elif method == "centroid90_hybrid":
            next_cell = _find_next_cell_hybrid(grid, current_cell, vis, centroid_line_angle, current_direction_angle, 
                                               hybrid_centroid_weight, allow_diagonal_in_path=allow_diagonal_in_path, angle_offset_rad=math.pi/2)
            # Update current direction angle
            if next_cell is not None:
                current_direction_angle = math.atan2(next_cell[0] - current_cell[0], next_cell[1] - current_cell[1]) 
        else:
            raise ValueError("Invalid method specified.")

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