from collections import deque as queue
import numpy as np
import math

# BASED ON https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/

# TODO DER ER ET FUNDEMENTALT ISSUE HER... BFS ER POITNSE IKKE NØDVENDLIGVIS VED SIDEN AF HINANDEN
    # I DERES PAPER: er hver point ved siden af hidnanden fordi dronen vælger de points der er tættest på

# Direction vectors
dRow = [ -1, 0, 1, 0]
dCol = [ 0, 1, 0, -1]

# TODO PRØV AT LAV DIAGONASL I DET HER. så skal vi måske også passe metadata til tsunami om om det er med eller uden diagnonal (så kan den mirror valget for sin nabo finding halløj)
# Man kan også ændre på rækkefælgen af dRow og dCol så den f.eks. altid prøver at gå højre først, så ned, så venstre, så op (eller sådan noget)

# Function to check if a cell
# is be visited or not
def is_valid(grid, vis, x, y):
  
    # If cell lies out of bounds
    if (x < 0 or y < 0 or x >= grid.shape[0] or y >= grid.shape[1]):
        return False

    # If cell is already visited
    if (vis[x][y]):
        return False
    
    # If cell is not traversable (i.e. "no fly zone")
    if (grid[x][y] == 0):
        return False

    # Otherwise
    return True

# Function to perform the BFS traversal
# grid: 2D numpy array where 1 = flyable, 0 = no-fly zone
# start_row, start_col: starting coordinates for the BFS
def breadth_first_traversal(grid, start_row, start_col):

    # Error check
    if (grid[start_row][start_col] == 0):
        raise ValueError("Starting point must be within the polygon and outside no-fly zones")

    # Declare the visited array
    vis = np.full((grid.shape[0], grid.shape[1]), False)

    # Result list to store the BFS traversal order
    result = []

    # Stores indices of the matrix cells
    q = queue()

    # Mark the starting cell as visited
    # and push it into the queue
    q.append(( start_row, start_col ))
    vis[start_row][start_col] = True

    # Iterate while the queue
    # is not empty
    while (len(q) > 0):
        cell = q.popleft()
        x = cell[0]
        y = cell[1]
        #print(grid[x][y], end = " ")
        result.append((x, y))

        #q.pop()

        # Go to the adjacent cells (not diagonal)
        for i in range(4):
            adjx = x + dRow[i]
            adjy = y + dCol[i]
            if (is_valid(grid, vis, adjx, adjy)):
                q.append((adjx, adjy))
                vis[adjx][adjy] = True

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
def single_drone_traversal_order(grid, start_row, start_col, allow_diagonal=False):
    bft = breadth_first_traversal(grid, start_row, start_col)

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
                dx = abs(cell[0] - current_cell[0])
                dy = abs(cell[1] - current_cell[1])

                if (allow_diagonal and max(dx, dy) == 1) or (not allow_diagonal and dx + dy == 1):
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
                    dx = abs(cell[0] - current_cell[0])
                    dy = abs(cell[1] - current_cell[1])

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