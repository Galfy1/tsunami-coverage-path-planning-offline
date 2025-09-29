from collections import deque as queue
import numpy as np


# BASED ON https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/

# Direction vectors
dRow = [ -1, 0, 1, 0]
dCol = [ 0, 1, 0, -1]

# Function to check if a cell
# is be visited or not
def isValid(grid, vis, row, col):
  
    # If cell lies out of bounds
    if (row < 0 or col < 0 or row >= 4 or col >= 4):
        return False

    # If cell is already visited
    if (vis[row][col]):
        return False
    
    # If cell is not traversable (i.e. "no fly zone")
    if (grid[row][col] == 0): 
        return False

    # Otherwise
    return True

# Function to perform the BFS traversal
def BFS(grid, start_row, start_col):

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
            if (isValid(grid, vis, adjx, adjy)):
                q.append((adjx, adjy))
                vis[adjx][adjy] = True

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

    print(BFS(grid, 0, 0))