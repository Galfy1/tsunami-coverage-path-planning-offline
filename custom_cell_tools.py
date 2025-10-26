# Direction vectors
dRow_4way = [ -1, 0, 1, 0]
dCol_4way = [ 0, 1, 0, -1]

dRow_8way = [ -1, -1, 0, 1, 1, 1, 0, -1]
dCol_8way = [ 0, 1, 1, 1, 0, -1, -1, -1]


# Function to check if a cell
# is be visited or not
def is_cell_valid(grid, vis, x, y):
  
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