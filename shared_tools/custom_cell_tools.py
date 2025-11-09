# Direction vectors
dx_4way = [ -1, 0, 1, 0]
dy_4way = [ 0, 1, 0, -1]

dx_8way = [ -1, -1, 0, 1, 1, 1, 0, -1]
dy_8way = [ 0, 1, 1, 1, 0, -1, -1, -1]

diagonal_indices_8way = [1, 3, 5, 7]  # Indices for diagonal directions in dx_8way/dy_8way


# Function to check if a cell
# is be visited or not
# (its assumed that grid and vis are numpy arrays and cells are stored as (y,x))
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