import numpy as np

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


def convert_grid_to_gps(fly_nofly_grid, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y):
    gps_grid = np.empty(fly_nofly_grid.shape, dtype=object)  # create empty grid of same shape (dtype object to hold python tuples)
    for y in range(fly_nofly_grid.shape[0]):
        for x in range(fly_nofly_grid.shape[1]):
            lat = y_axis_coords[y] + (grid_res_y / 2)  # center of cell
            lon = x_axis_coords[x] + (grid_res_x / 2)  # center of cell
            gps_grid[y, x] = (lat, lon)
    return gps_grid