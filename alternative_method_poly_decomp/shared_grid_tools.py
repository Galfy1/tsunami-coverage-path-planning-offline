import queue
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from collections import deque as queue

from shared_tools.custom_cell_tools import dx_4way, dy_4way, dx_8way, dy_8way

n_neighbors = 8
dx_nway = dx_8way
dy_nway = dy_8way

def are_grids_adjacent(grid1: np.ndarray, grid2: np.ndarray) -> bool:
    # Both grids are same size. they either have 1 or 0. we want to check if they share a common boundary segment of 1s

    # error check
    if grid1.shape != grid2.shape:
        raise ValueError("Grids must be of the same shape to check adjacency in this function")

    # check all cells:
    for y in range(grid1.shape[0]):
        for x in range(grid1.shape[1]):

            if grid1[y][x] == 1:

                # check neighbors in grid2
                for direction in range(n_neighbors):

                    # adjacent cell coordinates
                    adjx = x + dx_nway[direction]
                    adjy = y + dy_nway[direction]

                    # bounds check
                    if (adjx < 0 or adjy < 0 or adjx >= grid2.shape[1] or adjy >= grid2.shape[0]):
                        continue # out of bounds

                    # check if adjacent cell in grid2 is 1
                    if grid2[adjy][adjx] == 1:
                        return True
        
    return False


def split_grid_with_disconnected_sections(grid: np.ndarray):
    # check for disconnected sections in the grid and split into multiple sub-grids if found

    disconnected_sections = []
    
    #print("BURGER")

    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if grid[y][x] == 1:
                #print(f"SUSHI {x}, {y}")
                # found a flyable cell, start flood fill to find all connected cells
                visited = np.zeros_like(grid)
                to_visit = queue()
                to_visit.append( (y,x) )
                visited[y][x] = 1

                # flood fill flyable area (1s in grid)
                while(len(to_visit) > 0):
                    cy, cx = to_visit.popleft()

                    # check neighbors
                    for direction in range(n_neighbors):
                        adjx = cx + dx_nway[direction]
                        adjy = cy + dy_nway[direction]

                        # bounds check
                        if (adjx < 0 or adjy < 0 or adjx >= grid.shape[1] or adjy >= grid.shape[0]):
                            continue # out of bounds

                        if grid[adjy][adjx] == 1 and visited[adjy][adjx] == 0:
                            visited[adjy][adjx] = 1
                            to_visit.append( (adjy, adjx) )
                
                # Now, visited contains all connected cells from the starting point
                # Create a new sub-grid for this connected section
                sub_grid = np.zeros_like(grid)
                for yy in range(grid.shape[0]):
                    for xx in range(grid.shape[1]):
                        if visited[yy][xx] == 1:
                            sub_grid[yy][xx] = 1
                
                disconnected_sections.append(sub_grid)

                # Remove the visited cells from the original grid to avoid re-processing
                for yy in range(grid.shape[0]):
                    for xx in range(grid.shape[1]):
                        if visited[yy][xx] == 1:
                            grid[yy][xx] = 0


    return disconnected_sections


def split_grid_along_sweep_line(grid: np.ndarray, sweep_line: LineString):

    sub_grids = []

    # (remember, shapely LineString coords are in (x,y) format, while our grid is in (y,x) format)

    # Split the grid along the selected sweep line into two sub-grids
    if sweep_line.coords[0][1] == sweep_line.coords[1][1]:  # horizontal line (if p1 y matches p2 y)
        split_y = int(sweep_line.coords[0][1]) # "height"(y) if horizontal line
        # create sub-grids (same dimensions as original grid, but with 0s outside the sub-area
        sub_grid1 = np.copy(grid)
        sub_grid1[split_y:,:] = 0
        sub_grid2 = np.copy(grid)
        sub_grid2[:split_y,:] = 0
    else:  # vertical line
        split_x = int(sweep_line.coords[0][0]) # "width"(x) if vertical line
        sub_grid1 = np.copy(grid)
        sub_grid1[:, split_x:] = 0
        sub_grid2 = np.copy(grid)
        sub_grid2[:, :split_x] = 0


    # sub_grids.append(sub_grid1)
    # sub_grids.append(sub_grid2)
    # TODO indkommenter når det andet optil virker (og fjern det lige ovenfor)
    # If any of the sub-grids have disconnected sections, split them further
    sub_grids.extend(split_grid_with_disconnected_sections(sub_grid1))
    sub_grids.extend(split_grid_with_disconnected_sections(sub_grid2))

    return sub_grids