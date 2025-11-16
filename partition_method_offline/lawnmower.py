import math
import numpy as np
from partition_method_offline.scan_for_monotonicity import scan_for_non_monotone_sections
from shared_tools.custom_cell_tools import dx_4way, dy_4way, dx_8way, dy_8way, diagonal_indices_8way


def lawnmower(grid: np.ndarray, start_corner = 'nw', direction: str = 'horizontal'):
    # implement lawnmower path generation for the given grid


    # TODO Gør, så når man lawnmover, tjekker den gridded om det er mononome i en retning.
	#hvis den er monoton i et retning, kan den ikke lave simple lawnmover (uden potentielt at have missed area... som ville kræve path planning (e.g. A*) for at "backpropegate" ud af stuck, hen til den tætteste missed cell. samtilidgt vil det også betyde revisisted cells.. hvilket er ineffektivt)
    # hvis den ikke allower er retning, så return et None path (eventuelt med inf længde og turns, etc.)


    path = []
    end_cell = None
    turn_count = 0

    # Error check 
    if direction not in ['horizontal', 'vertical']:
        raise ValueError("Invalid direction for lawnmower path. Use 'horizontal' or 'vertical'.")
    if start_corner not in ['nw', 'ne', 'sw', 'se']:
        raise ValueError("Invalid start corner. Use 'nw', 'ne', 'sw', or 'se'.")
    

    # Check if grid is suitable for lawnmower path
    _, _, non_monotone_in_x, non_monotone_in_y = scan_for_non_monotone_sections(grid)
    if (direction == 'horizontal' and non_monotone_in_y) or (direction == 'vertical' and non_monotone_in_x):
        #print("WARNING: The provided grid is not suitable for simple lawnmower path in the chosen direction due to non-monotonicity.")
        return None, float('inf'), None, None, float('inf')


    # Find the starting cell
    def find_start_cell(grid, start_corner, direction):
        rows, cols = grid.shape

        # Determine scan directions
        y_range = range(rows)
        x_range = range(cols)

        # Flip ranges based on corner
        if 'n' in start_corner:  # north means start from top (higher y)
            y_range = range(rows - 1, -1, -1) # (from last row to 0  (start, end, step) (remember, end is exclusive, thats why its -1 for 0))
        if 's' in start_corner:  # south means start from bottom (lower y)
            y_range = range(rows)

        if 'w' in start_corner:  # west means start from left
            x_range = range(cols)
        if 'e' in start_corner:  # east means start from right
            x_range = range(cols - 1, -1, -1)

        # Swap scan order depending on direction
        #       To ensure uav gets all the area on 1 sweep:
        #           For horizontal dircetion, the starting cell HAS to one of the top/bottom most cells (depeding on start_corner variable)
        #           For vertical direction, the starting cell HAS to be one of the left/right most cells (depeding on start_corner variable)
        if direction == 'horizontal':
            outer, inner = y_range, x_range
        elif direction == 'vertical':
            outer, inner = x_range, y_range

        for a in outer:
            for b in inner:
                # Depending on direction, interpret (a,b)
                y, x = (a, b) if direction == 'horizontal' else (b, a)
                if grid[y][x] == 1:
                    return (y, x)

        return None
    

    #################### Find starting cell ####################

    start_cell = find_start_cell(grid, start_corner, direction)
    if start_cell is None:
        raise ValueError("No valid starting cell found in the grid.")

    #################### Coordinate abstraction ####################

    # since this code as to work for both horizontal and vertical lawnmower patterns,
    # we abstract away the coordinate system to work in "primary" and "secondary" axes instead of y,x.
    #   "primary" axis is the axis along which we sweep (x for horizontal, y for vertical)
    #   "secondary" axis is the axis along which we move to the next line (y for horizontal, x for vertical)
    
    if direction == "horizontal":
        # Primary axis is x (sweep along rows)
        def get_primary(y, x): return x
        def get_secondary(y, x): return y
        def convert_to_y_x(secondary, primary): return (secondary, primary)  # (y, x) - used to convert primary/secondary back to y,x 
        primary_size = grid.shape[1]  # width
        secondary_size = grid.shape[0]  # height
        primary_dir = 1 if 'w' in start_corner else -1 # primary direction 
        secondary_dir = -1 if 'n' in start_corner else 1 # secondary direction
    else:  # vertical
        # Primary axis is y (sweep along columns)
        def get_primary(y, x): return y
        def get_secondary(y, x): return x
        def convert_to_y_x(secondary, primary): return (primary, secondary)  # (y, x) - used to convert primary/secondary back to y,x 
        primary_size = grid.shape[0]  # height
        secondary_size = grid.shape[1]  # width
        primary_dir = -1 if 'n' in start_corner else 1 # primary direction
        secondary_dir = 1 if 'w' in start_corner else -1 # secondary direction

    y, x = start_cell # current position
    path.append((y, x))
    front_move_direction = secondary_dir
    lawnmower_state = 'sweep_in_1_direction'

    #################### Unified lawnmower path generation ####################

    while True:

        # (To update the current position, we update both y and x varaibles)
        # (primary/secondary are temporary variables only used inside the loop to accommodate both horizontal and vertical sweeping)

        if lawnmower_state == 'sweep_in_1_direction':
            primary = get_primary(y, x) # current position in primary axis
            secondary = get_secondary(y, x) # current position in secondary axis

            ######## Sweep in primary direction ########

            # before the actual sweeping, we handle the edge case of a lonely cell only beeing connected diagonally. 
            #   (this can happens due to the discretization... and the fact that we use 8-way connectivity in are_grids_adjacent() and split_grid_with_disconnected_sections())
            # check if the current cell is only connected diagonally to 1 neighbor (in any direction)
            neighbors_index = []
            for direction in range(8):
                # adjacent cell coordinates
                adjx = x + dx_8way[direction]
                adjy = y + dy_8way[direction]
                if 0 <= adjy < grid.shape[0] and 0 <= adjx < grid.shape[1]: # within bounds check
                    if grid[adjy, adjx] == 1 and (adjy, adjx) not in path: # only care about connected cells thats not in path
                        # found a connected neighbor
                        neighbors_index.append(direction)
            if len(neighbors_index) == 1 and neighbors_index[0] in diagonal_indices_8way:
                # we have a lonely cell only connected diagonally!
                diag_direction = neighbors_index[0]
                adjx = x + dx_8way[diag_direction]
                adjy = y + dy_8way[diag_direction]
                # move into the diagonal cell
                x = adjx
                y = adjy
                primary = get_primary(y, x)
                secondary = get_secondary(y, x)
                path.append((y, x))
        

            # Sweep in primary direction (untill hitting a boundary or visited cell)
            while (0 <= primary + primary_dir < primary_size): # break if hitting outer grid limits
                next_cell = convert_to_y_x(secondary, primary + primary_dir)
                if grid[next_cell] != 1 or next_cell in path: # break if hitting non-1 cell or visited cell
                    break
                # we are free to move - so we keep going
                primary += primary_dir # move along primary axis
                y, x = convert_to_y_x(secondary, primary)
                path.append((y, x)) # create path as we go

            # Sweep in opposite direction to ensure full coverage (in the same way as above)
            primary_dir = -1 * primary_dir # (we dont count this as a turn, since it might be the only direction the uav can move)
            while (0 <= primary + primary_dir < primary_size):
                next_cell = convert_to_y_x(secondary, primary + primary_dir)
                if grid[next_cell] != 1 or next_cell in path:
                    break
                primary += primary_dir
                y, x = convert_to_y_x(secondary, primary)
                path.append((y, x))


            ######## Find border cells in next line ########

            next_line_border_cells = []
            transit_primary = None

            # Move secondary to next line
            next_secondary = secondary + secondary_dir
            if not (0 <= next_secondary < secondary_size):
                lawnmower_state = 'sweep_in_1_direction_done'
                continue

            # Find a path to gain access to next row (no backtracking allowed, only L-shaped paths, only allowed to move in 1's)
            # (we call the x value where a path is possible is called "transit_primary")

            # Search left/backward for a valid transit_primary cell
            for p in range(primary, -1, -1):
                curr_cell = convert_to_y_x(secondary, p)
                next_cell = convert_to_y_x(next_secondary, p)
                if grid[curr_cell] == 0:  # we have gone too far left - stop looking
                    break
                if grid[next_cell] == 1:
                    # valid transit_primary found!
                    transit_primary = p
                    break

            # Search right/forward if needed
            if transit_primary is None:
                for p in range(primary, primary_size):
                    curr_cell = convert_to_y_x(secondary, p)
                    next_cell = convert_to_y_x(next_secondary, p)
                    if grid[curr_cell] == 0:
                        break
                    if grid[next_cell] == 1:
                        transit_primary = p
                        break

                    
            if transit_primary is not None:
                # now, find the border cells in the next line (from transit_primary)

                # Find "left" boundary cell 
                for p in range(transit_primary, -1, -1):
                    cell = convert_to_y_x(next_secondary, p)
                    if grid[cell] == 0:
                        # we hit a boundary. add the 1 cell of the boundary
                        next_line_border_cells.append(convert_to_y_x(next_secondary, p + 1))
                        break
                    elif p <= 0:
                        # we are at the edge of the grid. this has to be a boundary
                        next_line_border_cells.append(cell)
                        break

                # Find "right" boundary cell 
                for p in range(transit_primary, primary_size):
                    cell = convert_to_y_x(next_secondary, p)
                    if grid[cell] == 0:
                        next_line_border_cells.append(convert_to_y_x(next_secondary, p - 1))
                        break
                    elif p >= primary_size - 1:
                        next_line_border_cells.append(cell)
                        break
                # remove duplicates:
                next_line_border_cells = list(set(next_line_border_cells)) # (set does not allow duplicates)

            ######## Choose closest border cell ########
            min_dist = float('inf')
            next_cell = None
            for cell in next_line_border_cells:
                dist = abs(get_primary(*cell) - primary)
                if dist < min_dist:
                    min_dist = dist
                    next_cell = cell

            if next_cell is None or next_cell in path: # (if next_cell is in path, it is safe to asume that the row is already fully covered)
                # no valid next cell found - we are done with sweeping in this direction
                lawnmower_state = 'sweep_in_1_direction_done'
                continue

            ######## Commit to chosen cell ########
            y, x = next_cell
            path.append((y, x))

            ######## move the other way back ########
            primary_dir = -1 * primary_dir
            turn_count += 1

        elif lawnmower_state == 'sweep_in_1_direction_done':
            # If we are done sweeping in one direction, we need to check if we can sweep in the other direction
            if secondary_dir == front_move_direction:
                # let us sweep in the other direction now
                secondary_dir = -1 * secondary_dir # change front direction for next sweep
                lawnmower_state = 'sweep_in_1_direction'
            else: # we have done sweeping in both directions
                lawnmower_state = 'check_for_completion'
            continue

        elif lawnmower_state == 'check_for_completion':
            #print("Checking for completion...")
            # Check if all 1's in grid are covered in path
            all_covered = True
            for y_check in range(grid.shape[0]):
                for x_check in range(grid.shape[1]):
                    if (grid[y_check][x_check] == 1) and ((y_check, x_check) not in path):
                        all_covered = False
                        break
                if all_covered == False:
                    print("WARNING: path generation failed to cover all 1's in the grid.")
                    print(f"non covered cell found at: {y_check}, {x_check}")
                    break
            break

    end_cell = (y, x)

    # Compute the Euclidean distance of the path
    path_len_euclidean = sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))

    #print(f"Lawnmower path generated with {len(path)} cells, {turn_count} turns, and Euclidean length {path_len_euclidean:.2f}.")

    return path, path_len_euclidean, start_cell, end_cell, turn_count