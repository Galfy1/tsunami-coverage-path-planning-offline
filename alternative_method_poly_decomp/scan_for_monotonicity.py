from shapely.geometry import Polygon, Point, LineString
import numpy as np
from alternative_method_poly_decomp.shared_grid_tools import split_grid_along_sweep_line


# NOTE: if allow_valid_monotone is set to True, then sweep lines that result in two regular partitions will 
#       also be considered "non-monotone" sweep lines (even though they are technically monotone sweep lines) - (yes, its a bit misleading naming wise)
def scan_for_non_monotone_sections(grid: np.ndarray, allow_valid_monotone: bool = False):

    non_monotone_sweep_lines = []
    non_monotone_in_x = False
    non_monotone_in_y = False

    # go through all horizontal sweep lines:
    p1_x = 0
    p2_x = grid.shape[1]
    for y in range(grid.shape[0]):
        intersection_points = []
        # check intersection with grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
        val = 0
        for x in range(grid.shape[1]):
            cell_val = grid[y][x]
            if cell_val != val:
                # transition detected
                intersection_points.append((y, x))
                val = cell_val
        # account for edge case where sweep line ends on a 1 cell
        if grid[y][grid.shape[1]-1] == 1:
            if len(intersection_points) == 0 or intersection_points[-1] != (y, grid.shape[1]-1): # if not already added
                intersection_points.append((y, grid.shape[1]-1))


        #print(f"leng: {len(intersection_points)}")
        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            #sweep_line = LineString([(p1_x, y), (p2_x, y)])
            gap_severity = 0
            for i in range(1, len(intersection_points) - 1, 2):
                start_pt = intersection_points[i]
                end_pt = intersection_points[i + 1]
                gap_severity += abs(end_pt[1] - start_pt[1])  # x coordinate difference
                pass

            # because  of the nature of the grid.. we cant split exactly on the line... so we have to split just above or below it..
            # (excessive splitting will be corrected later in the "culling merging" step)
            above_line = LineString([(p1_x, y + 1), (p2_x, y + 1)])
            below_line = LineString([(p1_x, y), (p2_x, y)])
            non_monotone_sweep_lines.append((above_line, intersection_points, gap_severity))
            non_monotone_sweep_lines.append((below_line, intersection_points, gap_severity))

            non_monotone_in_y = True
        elif len(intersection_points) == 2:
            if allow_valid_monotone:
                # Note: This might result in some duplicate entries in non_monotone_sweep_lines.
                #       However, its faster to allow duplicates here and handle them later if needed, than to check for duplicates here - so we dont.

                # TODO nu ved jeg hvorfor den gør alt så langsomt... det fordi scan_for_non_monotone_sections også bliver brugt i culling_merging() og lawnmower()
                                # det kan faktisk løses! for de behøver ikke de her ting vi tilføjer... aka, eventuelt lav et input flag til at "allow_asd" der slår det til of fra... for vi behøver det faktisk kun i poly_decomp_and_path_plan.py

                # TODO  den version hvor jeg ikke early breaker virkede ... den her virker ikke for whatever reason..

                # while(True): # this is just to allow easy early breaking for faster processing
            
                above_line = LineString([(p1_x, y + 1), (p2_x, y + 1)])
                sub_grids = split_grid_along_sweep_line(grid, above_line)
                if above_line == LineString([(0,45), (83,45)]): # TODO fjern
                    print("asdasd")
                if len(sub_grids) == 2:
                    while(True): # this is just to allow easy early breaking for faster processing
                        # check if both resulting sub-grids are "regular" (i.e. monotone in either x or y)
                        _, is_regular_0, _, _ = scan_for_monotone_sections(sub_grids[0])
                        if is_regular_0 == False: break # early exit
                        _, is_regular_1, _, _ = scan_for_monotone_sections(sub_grids[1])
                        if is_regular_1 == False: break # early exit
                        # if is_regular_0 and is_regular_1:
                            #print(f"MONOTONE: {above_line}")
                        non_monotone_sweep_lines.append((above_line, intersection_points, float('inf')))
                        break

                below_line = LineString([(p1_x, y), (p2_x, y)])
                sub_grids = split_grid_along_sweep_line(grid, below_line)
                if len(sub_grids) == 2:
                    while(True): # this is just to allow easy early breaking for faster processing
                        # check if both resulting sub-grids are "regular" (i.e. monotone in either x or y)
                        _, is_regular_0, _, _ = scan_for_monotone_sections(sub_grids[0])
                        if is_regular_0 == False: break # early exit
                        _, is_regular_1, _, _ = scan_for_monotone_sections(sub_grids[1])
                        if is_regular_1 == False: break # early exit
                        # if is_regular_0 and is_regular_1:
                            #print(f"MONOTONE: {below_line}")
                        non_monotone_sweep_lines.append((below_line, intersection_points, float('inf')))
                        break

                # TODO non_monotone_sweep_lines og scan_for_non_monotone_sections skal have nye navne hvis det her virker


    # go through all vertical sweep lines:
    p1_y = 0
    p2_y = grid.shape[0]
    for x in range(grid.shape[1]):
        intersection_points = []
        # check intersection with grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
        val = 0
        for y in range(grid.shape[0]):
            cell_val = grid[y][x]
            if cell_val != val:
                # transition detected
                intersection_points.append((y, x))
                val = cell_val
        # account for edge case where sweep line ends on a 1 cell
        if grid[grid.shape[0]-1][x] == 1:
            if len(intersection_points) == 0 or intersection_points[-1] != (grid.shape[0]-1, x): # if not already added
                intersection_points.append((grid.shape[0]-1, x))


        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            #sweep_line = LineString([(x, p1_y), (x, p2_y)])
            gap_severity = 0
            for i in range(1, len(intersection_points) - 1, 2):
                start_pt = intersection_points[i]
                end_pt = intersection_points[i + 1]
                gap_severity += abs(end_pt[0] - start_pt[0])  # y coordinate difference
                pass

            # because  of the nature of the grid.. we cant split exactly on the line... so we have to split just left or right of it..
            # (excessive splitting will be corrected later in the "culling merging" step)
            left_line = LineString([(x, p1_y), (x, p2_y)])
            right_line = LineString([(x + 1, p1_y), (x + 1, p2_y)])
            non_monotone_sweep_lines.append((left_line, intersection_points, gap_severity))
            non_monotone_sweep_lines.append((right_line, intersection_points, gap_severity))

            non_monotone_in_x = True
        elif len(intersection_points) == 2:
            if allow_valid_monotone:
                # Note: This might result in some duplicate entries in non_monotone_sweep_lines.
                #       However, its faster to allow duplicates here and handle them later if needed, than to check for duplicates here - so we dont.
            
                # while(True): # this is just to allow easy early breaking for faster processing
                above_line = LineString([(p1_x, y + 1), (p2_x, y + 1)])
                sub_grids = split_grid_along_sweep_line(grid, above_line)
                if len(sub_grids) == 2:
                    while(True): # this is just to allow easy early breaking for faster processing
                        # check if both resulting sub-grids are "regular" (i.e. monotone in either x or y)
                        _, is_regular_0, _, _ = scan_for_monotone_sections(sub_grids[0]) 
                        if is_regular_0 == False: break # early exit
                        _, is_regular_1, _, _ = scan_for_monotone_sections(sub_grids[1])
                        if is_regular_1 == False: break # early exit
                        # if is_regular_0 and is_regular_1:
                            #print(f"MONOTONE: {above_line}")
                        non_monotone_sweep_lines.append((above_line, intersection_points, float('inf')))
                        break

                below_line = LineString([(p1_x, y), (p2_x, y)])
                sub_grids = split_grid_along_sweep_line(grid, below_line)
                if len(sub_grids) == 2:
                    while(True): # this is just to allow easy early breaking for faster processing
                        # check if both resulting sub-grids are "regular" (i.e. monotone in either x or y)
                        _, is_regular_0, _, _ = scan_for_monotone_sections(sub_grids[0])
                        if is_regular_0 == False: break # early exit
                        _, is_regular_1, _, _ = scan_for_monotone_sections(sub_grids[1])
                        if is_regular_1 == False: break # early exit
                        # if is_regular_0 and is_regular_1:
                            #print(f"MONOTONE: {below_line}")
                        non_monotone_sweep_lines.append((below_line, intersection_points, float('inf')))
                        break 

                # TODO non_monotone_sweep_lines og scan_for_non_monotone_sections skal have nye navne hvis det her virker

    is_irregular = non_monotone_in_x and non_monotone_in_y
 
    return non_monotone_sweep_lines, is_irregular, non_monotone_in_x, non_monotone_in_y



def scan_for_monotone_sections(grid: np.ndarray):
    # (see scan_for_non_monotone_sections() for more comments)
    monotone_sweep_lines = []
    monotone_in_x = True
    monotone_in_y = True

    p1_x = 0
    p2_x = grid.shape[1]
    for y in range(grid.shape[0]):
        intersection_points = []
        val = 0
        for x in range(grid.shape[1]):
            cell_val = grid[y][x]
            if cell_val != val:
                intersection_points.append((y, x))
                val = cell_val
        if grid[y][grid.shape[1]-1] == 1:
            if len(intersection_points) == 0 or intersection_points[-1] != (y, grid.shape[1]-1): # if not already added
                intersection_points.append((y, grid.shape[1]-1))
        if len(intersection_points) <= 2:
            # Again.. because of the nature of the grid.. we cant split exactly on the line...
            # However - Since we dont use scan_for_monotone_sections() in a scenario where we would 
            #           expect subgrids to always be monotone if splitting a grid along this line, we simply ignore this issue here.
            #           That is, the issue does not matter in our case, so we simply split the line as is.
            sweep_line = LineString([(p1_x, y), (p2_x, y)])
            monotone_sweep_lines.append((sweep_line, intersection_points, 0))
        else:
            monotone_in_y = False

    p1_y = 0
    p2_y = grid.shape[0]
    for x in range(grid.shape[1]):
        intersection_points = []
        val = 0
        for y in range(grid.shape[0]):
            cell_val = grid[y][x]
            if cell_val != val:
                intersection_points.append((y, x))
                val = cell_val
        if grid[grid.shape[0]-1][x] == 1:
            if len(intersection_points) == 0 or intersection_points[-1] != (grid.shape[0]-1, x): # if not already added
                intersection_points.append((grid.shape[0]-1, x))
        if len(intersection_points) <= 2:
            sweep_line = LineString([(x, p1_y), (x, p2_y)])
            monotone_sweep_lines.append((sweep_line, intersection_points, 0))
        else:
            monotone_in_x = False

    is_regular = monotone_in_x or monotone_in_y
    return monotone_sweep_lines, is_regular, monotone_in_x, monotone_in_y
