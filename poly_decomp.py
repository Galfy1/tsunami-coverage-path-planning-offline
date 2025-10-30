
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math
from collections import deque as queue

# reuse code from offline_phase.py
from offline_phase import create_grid_from_polygon_and_noflyzones


DRONE_START = (37.4135766590003, -121.997506320477) # (lat, lon) aka (y,x)
CAMERA_COVERAGE_LEN = 4 # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)






def main(args=None) -> None:

    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    with open('irregular_poly.poly','r') as f: 
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            if(row[0] == '#saved'): continue # skip header
            polygon_coords.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    polygon = Polygon(polygon_coords)
    if not polygon.is_valid:
        raise ValueError("Polygon is not valid")

    # This decomp method does not allow for holes (i.e. no "no fly zones" inside the polygon)

    fly_grid, home_cell, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y = create_grid_from_polygon_and_noflyzones(
                                                                                        polygon, [], DRONE_START, CAMERA_COVERAGE_LEN)
    # (fly_grid is a 2D numpy array where 1 = flyable, 0 = no-fly zone - (y,x) indexing)



    # go though each "candidate sweep line" and check for non-monotone sections

    all_sub_grids = []

    grid_queue = queue() # queue of grid/subgrids to be processed
    grid_queue.append(fly_grid)

    while(len(grid_queue) > 0):

        grid = grid_queue.popleft()

        non_monotone_sweep_lines = []

        # go through all horizontal sweep lines:
        p1_x = 0
        p2_x = len(x_axis_coords)

        non_monotone_in_y = False
        non_monotone_in_x = False

        for y in range(grid.shape[0]):
            intersection_points = []
            # check intersection with grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
            val = 0
            for x in range(grid.shape[1]):
                cell_val = grid[y][x]
                if cell_val != val:
                    # transition detected
                    intersection_points.append( (y,x) )
                    val = cell_val
            
            #print(f"leng: {len(intersection_points)}")
            if len(intersection_points) > 2:
                # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
                sweep_line = LineString([ (p1_x, y), (p2_x, y) ])
                gap_severity = 0
                for i in range(1, len(intersection_points)-1, 2):
                    start_pt = intersection_points[i]
                    end_pt = intersection_points[i+1]
                    gap_severity += abs(end_pt[0] - start_pt[0]) # y coordinate difference
                    pass

                non_monotone_sweep_lines.append( (sweep_line, intersection_points, gap_severity) )
                non_monotone_in_y = True


        # go through all vertical sweep lines:

        p1_y = 0
        p2_y = len(y_axis_coords)

        for x in range(grid.shape[1]):
            intersection_points = []
            # check intersection with grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
            val = 0
            for y in range(grid.shape[0]):
                cell_val = grid[y][x]
                if cell_val != val:
                    # transition detected
                    intersection_points.append( (y,x) )
                    val = cell_val
            
            #print(f"leng: {len(intersection_points)}")
            if len(intersection_points) > 2:
                # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
                sweep_line = LineString([ (x, p1_y), (x, p2_y) ])
                gap_severity = 0
                for i in range(1, len(intersection_points)-1, 2):
                    start_pt = intersection_points[i]
                    end_pt = intersection_points[i+1]
                    gap_severity += abs(end_pt[1] - start_pt[1]) # x coordinate difference

                non_monotone_sweep_lines.append( (sweep_line, intersection_points, gap_severity) )
                non_monotone_in_x = True

        # Check if "polygon" is irregular (i.e., non–monotone in both directions)
        if non_monotone_in_x and non_monotone_in_y:
            print("Polygon is irregular (non-monotone in both directions)")
            # We need to split!

            # Find candidate sweep line with largest gap severity
            max_gap_severity = -1
            selected_sweep_line = None
            for sweep_line, _ , gap_severity in non_monotone_sweep_lines:
                if gap_severity > max_gap_severity:
                    max_gap_severity = gap_severity
                    selected_sweep_line = sweep_line

            # Split the grid along the selected sweep line into two sub-grids
            if selected_sweep_line.coords[0][0] == selected_sweep_line.coords[1][0]:  # horizontal line (if p1 y matches p2 y)
                split_y = int(selected_sweep_line.coords[0][1]) # "height"(y) if horizontal line
                # create sub-grids (same dimensions as original grid, but with 0s outside the sub-area
                sub_grid1 = np.copy(grid)
                sub_grid1[split_y:,:] = 0
                sub_grid2 = np.copy(grid)
                sub_grid2[:split_y,:] = 0
            else:  # vertical line
                split_x = int(selected_sweep_line.coords[0][0]) # "width"(x) if vertical line
                sub_grid1 = np.copy(grid)
                sub_grid1[:, split_x:] = 0
                sub_grid2 = np.copy(grid)
                sub_grid2[:, :split_x] = 0

            # add sub-grids to queue for further processing
            grid_queue.append(sub_grid1)
            grid_queue.append(sub_grid2)

        else:
            print("Polygon is regular (monotone in at least one direction)")
            # STOP HER!!! VI ER DONE FOR NU



    print(f"Selected sweep line for splitting: {selected_sweep_line} with gap severity {max_gap_severity}")

    # Plot grid and sweep lines for visualization:
    import matplotlib.pyplot as plt
    plt.imshow(fly_grid, cmap='Greys', origin='lower')
    # for sweep_line, points in non_monotone_sweep_lines:
    #     x_coords = [p[1] for p in points]
    #     plt.plot(x_coords, [sweep_line.y] * len(x_coords), color='red')
    plt.show()

    # # Now process the non-monotone sweep lines to find the actual non-monotone sections
    # non_monotone_sections = []
    # for sweep_line, points in non_monotone_sweep_lines:
    #     points.sort(key=lambda p: p[0])  # Sort by x coordinate
    #     for i in range(len(points)-1):
    #         current_end = points[i+1]
    #         next_start = points[i+2] if i+2 < len(points) else None
    #         if next_start is not None and next_start < current_end:
    #             non_monotone_sections.append((sweep_line, current_end, next_start)) # (sweep line, current end, next start)

if __name__ == '__main__':
    main()