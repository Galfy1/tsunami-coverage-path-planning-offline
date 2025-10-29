
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math

# reuse code from offline_phase.py
from offline_phase import create_grid_from_polygon_and_noflyzones


DRONE_START = (37.4122067992952, -121.998909115791) # (lat, lon) aka (y,x)
CAMERA_COVERAGE_LEN = 10  # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)






def main(args=None) -> None:
    pass

    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    with open('baylands_polygon_v3.poly','r') as f: 
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

    non_monotone_sweep_lines = []

    # go through all horizontal sweep lines:
    p1_x = 0
    p2_x = len(x_axis_coords)

    non_monotone_in_x = False
    non_monotone_in_x = False

    for y in range(fly_grid.shape[0]):
        intersection_points = []
        # check intersection with fly_grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
        val = 0
        for x in range(fly_grid.shape[1]):
            cell_val = fly_grid[y][x]
            if cell_val != val:
                # transition detected
                intersection_points.append( (y,x) )
                val = cell_val
            
        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            sweep_line = LineString([ (p1_x, y_axis_coords[y]), (p2_x, y_axis_coords[y]) ])
            gap_severity = 0
            # for i in range(0, len(intersection_points)-1, 2):
            #     start_pt = intersection_points[i]
            #     end_pt = intersection_points[i+1]
            #     gap_severity += abs(end_pt[0] - start_pt[0]) + abs(end_pt[1] - start_pt[1])
                
            non_monotone_sweep_lines.append( (sweep_line, intersection_points) )
            non_monotone_in_y = True

    # go through all vertical sweep lines:

    # Check if "polygon" is irregular (i.e., non–monotone in both directions)
    if non_monotone_in_x and non_monotone_in_y:
        print("Polygon is irregular (non-monotone in both directions)")
        # FIND SWEEP LINE MED STØRSTE GAP - OG SPLIT POLYGON HER.. OG GØR ALT IGEN INDTIL (non_monotone_in_x and non_monotone_in_y) ER FALSE
    else:
        print("Polygon is monotone in at least one direction")
        # STOP HER!!! VI ER DONE FOR NU


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