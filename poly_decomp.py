
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point
import numpy as np
import math

# reuse code from offline_phase.py
from offline_phase import meters_to_lat_long_dif, find_home_cell


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


    grid_res_y, grid_res_x = meters_to_lat_long_dif(DRONE_START[0], CAMERA_COVERAGE_LEN)

    # Compute bounding box of polygon
    minx, miny, maxx, maxy = polygon.bounds

    # Generate pre-aligned x and y "grid" coords. (these coords are parallel with the lat/lon axes.)
    x_axis_coords = np.arange(minx, maxx, grid_res_x) 
    y_axis_coords = np.arange(miny, maxy, grid_res_y)

    #### Construct fly_grid #####

    # Create empty grid
    fly_grid = np.zeros((len(y_axis_coords), len(x_axis_coords)), dtype=int)
    
    # Fill grid from polygon
    for i, y in enumerate(y_axis_coords):
        for j, x in enumerate(x_axis_coords):
            point = Point(x, y)
            if polygon.contains(point):
                fly_grid[i, j] = 1



if __name__ == '__main__':
    main()