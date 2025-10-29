
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point
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



if __name__ == '__main__':
    main()