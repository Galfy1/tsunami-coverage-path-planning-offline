import numpy as np
import math
from shapely.geometry import Point, Polygon


def meters_to_lat_long_dif(current_approx_lat, meters: float):
    # https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters 
    # The conversion from meters is not the same for latitude and longitude - we account for this here.

    EARTH_RADIUS = 6371000.0  # mean Earth radius in meters

    lat_diff = (meters / EARTH_RADIUS) * (180 / math.pi)
    lon_diff = (meters / EARTH_RADIUS) * (180 / math.pi) / math.cos(math.radians(current_approx_lat))

    return lat_diff, lon_diff


def find_home_cell(drone_start_coord, x_axis_coords, y_axis_coords, grid):

    home_cell = None
    for i in range(len(y_axis_coords) - 1):
        for j in range(len(x_axis_coords) - 1):
            y_cell_min, y_cell_max = sorted([y_axis_coords[i], y_axis_coords[i + 1]]) # with sorted it doesn’t matter whether your coordinate list is increasing or decreasing in value (we force it to be increasing so our checking logic below works)
            x_cell_min, x_cell_max = sorted([x_axis_coords[j], x_axis_coords[j + 1]])

            # Check if drone start is inside this cell
            if y_cell_min <= drone_start_coord[0] < y_cell_max and x_cell_min <= drone_start_coord[1] < x_cell_max:
                home_cell = (i, j)
                break
        if home_cell:
            break
    # Check if found
    if home_cell is None:
        raise ValueError("Drone start position is outside the grid")
    if grid[home_cell[0], home_cell[1]] == 0:
        raise ValueError("Drone start position is in a no-fly zone")
    
    return home_cell


def create_grid_from_polygon_and_noflyzones(polygon: Polygon, no_fly_zones: list[Polygon], drone_start_coord, camera_coverage_len):
    
    grid_res_y, grid_res_x = meters_to_lat_long_dif(drone_start_coord[0], camera_coverage_len)

    # Compute bounding box of polygon
    minx, miny, maxx, maxy = polygon.bounds

    # Generate pre-aligned x and y "grid" coords. (these coords are parallel with the lat/lon axes.)
    x_axis_coords = np.arange(minx, maxx, grid_res_x) 
    y_axis_coords = np.arange(miny, maxy, grid_res_y)

    #### Construct fly_nofly_grid #####
    # Create "grid" to keep track of which cells are inside the polygon and not in no-fly zone (0 = outside polygon or in no-fly zone, 1 = inside polygon and not in no-fly zone)

    # Create empty grid
    fly_nofly_grid = np.zeros((len(y_axis_coords), len(x_axis_coords)), dtype=int)
    
    # Fill grid from polygon
    for i, y in enumerate(y_axis_coords):
        for j, x in enumerate(x_axis_coords):
            point = Point(x, y)
            if polygon.contains(point):
                # Check if point is in any no-fly zone
                if any(no_fly_zone.contains(point) for no_fly_zone in no_fly_zones):
                    continue  # point is in a no-fly zone, leave as 0
                fly_nofly_grid[i, j] = 1


    #### Find grid cell corresponding to drone start position ####
    home_cell = find_home_cell(drone_start_coord, x_axis_coords, y_axis_coords, fly_nofly_grid)

    #print(f"home_cell: {home_cell}, grid value at home_cell: {fly_nofly_grid[home_cell[0], home_cell[1]]}")

    return fly_nofly_grid, home_cell, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y