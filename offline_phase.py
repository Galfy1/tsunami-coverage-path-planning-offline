


import os
import numpy as np
from shapely.geometry import Point, Polygon, LineString
import math
#from sympy import Add
from breadth_first_traversal import breadth_first_traversal, single_drone_traversal_order
import matplotlib.pyplot as plt
import pickle

import csv

DRONE_START = (37.4122067992952, -121.998909115791) # (lat, lon) aka (y,x)
ALLOW_DIAGONAL_IN_BFT = True 
ALLOW_DIAGONAL_IN_PATH_OFFLINE_PLOTTING = False # THIS IS JUST FOR PLOTTING IN THIS FILE ! For tsunami (for now) the setting is set in the online file.


def convert_cells_to_gps(cells, x_coords, y_coords):
    result_gps = []
    for (i, j) in cells:
        lat = y_coords[i]
        lon = x_coords[j]
        result_gps.append((lat, lon))

    # make sure the traversal order gps is in the middle of the cells:
    grid_res = x_coords[1] - x_coords[0]  # assuming uniform spacing. will be positive if x_coords is increasing, negative if decreasing
    result_gps_centered = []
    for (lat, lon) in result_gps:
        result_gps_centered.append((lat + (grid_res / 2), lon + (grid_res / 2)))

    return result_gps_centered



# def scale_linestring(linestring: LineString, scale: float) -> LineString:

#     # scalie equally in both directions from p1 to p2

#     p1 = linestring.coords[0]
#     p2 = linestring.coords[1]

#     angle = math.atan2(p2.y - p1.y, p2.x - p1.x)
#     length = linestring.length
#     new_length = length * scale
#     extend_length = (new_length - length) / 2
    
#     new_p1_x = p2.x - new_length * math.cos(angle)
#     new_p1_y = p2.y - new_length * math.sin(angle)
#     new_p2_x = p1.x + new_length * math.cos(angle)
#     new_p2_y = p1.y + new_length * math.sin(angle)
#     new_linestring = LineString([Point(new_p1_x, new_p1_y), Point(new_p2_x, new_p2_y)])
#     return new_linestring


def scale_linestring(linestring: LineString, scale: float) -> LineString:

    if len(linestring.coords) != 2:
        raise ValueError("This function only supports LineStrings with exactly two points")

    (x1, y1), (x2, y2) = linestring.coords

    # compute direction vector (cartesian form)
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy) # hypot is simply sqrt(dx*dx + dy*dy) (Pythagorean theorem)

    if length == 0:
        raise ValueError("Cannot scale a zero-length line")

    # normalize direction vector (i.e. scale it down to length 1) - this way, dx_normalized and dy_normalized represents how much of the length is in x and y direction respectively
    dx_normalized = dx / length
    dy_normalized = dy / length

    # compute how much to extend each end
    new_length = length * scale
    extend_length = (new_length - length) / 2

    # move each endpoint along the direction vector
    # (e.g. (dx_normalized * extend_length) is basically saying "how much of the extend_length is in the x direction")
    # this will work for linestrings in all directions (the sign of the direction vector components will take care of that)
    new_p1 = (x1 - dx_normalized * extend_length, y1 - dy_normalized * extend_length) # "-" is used to move point "backwards" along the direction vector
    new_p2 = (x2 + dx_normalized * extend_length, y2 + dy_normalized * extend_length) # "+" is used to move point "forwards" along the direction vector

    return LineString([new_p1, new_p2])
    
    
def extend_p2_in_linestring(linestring: LineString, extend_length: float) -> LineString:
    # if len(linestring.coords) != 2:
    #     raise ValueError("This function only supports LineStrings with exactly two points")

    # (x1, y1), (x2, y2) = linestring.coords

    # # compute direction vector (cartesian form)
    # dx = x2 - x1
    # dy = y2 - y1
    # length = math.hypot(dx, dy) # hypot is simply sqrt(dx*dx + dy*dy) (Pythagorean theorem)

    # if length == 0:
    #     raise ValueError("Cannot extend a zero-length line")

    # # normalize direction vector (i.e. scale it down to length 1) - this way, dx_normalized and dy_normalized represents how much of the length is in x and y direction respectively
    # dx_normalized = dx / length
    # dy_normalized = dy / length

    # # move endpoint p2 along the direction vector
    # new_p2 = (x2 + dx_normalized * extend_length, y2 + dy_normalized * extend_length) # "+" is used to move point "forwards" along the direction vector

    # return LineString([ (x1, y1), new_p2 ])

    pass


# https://www.matematikbanken.dk/id/158/ 

def align_coords_with_centroid_angle(polygon: Polygon, home_gps, x_coords, y_coords, grid_res):

    aligned_coords = []

    # https://en.wikipedia.org/wiki/Centroid#Of_a_polygon 
    # https://stackoverflow.com/questions/75699024/finding-the-centroid-of-a-polygon-in-python 
    # https://shapely.readthedocs.io/en/stable/reference/shapely.Polygon.html#shapely.Polygon.centroid  
    centroid = polygon.centroid  # Point(lon, lat)
    centroid_line = LineString([Point(home_gps[1], home_gps[0]), centroid]) # Point(lon, lat)
    long_centroid_line = scale_linestring(centroid_line, 20) # make it longer in both directions to ensure it crosses the entire polygon. 20 is arbitrary, just needs to be large enough.


    # find linjen der går fra ppunktet og vinkelret ind til centroid linjen.
    # (måske kan sharely bruges til dette)
    # "scaler" puntet væk fra centroid linjen, så den snapper til multipla af grid resolution


    for x_coord in x_coords:
        for y_coord in y_coords:

            gps_point = Point(x_coord, y_coord)

            closest_point_on_centroid_line = long_centroid_line.interpolate(long_centroid_line.project(gps_point))
            direction_line = LineString([gps_point, closest_point_on_centroid_line])
            direction_line_length = direction_line.length

            # Figure out how much to extend point to snap to grid res
            extend_amount = round(direction_line_length / grid_res) * grid_res

            extended_direction_line = # BRUG function der extender p2 i linestring med extend_amount (tjek lige at grid punktet er p2. og ikke punket på centroid linjen)






    # for coord in coords:
    #     point = Point(coord)
    #     angle = math.atan2(point.y - centroid.y, point.x - centroid.x)
    #     distance = point.distance(centroid)
    #     new_x = centroid.x + distance * math.cos(angle)
    #     new_y = centroid.y + distance * math.sin(angle)
    #     aligned_coords.append((new_x, new_y))

    return aligned_coords






def main(args=None) -> None:


    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    with open('baylands_polygon_v3.poly','r') as f: 
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            if(row[0] == '#saved'): continue # skip header
            polygon_coords.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    print("''''''''''''''''''''''''''")
    print(polygon_coords)
    print("''''''''''''''''''''''''''")
    polygon = Polygon(polygon_coords)

    no_fly_zone = []
    with open('no_fly_zone.poly','r') as f:
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            if(row[0] == '#saved'): continue # skip header
            no_fly_zone.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    # Convert no-fly zone coordinates to a Shapely polygon
    no_fly_zone_polygon = Polygon(no_fly_zone)

    # Example polygon (GPS coordinates)
    # polygon_coords = [
    #     (12.490, 41.890),  # Example near Rome
    #     (12.492, 41.890),
    #     (12.492, 41.892),
    #     (12.490, 41.892),
    #     (12.490, 41.890)
    # ]



    # make sure polygon is convex
    if(polygon.equals(polygon.convex_hull) == False):
        raise ValueError("Polygon must be convex")
    if(no_fly_zone_polygon.equals(no_fly_zone_polygon.convex_hull) == False): # TODO, NOT SURE IF THIS IS REQURED
        raise ValueError("No-fly zone polygon must be convex")



    # Define grid resolution (degrees)
    #grid_res = 0.00005  # TODO NOT TRUE: I CHANGE IT - ~11 meters // https://lucidar.me/en/online-unit-converter-length-to-angle/convert-degrees-to-meters/ (earth radius = 6373000 m) 
    grid_res = 0.0001

    # Compute bounding box of polygon
    minx, miny, maxx, maxy = polygon.bounds

    # Generate grid points # TODO DE KAN OGSÅ BRUGES TIL AT CONVERT TILBAGE TIL LAT LONG
    x_coords = np.arange(minx, maxx, grid_res) 
    y_coords = np.arange(miny, maxy, grid_res)

    # Create empty grid
    grid = np.zeros((len(y_coords), len(x_coords)), dtype=int)

    # Fill grid where point lies inside polygon
    for i, y in enumerate(y_coords):
        for j, x in enumerate(x_coords):
            point = Point(x, y)
            if polygon.contains(point) and not no_fly_zone_polygon.contains(point):
                grid[i, j] = 1



    # Find grid cell corresponding to drone start position
    home_cell = None
    for i in range(len(y_coords) - 1):
        for j in range(len(x_coords) - 1):
            y_cell_min, y_cell_max = sorted([y_coords[i], y_coords[i + 1]]) # with sorted it doesn’t matter whether your coordinate list is increasing or decreasing in value (we force it to be increasing so our checking logic below works)
            x_cell_min, x_cell_max = sorted([x_coords[j], x_coords[j + 1]])

            # Check if drone start is inside this cell
            if y_cell_min <= DRONE_START[0] < y_cell_max and x_cell_min <= DRONE_START[1] < x_cell_max:
                home_cell = (i, j)
                print("DRONE START GRID COORDS:", home_cell)
                break
        if home_cell:
            break
    # Check if found
    if home_cell is None:
        raise ValueError("Drone start position is outside the grid")
    if grid[home_cell[0], home_cell[1]] == 0:
        raise ValueError("Drone start position is in a no-fly zone")
    print("DRONE START GRID COORDS:", home_cell)





    # #print(grid)
    # traversal_order_cells = traversal_order(grid, home_cell[0], home_cell[1], allow_diagonal=True) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
    # #print(traversal_order_cells)
    # traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_coords, y_coords)
    # print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)

    bf_traversal_cells = breadth_first_traversal(grid, home_cell[0], home_cell[1], allow_diagonal=ALLOW_DIAGONAL_IN_BFT)
    bf_traversal_gps = convert_cells_to_gps(bf_traversal_cells, x_coords, y_coords)


    data_to_save = {
        'home_cell': home_cell,
        'home_gps': DRONE_START,
        'bf_traversal_cells': bf_traversal_cells,
        'bf_traversal_gps': bf_traversal_gps,
    }
    # Save traversal order to a file using pickle
    with open('bf_traversal.pkl', 'wb') as fp:
        pickle.dump(data_to_save, fp)

    #print(grid)
    #print("Grid shape:", grid.shape)

    # # heatmap plot
    # heatmap = np.zeros((len(y_coords), len(x_coords)), dtype=int)
    # for idx, (i, j) in enumerate(traversal_order_cells):
    #     heatmap[i, j] = idx + 1  # Start from 1 for better visibility
    # plt.imshow(heatmap, origin="lower", cmap="hot", interpolation='nearest')
    # plt.show()

    # TODO, VI SKAL LIGE VÆLGE OM ORIGIN I VOReS GRID I OPPE I VENSTRE HJØRNE ELLER NEDRE VENSTRE HJØRNE
    # NÅR VI PRINTER GRID, SER DEN UD PÅ EN MÅDE (forket i forhold til missionplanner)
    # NÅR VI PRITNER GRID MED origin="lower" SER DEN UD PÅ EN ANDEN MÅDE (rigtig i forhold til missionplanner)
    # DET SKAL VI LIGE FINDE UD AF HVAD DER SKER DER
    # MEN DET GØR MÅSKE IKKE NOGET AT ARRAYED ER FLIPPED NÅR VI PLOTTER DET (det er bare sådan python plotter numpy array tænker jeg)









    # for i, y in enumerate(y_coords):
    #     if i == 0: x_prev = 0 # reset x_prev at start of new row
    #     for j, x in enumerate(x_coords):
    #         if j == 0: y_prev = 0 # reset y_prev at start of new column
    #         if(DRONE_START[1] < x and DRONE_START[1] > x_prev and DRONE_START[0] < y and DRONE_START[0] > y_prev): # (lat, lon) aka (y,x)
    #             print("DRONE START GRID COORDS: ", (i,j))
    #             home_cell = (i,j)
    #             break
    #         x_prev = x
    #     y_prev = y
    # # Check if home_cell is valid
    # if grid[home_cell[0], home_cell[1]] == 0:
    #     raise ValueError("Drone start position is not within the polygon or is in a no-fly zone")
    # print("DRONE START GRID COORDS: ", home_cell)








    # PLOTTING SINGLE DRONE PATH ON A MAP (or just BFT points if you want)
    PLOT_TYPE = 'bft_and_path' # "just_bft" or "bft_and_path"


    traversal_order_cells = single_drone_traversal_order(grid, home_cell[0], home_cell[1], allow_diagonal_in_bft=ALLOW_DIAGONAL_IN_BFT, allow_diagonal_in_path=ALLOW_DIAGONAL_IN_PATH_OFFLINE_PLOTTING) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
    #print(traversal_order_cells)
    traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_coords, y_coords)
    print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)


    import folium
    import matplotlib.cm as cm
    import matplotlib.colors as colors
    from matplotlib import colormaps
    # List of coordinates (using a shortened sample for demonstration; will replace with full list)
    

    # Center the map around the average of coordinates
    avg_lat = sum(lat for lat, lon in traversal_order_gps) / len(traversal_order_gps)
    avg_lon = sum(lon for lat, lon in traversal_order_gps) / len(traversal_order_gps)

    # Create the map
    m = folium.Map(location=[avg_lat, avg_lon], zoom_start=18)

    # Set up a colormap
    amount_of_colored_points = len(bf_traversal_gps)  # You can change this to a lower number if you want to "zoom in" on the colormap
    #amount_of_colored_points = 60
    cmap = cm.get_cmap('inferno', amount_of_colored_points)  # You can change colormap to what you want

    # Plot BF traversal points
    for i, (lat, lon) in enumerate(bf_traversal_gps):
        # (Add points to the map with colors based on their order)
        color = colors.rgb2hex(cmap(i)[:3]) # Convert RGBA to hex
        folium.CircleMarker(location=[lat, lon], radius=5, color=color, fill=True, fill_opacity=0.7).add_to(m)

    if (not (PLOT_TYPE == 'just_bft')):
        # Plot traversal order path
        folium.PolyLine(locations=traversal_order_gps, color="blue", weight=2.0, opacity=1, ).add_to(m)
        pass


    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script directory
    map_path = os.path.join(script_dir, "map.html")          # Set filename
    m.save(map_path)














    # plt.imshow(grid, origin="lower", cmap="Greens")
    # plt.title("Polygon Rasterized to Grid")
    # plt.show()


    #Link til wavefront "breath first traversal (BFS)":
    # https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/
    # tænker bare vi bruger den order som BFS traversal returner.
        # lige nu printer den grid elementerne. vi vil lave den så den returner en liste af tuples med koordinaterne i x,y i griddet.
    # og så har vi samtidligt vores grid (hver element bool) som holder styr på om hver celle skal besøges eller ej (skal ikke være i no-fly-zone/obstacle eller allerede besøgt).
    # dronen kigger på på traversal listen og tjekker om næste punkt er valid udfra vores grid


    # Vi skal også have noget der converter fra grid koordinater til GPS koordinater, til når dronen rent faktisk skal flyve hen til et punkt.











if __name__ == '__main__':
    main()