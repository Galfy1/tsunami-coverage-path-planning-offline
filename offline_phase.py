


import os
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import math
#from sympy import Add
from breadth_first_traversal import breadth_first_traversal, single_drone_traversal_order
import matplotlib.pyplot as plt
import pickle

import csv

DRONE_START = (37.4122067992952, -121.998909115791) # (lat, lon) aka (y,x)
CAMERA_COVERAGE_LEN = 10  # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)
ALLOW_DIAGONAL_IN_BFT = False 
ALLOW_DIAGONAL_IN_PATH_OFFLINE_PLOTTING = True # THIS IS JUST FOR PLOTTING IN THIS FILE ! For tsunami (for now) the setting is set in the online file.
ENABLE_CENTROID_ALIGNMENT = False # (warning: each waypoint may be shifted up to +- grid_res/2 in both lat and lon direction when this is enabled. this might push the waypoint slightly outside polygon or in a no-fly zone)

debug_counter = 0 # TODO remove
debug_point = Point(0,0) # TODO remove
debug_line = LineString() # TODO remove

def convert_cells_to_gps(cells, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y, enable_centroid_alignment=False, polygon=None):

    # Add half grid res to get to center of cell, instead of just the lower left corner
    #grid_res = x_axis_coords[1] - x_axis_coords[0]  # assuming uniform spacing. will be positive if x_axis_coords is increasing, negative if decreasing
    y_axis_coords_cent = y_axis_coords + (grid_res_y / 2)
    x_axis_coords_cent = x_axis_coords + (grid_res_x / 2)

    # Convert cells to gps coords
    result_gps_centered = []
    for (i, j) in cells:
        lat = y_axis_coords_cent[i]
        lon = x_axis_coords_cent[j]
        result_gps_centered.append((lat, lon))

    # Align coords with centroid angle (if enabled)
    if enable_centroid_alignment:

        # NOTE: this done here, instead of early in the offline phase, as the aligned coords would otherwise break the grid structure used for BFS traversal

        if polygon is None:
            raise ValueError("Polygon must be provided when centroid alignment is enabled")

        info_for_plotting = {}
        prealigned_to_aligned_coords_dict, info_for_plotting["centroid"], info_for_plotting["centroid_line"], info_for_plotting["long_centroid_line"] = align_coords_with_centroid_angle(polygon, DRONE_START, x_axis_coords_cent, y_axis_coords_cent, grid_res_x, grid_res_y) 

        # now convert to aligned coords
        result_gps_aligned = []
        for (lat, lon) in result_gps_centered:
            aligned_coord = prealigned_to_aligned_coords_dict.get((lat, lon)) # coords are stored as (lat, lon) in the dict
            if aligned_coord is None:
                raise ValueError(f"Could not find aligned coord for pre-aligned coord ({lon}, {lat})")
            result_gps_aligned.append((aligned_coord[0], aligned_coord[1])) 
        return result_gps_aligned, info_for_plotting
        
    return result_gps_centered, None



# def convert_cells_to_gps_aligned(cells, aligned_coords):
#     result_gps = []
#     for (i, j) in cells:


        

        # lat = y_coords[i]
        # lon = x_coords[j]
        # result_gps.append((lat, lon))



    # TODO DET NEDENSTÅENDE SKAL NOK I SIN EGEN FUNCTIO NOG KALDES ET HELT ANDET STED
    # # make sure the traversal order gps is in the middle of the cells:
    # grid_res = x_coords[1] - x_coords[0]  # assuming uniform spacing. will be positive if x_coords is increasing, negative if decreasing
    # result_gps_centered = []
    # for (lat, lon) in result_gps:
    #     result_gps_centered.append((lat + (grid_res / 2), lon + (grid_res / 2)))

    return result_gps_centered


def _get_line_properties(linestring: LineString):
    """Extracts useful geometric info from a two-point LineString."""
    if len(linestring.coords) != 2:
        raise ValueError("This function only supports LineStrings with exactly two points")

    (x1, y1), (x2, y2) = linestring.coords

    # compute direction vector (cartesian form)
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)

    if length == 0:
        raise ValueError("Cannot operate on a zero-length line")

    # normalize direction vector (i.e. scale it down to length 1) - this way, dx_normalized and dy_normalized represents how much of the length is in x and y direction respectively
    dx_normalized = dx / length
    dy_normalized = dy / length

    return (x1, y1), (x2, y2), dx_normalized, dy_normalized, length


def scale_linestring(linestring: LineString, scale: float) -> LineString:
    (x1, y1), (x2, y2), dx_n, dy_n, length = _get_line_properties(linestring) # (dx_n means "x component of normalized direction vector")

    new_length = length * scale
    extend_length = (new_length - length) / 2

    # move each endpoint along the direction vector
    # (e.g. (dx_normalized * extend_length) is basically saying "how much of the extend_length is in the x direction")
    new_p1 = (x1 - dx_n * extend_length, y1 - dy_n * extend_length) # "-" is used to move point "backwards" along the direction vector
    new_p2 = (x2 + dx_n * extend_length, y2 + dy_n * extend_length) # "+" is used to move point "forwards" along the direction vector

    return LineString([new_p1, new_p2])


def extend_p2_in_linestring(linestring: LineString, extend_length: float) -> LineString:
    (x1, y1), (x2, y2), dx_n, dy_n, _ = _get_line_properties(linestring)

    # move endpoint p2 along the direction vector
    new_p2 = (x2 + dx_n * extend_length, y2 + dy_n * extend_length) # "+" is used to move point "forwards" along the direction vector

    return LineString([(x1, y1), new_p2])


# https://www.matematikbanken.dk/id/158/ 

def align_coords_with_centroid_angle(polygon: Polygon, home_gps, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y):
    
    prealigned_to_aligned_coords_dict = {} # (note, this holds entries for all points, not just the ones inside the polygon - but that's okay)

    # https://en.wikipedia.org/wiki/Centroid#Of_a_polygon 
    # https://stackoverflow.com/questions/75699024/finding-the-centroid-of-a-polygon-in-python 
    # https://shapely.readthedocs.io/en/stable/reference/shapely.Polygon.html#shapely.Polygon.centroid  
    centroid = polygon.centroid  # Point(lon, lat)
    centroid_line = LineString([Point(home_gps[1], home_gps[0]), centroid]) # Point(lon, lat)
    long_centroid_line = scale_linestring(centroid_line, 20) # make it longer in both directions to ensure it crosses the entire polygon. 20 is arbitrary, just needs to be large enough.

    centroid_line_perpen = # TODO  # this is a line that is perpendicular to the centroid line and crosses p1 on the centroid line


    for x_coord in x_axis_coords:
        for y_coord in y_axis_coords:

            coord = Point(x_coord, y_coord) # here, coord is a point! (x,y) aka (lon, lat) - NOT the usual (lat, lon)

            #### SNAPPING PERPENDICULAR TO CENTROID LINE ####

            closest_point_on_centroid_line = nearest_points(long_centroid_line, coord)[0] # https://shapely.readthedocs.io/en/2.0.4/manual.html#nearest-points 
            direction_line = LineString([closest_point_on_centroid_line, coord]) # line from coord to centroid line
            direction_line_length = direction_line.length

            # Figure out how much to extend point to snap to grid res
            grid_res = grid_res_y # TODO FOR DEBUGGING.
            extend_amount = round(direction_line_length / grid_res) * grid_res - direction_line_length # https://stackoverflow.com/questions/7859147/round-in-numpy-to-nearest-step 
            extended_direction_line = extend_p2_in_linestring(direction_line, extend_amount) # p2 is the waypoint

            coord_aligned = extended_direction_line.coords[1] # extract p2 from linestring (i.e. the extended coord) (.coords are x,y tuples, not Point objects, which is good!)

            #### SNAPPING PARALLEL TO CENTROID LINE ####

            # TODO 


            #### STORE ALLIGNED COORD IN LOOKUP DICT ####

            prealigned_to_aligned_coords_dict[(coord.y, coord.x)] = (coord_aligned[1], coord_aligned[0]) # (lon, lat) # TODO THIS IS THE CORRECT ONE
            #prealigned_to_aligned_coords_dict[(coord.y, coord.x)] = (coord.y, coord.x) # stored as (lat,long) - thats why its y,x # TODO DEBUGGING!! DEN OVENFOR SKAL BRUGE I STEDET

            # TODO FOR DEBUGGING:
            global debug_counter, debug_line, debug_point
            debug_counter += 1
            if debug_counter == 20:
                print(f"yupper: {direction_line_length+extend_amount} - {extended_direction_line.length}")
                debug_line = extended_direction_line # TODO remove
                debug_point = coord # TODO remove
            # FOR DEBUGGING END


            # TODO Problem. hvis vi snapper til grid res, hvis den den ene vej være for lidt punkter til at dække hele vejen over. og den anden vej være for mange..
            # MÅSKE: man skal snappe til den der hypotynuse længde i stedet? (kræver man tager centroid vinklen med i regnestykket)
            # Eller er det et problem vi stadig vil få? tænk over det
            
            # TODO Potentielt problem. efter allignement er de godt nok linet op.. men det tager bft og path planning ikke højde for.. så måske den springer frem og tilbage og derfor får endnu større hakker
            # Potentielt fix: i path planning processen kigger man ikke kun på bft, men OGSÅ de alligned gps coords. og så biaser man den til at vælge naboen, som resulterer i den mindte vinkelforskel i forhold til centroid linjen.
                    # JA, det tror jeg ville være en god ide! så når den path planner, tager den ikke bare den første nabo, men den med mindst vinkel forskel!
                    # men det bliver svært.. for den vælger jo ikke bare den første nabo nu.. den vælger den første entry i bft'en som er en nabo. tænk over det
                            # jeg definere "vinkelforskel" til: vinkelforskel realtiv til centroid linjen
                            # (måske man kan få den til at tage højrde for begge. måske den kan følge BFT'en, medmindre BFT'ens svar er over THRESHOLD dårligere vinkelforskel end en anden nabo --> så vælge den den anden nabo


            # TODO DEN STORE OPGAVE: Shaprely er x,y... altså linær koordinater
                    # men lat og long er ikke helt linære ... derfor den der direction_line f.eks. er skæv. fordi den er lavet ud fra at x og y er linære. måske centroid også er lidt off så. 
                    # måske kig på noget projection halløj https://shapely.readthedocs.io/en/stable/manual.html#other-transformations 
                    # ELLER OGSÅ, skal vi bare sige der kan opstå slight innacuaries pga det, men det accepteres pga simpliciteten.
                    # DET HER ER NOK GRUNDEN TIL DEN "wigler" lidt på kortet


            # TODO VIGTIGT! sikre der ikke ligger gps waypoints oveni hindanden!!! så skal den give en error.


            # aligned_coords.append({
            #     "aligned_waypoint": (waypoint_aligned.x, waypoint_aligned.y),
            #     "prealigned_waypoint": (waypoint.x, waypoint.y)
            # })
            # aligned_coords.append({ # TODO DEBUGGING. LINJEN OVERNFOR SKAL BRUGES
            #     "aligned_waypoint": (waypoint_aligned.x, waypoint_aligned.y),
            #     "prealigned_waypoint": (waypoint.x, waypoint.y)
            # })

    # # Convert aligned_coords from list to numpy array (split into x and y arrays)
    # aligned_coords_np = np.array(aligned_coords)

    # # TODO FOR DEBUGGING
    # print(f"x_coords: {x_coords}")
    # print(f"x_coords_aligned: {x_coords_aligned}")
    # return

    return prealigned_to_aligned_coords_dict, centroid, centroid_line, long_centroid_line # (centroid and long_centroid_line can be used for plotting)



def meters_to_lat_long_dif(current_approx_lat, meters: float):
    # https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters 
    # The conversion from meters is not the same for latitude and longitude - we account for this here.

    EARTH_RADIUS = 6371000.0  # mean Earth radius in meters

    lat_diff = (meters / EARTH_RADIUS) * (180 / math.pi)
    lon_diff = (meters / EARTH_RADIUS) * (180 / math.pi) / math.cos(math.radians(current_approx_lat))

    return lat_diff, lon_diff



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

    grid_res_y, grid_res_x = meters_to_lat_long_dif(DRONE_START[0], CAMERA_COVERAGE_LEN)
    #grid_res_y = 0.0001

    # Compute bounding box of polygon
    minx, miny, maxx, maxy = polygon.bounds

    # Generate pre-aligned x and y "grid" coords. (these coords are parallel with the lat/lon axes.)
    x_axis_coords = np.arange(minx, maxx, grid_res_x) 
    y_axis_coords = np.arange(miny, maxy, grid_res_y)

    #### Construct fly_nofly_grid #####
    # Create "grid" to keep track of which cells are inside the polygon and not in no-fly zone (0 = outside polygon or in no-fly zone, 1 = inside polygon and not in no-fly zone)

    # Create empty grid
    fly_nofly_grid = np.zeros((len(y_axis_coords), len(x_axis_coords)), dtype=int)
    
    # Fill grid
    for i, y in enumerate(y_axis_coords):
        for j, x in enumerate(x_axis_coords):
            point = Point(x, y)
            if polygon.contains(point) and not no_fly_zone_polygon.contains(point):
                fly_nofly_grid[i, j] = 1
                #print("YAP:", (i,j))


    #### Find grid cell corresponding to drone start position ####
    home_cell = None
    for i in range(len(y_axis_coords) - 1):
        for j in range(len(x_axis_coords) - 1):
            y_cell_min, y_cell_max = sorted([y_axis_coords[i], y_axis_coords[i + 1]]) # with sorted it doesn’t matter whether your coordinate list is increasing or decreasing in value (we force it to be increasing so our checking logic below works)
            x_cell_min, x_cell_max = sorted([x_axis_coords[j], x_axis_coords[j + 1]])

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
    if fly_nofly_grid[home_cell[0], home_cell[1]] == 0:
        raise ValueError("Drone start position is in a no-fly zone")
    print("DRONE START GRID COORDS:", home_cell)

    print(f"home_cell: {home_cell}, grid value at home_cell: {fly_nofly_grid[home_cell[0], home_cell[1]]}")

    # TODO DEBUGGING !! DET OVENFOR SKAL INDKOMMENRETES!!!!
    #home_cell = (12, 519)



    # #print(grid)
    # traversal_order_cells = traversal_order(grid, home_cell[0], home_cell[1], allow_diagonal=True) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
    # #print(traversal_order_cells)
    # traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_coords, y_coords)
    # print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)

    bf_traversal_cells = breadth_first_traversal(fly_nofly_grid, home_cell[0], home_cell[1], allow_diagonal=ALLOW_DIAGONAL_IN_BFT)
    bf_traversal_gps, _ = convert_cells_to_gps(bf_traversal_cells, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y, enable_centroid_alignment=ENABLE_CENTROID_ALIGNMENT, polygon=polygon)


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


    traversal_order_cells = single_drone_traversal_order(fly_nofly_grid, home_cell[0], home_cell[1], allow_diagonal_in_bft=ALLOW_DIAGONAL_IN_BFT, allow_diagonal_in_path=ALLOW_DIAGONAL_IN_PATH_OFFLINE_PLOTTING) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
    #print(traversal_order_cells)
    traversal_order_gps, info_for_plotting = convert_cells_to_gps(traversal_order_cells, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y, enable_centroid_alignment=ENABLE_CENTROID_ALIGNMENT, polygon=polygon)
    #print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)


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

    # Plot centroid and centroid line if enabled:
    if ENABLE_CENTROID_ALIGNMENT:
        # Plot centroid
        folium.CircleMarker(location=[info_for_plotting['centroid'].y, info_for_plotting['centroid'].x], radius=7, color="green", fill=True, fill_opacity=0.9).add_to(m)
        # Plot long centroid line
        folium.PolyLine(locations=[(info_for_plotting['long_centroid_line'].coords[0][1], info_for_plotting['long_centroid_line'].coords[0][0]), (info_for_plotting['long_centroid_line'].coords[1][1], info_for_plotting['long_centroid_line'].coords[1][0])], color="black", weight=3.0, opacity=1, dash_array='5, 10').add_to(m)

    # TODO for debugging
    if ENABLE_CENTROID_ALIGNMENT:
        folium.PolyLine(locations=[(debug_line.coords[0][1], debug_line.coords[0][0]), (debug_line.coords[1][1], debug_line.coords[1][0])], color="red", weight=2.0, opacity=1, ).add_to(m)
        folium.CircleMarker(location=[debug_point.y, debug_point.x], radius=6, color="purple", fill=True, fill_opacity=0.9).add_to(m)
    # Debugging end

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








# def main_backup(args=None) -> None:


#     # Note: polygons can for example for created in Mission Planner and exported as .poly files
#     polygon_coords = []
#     with open('baylands_polygon_v3.poly','r') as f: 
#         reader = csv.reader(f,delimiter=' ')
#         for row in reader:
#             if(row[0] == '#saved'): continue # skip header
#             polygon_coords.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

#     print("''''''''''''''''''''''''''")
#     print(polygon_coords)
#     print("''''''''''''''''''''''''''")
#     polygon = Polygon(polygon_coords)

#     no_fly_zone = []
#     with open('no_fly_zone.poly','r') as f:
#         reader = csv.reader(f,delimiter=' ')
#         for row in reader:
#             if(row[0] == '#saved'): continue # skip header
#             no_fly_zone.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

#     # Convert no-fly zone coordinates to a Shapely polygon
#     no_fly_zone_polygon = Polygon(no_fly_zone)

#     # Example polygon (GPS coordinates)
#     # polygon_coords = [
#     #     (12.490, 41.890),  # Example near Rome
#     #     (12.492, 41.890),
#     #     (12.492, 41.892),
#     #     (12.490, 41.892),
#     #     (12.490, 41.890)
#     # ]



#     # make sure polygon is convex
#     if(polygon.equals(polygon.convex_hull) == False):
#         raise ValueError("Polygon must be convex")
#     if(no_fly_zone_polygon.equals(no_fly_zone_polygon.convex_hull) == False): # TODO, NOT SURE IF THIS IS REQURED
#         raise ValueError("No-fly zone polygon must be convex")



#     # Define grid resolution (degrees)
#     #grid_res = 0.00005  # TODO NOT TRUE: I CHANGE IT - ~11 meters // https://lucidar.me/en/online-unit-converter-length-to-angle/convert-degrees-to-meters/ (earth radius = 6373000 m) 
#     grid_res = 0.0001

#     # Compute bounding box of polygon
#     minx, miny, maxx, maxy = polygon.bounds

#     # Generate grid points # TODO DE KAN OGSÅ BRUGES TIL AT CONVERT TILBAGE TIL LAT LONG
#     x_coords = np.arange(minx, maxx, grid_res) 
#     y_coords = np.arange(miny, maxy, grid_res)

#     # Align coords with centroid angle
#     if ENABLE_CENTROID_ALIGNMENT:
#         x_coords, y_coords, centroid, centroid_line, long_centroid_line = align_coords_with_centroid_angle(polygon, DRONE_START, x_coords, y_coords, grid_res) # TODO OVERVEJ OM DET DER "RYK HALVT IND I CELLEN HALLØJ JEG GØR ET ANDET STED GIVER MENINGE MED DET HER SLÅET TIL". måske skal det der "center ryk" rykkes ud a functionen og i sin egen. og så gøres som det første efter man laver x_coords og y_coords

#     # Create empty grid
#     grid = np.zeros((len(y_coords), len(x_coords)), dtype=int)

#     # Create "grid" to keep track of which cells are inside the polygon and not in no-fly zone (0 = outside polygon or in no-fly zone, 1 = inside polygon and not in no-fly zone)
#     for i, y in enumerate(y_coords):
#         for j, x in enumerate(x_coords):
#             point = Point(x, y)
#             if polygon.contains(point) and not no_fly_zone_polygon.contains(point):
#                 grid[i, j] = 1
#                 #print("YAP:", (i,j))



#     # Find grid cell corresponding to drone start position
#     home_cell = None
#     for i in range(len(y_coords) - 1):
#         for j in range(len(x_coords) - 1):
#             y_cell_min, y_cell_max = sorted([y_coords[i], y_coords[i + 1]]) # with sorted it doesn’t matter whether your coordinate list is increasing or decreasing in value (we force it to be increasing so our checking logic below works)
#             x_cell_min, x_cell_max = sorted([x_coords[j], x_coords[j + 1]])

#             # Check if drone start is inside this cell
#             if y_cell_min <= DRONE_START[0] < y_cell_max and x_cell_min <= DRONE_START[1] < x_cell_max:
#                 home_cell = (i, j)
#                 print("DRONE START GRID COORDS:", home_cell)
#                 break
#         if home_cell:
#             break
#     # Check if found
#     if home_cell is None:
#         raise ValueError("Drone start position is outside the grid")
#     if grid[home_cell[0], home_cell[1]] == 0:
#         raise ValueError("Drone start position is in a no-fly zone")
#     print("DRONE START GRID COORDS:", home_cell)

#     print(f"home_cell: {home_cell}, grid value at home_cell: {grid[home_cell[0], home_cell[1]]}")

#     # TODO DEBUGGING !! DET OVENFOR SKAL INDKOMMENRETES!!!!
#     #home_cell = (12, 519)



#     # #print(grid)
#     # traversal_order_cells = traversal_order(grid, home_cell[0], home_cell[1], allow_diagonal=True) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
#     # #print(traversal_order_cells)
#     # traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_coords, y_coords)
#     # print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)

#     bf_traversal_cells = breadth_first_traversal(grid, home_cell[0], home_cell[1], allow_diagonal=ALLOW_DIAGONAL_IN_BFT)
#     bf_traversal_gps = convert_cells_to_gps(bf_traversal_cells, x_coords, y_coords)


#     data_to_save = {
#         'home_cell': home_cell,
#         'home_gps': DRONE_START,
#         'bf_traversal_cells': bf_traversal_cells,
#         'bf_traversal_gps': bf_traversal_gps,
#     }
#     # Save traversal order to a file using pickle
#     with open('bf_traversal.pkl', 'wb') as fp:
#         pickle.dump(data_to_save, fp)

#     #print(grid)
#     #print("Grid shape:", grid.shape)

#     # # heatmap plot
#     # heatmap = np.zeros((len(y_coords), len(x_coords)), dtype=int)
#     # for idx, (i, j) in enumerate(traversal_order_cells):
#     #     heatmap[i, j] = idx + 1  # Start from 1 for better visibility
#     # plt.imshow(heatmap, origin="lower", cmap="hot", interpolation='nearest')
#     # plt.show()

#     # TODO, VI SKAL LIGE VÆLGE OM ORIGIN I VOReS GRID I OPPE I VENSTRE HJØRNE ELLER NEDRE VENSTRE HJØRNE
#     # NÅR VI PRINTER GRID, SER DEN UD PÅ EN MÅDE (forket i forhold til missionplanner)
#     # NÅR VI PRITNER GRID MED origin="lower" SER DEN UD PÅ EN ANDEN MÅDE (rigtig i forhold til missionplanner)
#     # DET SKAL VI LIGE FINDE UD AF HVAD DER SKER DER
#     # MEN DET GØR MÅSKE IKKE NOGET AT ARRAYED ER FLIPPED NÅR VI PLOTTER DET (det er bare sådan python plotter numpy array tænker jeg)









#     # for i, y in enumerate(y_coords):
#     #     if i == 0: x_prev = 0 # reset x_prev at start of new row
#     #     for j, x in enumerate(x_coords):
#     #         if j == 0: y_prev = 0 # reset y_prev at start of new column
#     #         if(DRONE_START[1] < x and DRONE_START[1] > x_prev and DRONE_START[0] < y and DRONE_START[0] > y_prev): # (lat, lon) aka (y,x)
#     #             print("DRONE START GRID COORDS: ", (i,j))
#     #             home_cell = (i,j)
#     #             break
#     #         x_prev = x
#     #     y_prev = y
#     # # Check if home_cell is valid
#     # if grid[home_cell[0], home_cell[1]] == 0:
#     #     raise ValueError("Drone start position is not within the polygon or is in a no-fly zone")
#     # print("DRONE START GRID COORDS: ", home_cell)








#     # PLOTTING SINGLE DRONE PATH ON A MAP (or just BFT points if you want)
#     PLOT_TYPE = 'bft_and_path' # "just_bft" or "bft_and_path"


#     traversal_order_cells = single_drone_traversal_order(grid, home_cell[0], home_cell[1], allow_diagonal_in_bft=ALLOW_DIAGONAL_IN_BFT, allow_diagonal_in_path=ALLOW_DIAGONAL_IN_PATH_OFFLINE_PLOTTING) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
#     #print(traversal_order_cells)
#     traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_coords, y_coords)
#     #print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)


#     import folium
#     import matplotlib.cm as cm
#     import matplotlib.colors as colors
#     from matplotlib import colormaps
#     # List of coordinates (using a shortened sample for demonstration; will replace with full list)
    

#     # Center the map around the average of coordinates
#     avg_lat = sum(lat for lat, lon in traversal_order_gps) / len(traversal_order_gps)
#     avg_lon = sum(lon for lat, lon in traversal_order_gps) / len(traversal_order_gps)

#     # Create the map
#     m = folium.Map(location=[avg_lat, avg_lon], zoom_start=18)

#     # Set up a colormap
#     amount_of_colored_points = len(bf_traversal_gps)  # You can change this to a lower number if you want to "zoom in" on the colormap
#     #amount_of_colored_points = 60
#     cmap = cm.get_cmap('inferno', amount_of_colored_points)  # You can change colormap to what you want

#     # Plot BF traversal points
#     for i, (lat, lon) in enumerate(bf_traversal_gps):
#         # (Add points to the map with colors based on their order)
#         color = colors.rgb2hex(cmap(i)[:3]) # Convert RGBA to hex
#         folium.CircleMarker(location=[lat, lon], radius=5, color=color, fill=True, fill_opacity=0.7).add_to(m)

#     if (not (PLOT_TYPE == 'just_bft')):
#         # Plot traversal order path
#         folium.PolyLine(locations=traversal_order_gps, color="blue", weight=2.0, opacity=1, ).add_to(m)
#         pass

#     # Plot centroid and centroid line if enabled:
#     if ENABLE_CENTROID_ALIGNMENT:
#         # Plot centroid
#         folium.CircleMarker(location=[centroid.y, centroid.x], radius=7, color="green", fill=True, fill_opacity=0.9).add_to(m)
#         # Plot long centroid line
#         folium.PolyLine(locations=[(centroid_line.coords[0][1], centroid_line.coords[0][0]), (centroid_line.coords[1][1], centroid_line.coords[1][0])], color="black", weight=3.0, opacity=1, dash_array='5, 10').add_to(m)


#     script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script directory
#     map_path = os.path.join(script_dir, "map.html")          # Set filename
#     m.save(map_path)














#     # plt.imshow(grid, origin="lower", cmap="Greens")
#     # plt.title("Polygon Rasterized to Grid")
#     # plt.show()


#     #Link til wavefront "breath first traversal (BFS)":
#     # https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/
#     # tænker bare vi bruger den order som BFS traversal returner.
#         # lige nu printer den grid elementerne. vi vil lave den så den returner en liste af tuples med koordinaterne i x,y i griddet.
#     # og så har vi samtidligt vores grid (hver element bool) som holder styr på om hver celle skal besøges eller ej (skal ikke være i no-fly-zone/obstacle eller allerede besøgt).
#     # dronen kigger på på traversal listen og tjekker om næste punkt er valid udfra vores grid


#     # Vi skal også have noget der converter fra grid koordinater til GPS koordinater, til når dronen rent faktisk skal flyve hen til et punkt.

if __name__ == '__main__':
    main()