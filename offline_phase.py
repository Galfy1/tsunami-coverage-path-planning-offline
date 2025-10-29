import os
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import math
#from sympy import Add
from breadth_first_traversal import breadth_first_traversal
from single_drone_path_planning import single_drone_traversal_order_bft, single_drone_traversal_order_alt
import matplotlib.pyplot as plt
import pickle

import csv

DRONE_START = (37.4122067992952, -121.998909115791) # (lat, lon) aka (y,x)
CAMERA_COVERAGE_LEN = 10  # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)

# BFT Settings (only is of couse only relevant if BFT method is used on the drone):
ALLOW_DIAGONAL_IN_BFT = False 

# Offline Plotting Settings (this is just for single drone traversal plotting):
PLOTTING_METHOD_SELECTION = "centroid90_hybrid"  # Options: "BFT",
                                           #          "centroid", "centroid90","centroid180", (uses unidirectional by default)
                                           #          "centroid_hybrid", "centroid90_hybrid", (uses bidirectional by default + biases the drone towards its current moving direction)
                                           #                                                  (because bidirectional is used, centroid180_hybrid is not relevant here - as it would be the same as centroid_hybrid)
                                           # All data relevant for all modes will be saved in the pickle file anyway - so this setting is just for plotting
PLOTTING_ONLY_PLOT_POINTS = False # If true, only the waypoints are plotted. If false, the full path planning lines are also plotted
PLOTTING_ALLOW_DIAGONAL_IN_PATH_PLANNING = True # THIS IS JUST FOR PLOTTING IN THIS FILE ! For tsunami (for now) the setting is set in the online file.
PLOTTING_HYBRID_CENTROID_WEIGHT = 0.6 # (Only relevant if a hybrid method) how much weight to put on centroid direction vs current direction (0 = only current direction, 1 = only centroid direction)

# TODO skriv i rapporten and centroid metoden ikke accounter for nofly zones

# TODO følgene kan bruges til noter i rapporten. kan også bruges ti at sammenligne single og multi drone behavior.
# SINGLE DRONE OBSERVATIONS:
    # ALTERNTIVES THAT SEEM TO WORK BEST 
        # "centroid180" (with the default directional setting of unidirectional): Will always "push" the drone back towards home (along the centoid line) (i.e. coveraging everything outwards from home)
        # "centroid90" (with the default directional setting of unidirectional): Same as "centroid180", but the drone will coverage "sideways" from home (along a line perpendicular to the centroid line)
        # "hybrid" (with the default directional setting of bidirectional)
    # NOTES:
        # Its a bit unintuative, but becouse directional is set to unidirectional:
            # "centroid" and "centroid180" will make paths perpendicular to the centroid line 
            # "centroid90" which makes paths parallel to the centroid line
        # "centroid" does the same as "centroid180" (asuming unidirectional) - BUT, at the start, the drone is forced to fly directly to the other side of the polygon before starting the real "coverage pattern"

# TODO hybrider performer ikke så godt med single drone... men tror måske de performer bedre med multi droner. (men har ikke tested det endnu)


# TODO skriv i rapporten hvad de forskellig modes er.. og at centroud (pure centroid) er unidirection. og hvordan hybrid er andereldes (er bi directional også)

# NOTE: We have tried to keep all coords, cells, grids, etc. (y,x) aka (lat, lon) - besides shapely Point, those are (x,y)


# TODO "Firkant problemet".. (se paint)
    # potentiel løsning: lav en funktion til at detecte og centroid alignemnt skal slås til eller fra. 
        # Det gør den ved at beregne polygonens "summet kantvinkel - kalder jeg det". 
        # aka gå igennem alle edges i polygonen og kig på dens vinkel (relativt til x-aksen) og gang dem med længden (der er så den vægtet "kant vinklen"). Sum alle de vægtede kantvinkelr op
                # hvis den summede kantvinkel er 45grader +-22,5grader, så aktiver centroid alignemnt. ellers slå det fra.





def convert_cells_to_gps(cells, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y):

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
        
    return result_gps_centered



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

    print(f"home_cell: {home_cell}, grid value at home_cell: {fly_nofly_grid[home_cell[0], home_cell[1]]}")

    return fly_nofly_grid, home_cell, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y


def main(args=None) -> None:


    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    with open('baylands_polygon_v3.poly','r') as f: 
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            if(row[0] == '#saved'): continue # skip header
            polygon_coords.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    # print("''''''''''''''''''''''''''")
    # print(polygon_coords)
    # print("''''''''''''''''''''''''''")
    polygon = Polygon(polygon_coords)
    if not polygon.is_valid:
        raise ValueError("Polygon is not valid")

    # no_fly_zone = []
    # with open('no_fly_zone.poly','r') as f:
    #     reader = csv.reader(f,delimiter=' ')
    #     for row in reader:
    #         if(row[0] == '#saved'): continue # skip header
    #         no_fly_zone.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    no_fly_zones = []
    for filename in os.listdir("no_fly_zones"):
        if filename.endswith('.poly'):
            with open(os.path.join("no_fly_zones", filename), 'r') as f:
                reader = csv.reader(f,delimiter=' ')
                current_no_fly_zone = []
                for row in reader:
                    if(row[0] == '#saved'): continue # skip header
                    current_no_fly_zone.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)
                no_fly_zones.append(Polygon(current_no_fly_zone)) # convert no-fly zone coordinates to a Shapely polygon




    # # Convert no-fly zone coordinates to a Shapely polygon
    # no_fly_zone_polygon = Polygon(no_fly_zone)
    # no_fly_zones = [no_fly_zone_polygon] # we can extend this to multiple no-fly zones if needed

    # make sure polygon is convex
    if(polygon.equals(polygon.convex_hull) == False):
        raise ValueError("Polygon must be convex")
    for no_fly_zone_polygon in no_fly_zones:
        if(no_fly_zone_polygon.equals(no_fly_zone_polygon.convex_hull) == False): # TODO, NOT SURE IF THIS IS REQURED
            raise ValueError("No-fly zone polygon must be convex")

    fly_nofly_grid, home_cell, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y = create_grid_from_polygon_and_noflyzones(
                                                                                            polygon, no_fly_zones, DRONE_START, CAMERA_COVERAGE_LEN)

    # Compute breadth-first traversal from home cell
    bf_traversal_cells = breadth_first_traversal(fly_nofly_grid, home_cell, allow_diagonal=ALLOW_DIAGONAL_IN_BFT)
    bf_traversal_gps = convert_cells_to_gps(bf_traversal_cells, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y)

    # Prepare data to save
    centroid = polygon.centroid 
    centroid_coords = (centroid.y, centroid.x)  # (lat, lon) (we dont want drone to be dependent on shapely Point objects)
    data_to_save = {
        'home_cell': home_cell,
        'home_gps': DRONE_START,
        'bf_traversal_cells': bf_traversal_cells,
        'bf_traversal_gps': bf_traversal_gps,
        'fly_nofly_grid': fly_nofly_grid,
        'centroid': centroid_coords,
    }
    # Save traversal order to a file using pickle
    with open('offline_phase_data.pkl', 'wb') as fp:
        pickle.dump(data_to_save, fp)

    # # heatmap plot
    # heatmap = np.zeros((len(y_coords), len(x_coords)), dtype=int)
    # for idx, (i, j) in enumerate(traversal_order_cells):
    #     heatmap[i, j] = idx + 1  # Start from 1 for better visibility
    # plt.imshow(heatmap, origin="lower", cmap="hot", interpolation='nearest')
    # plt.show()


    ################################ PLOTTING ################################



    #traversal_order_cells = single_drone_traversal_order(fly_nofly_grid, home_cell[0], home_cell[1], allow_diagonal_in_bft=ALLOW_DIAGONAL_IN_BFT, allow_diagonal_in_path=ALLOW_DIAGONAL_IN_PATH_OFFLINE_PLOTTING) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
    if (PLOTTING_METHOD_SELECTION == "BFT"):
        traversal_order_cells = single_drone_traversal_order_bft(fly_nofly_grid, home_cell, allow_diagonal_in_bft=ALLOW_DIAGONAL_IN_BFT, allow_diagonal_in_path=PLOTTING_ALLOW_DIAGONAL_IN_PATH_PLANNING)
    else:
        traversal_order_cells = single_drone_traversal_order_alt(fly_nofly_grid, home_cell, DRONE_START, polygon, method=PLOTTING_METHOD_SELECTION,
                                                                 allow_diagonal_in_path=PLOTTING_ALLOW_DIAGONAL_IN_PATH_PLANNING, hybrid_centroid_weight=PLOTTING_HYBRID_CENTROID_WEIGHT)
    traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y)
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

    # Plot BF traversal points (NOTE: For convenience we plot the BFT points for plotting, even if PLOTTING_METHOD_SELECTION is not "BFT")
    for i, (lat, lon) in enumerate(bf_traversal_gps):
        # (Add points to the map with colors based on their order)
        if PLOTTING_METHOD_SELECTION == "BFT":
            color = colors.rgb2hex(cmap(i)[:3]) # Convert RGBA to hex
        else:
            color = 'black' # all black if not BFT plotting (the colors are a BFT specific thing) 
        folium.CircleMarker(location=[lat, lon], radius=5, color=color, fill=True, fill_opacity=0.7).add_to(m)

    if (not PLOTTING_ONLY_PLOT_POINTS):
        # Plot traversal order path
        folium.PolyLine(locations=traversal_order_gps, color="blue", weight=2.0, opacity=1, ).add_to(m)
        pass


    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script directory
    map_path = os.path.join(script_dir, "map.html")          # Set filename
    m.save(map_path)



    #Link til wavefront "breath first traversal (BFS)":
    # https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/
    # tænker bare vi bruger den order som BFS traversal returner.
        # lige nu printer den grid elementerne. vi vil lave den så den returner en liste af tuples med koordinaterne i x,y i griddet.
    # og så har vi samtidligt vores grid (hver element bool) som holder styr på om hver celle skal besøges eller ej (skal ikke være i no-fly-zone/obstacle eller allerede besøgt).
    # dronen kigger på på traversal listen og tjekker om næste punkt er valid udfra vores grid


    # Vi skal også have noget der converter fra grid koordinater til GPS koordinater, til når dronen rent faktisk skal flyve hen til et punkt.



if __name__ == '__main__':
    main()