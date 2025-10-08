


import os
import numpy as np
from shapely.geometry import Point, Polygon
from sympy import Add
from breadth_first_traversal import breadth_first_traversal
import matplotlib.pyplot as plt
import pickle

import csv

DRONE_START = (37.4122067992952, -121.998909115791) # (lat, lon) aka (y,x)


def convert_cells_to_gps(traversal_order_cells, x_coords, y_coords):
    traversal_order_gps = []
    for (i, j) in traversal_order_cells:
        lat = y_coords[i]
        lon = x_coords[j]
        traversal_order_gps.append((lat, lon))

    # make sure the traversal order gps is in the middle of the cells:
    grid_res = x_coords[1] - x_coords[0]  # assuming uniform spacing. will be positive if x_coords is increasing, negative if decreasing
    traversel_order_gps_centered = []
    for (lat, lon) in traversal_order_gps:
        traversel_order_gps_centered.append((lat + (grid_res / 2), lon + (grid_res / 2)))

    return traversel_order_gps_centered


def main(args=None) -> None:


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


    #print(grid)
    traversal_order_cells = breadth_first_traversal(grid, home_cell[0], home_cell[1]) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
    #print(traversal_order_cells)
    traversal_order_gps = convert_cells_to_gps(traversal_order_cells, x_coords, y_coords)
    print("TRAVERSAL ORDER GPS COORDS:", traversal_order_gps)

    #print(grid)
    #print("Grid shape:", grid.shape)

    # heatmap plot
    heatmap = np.zeros((len(y_coords), len(x_coords)), dtype=int)
    for idx, (i, j) in enumerate(traversal_order_cells):
        heatmap[i, j] = idx + 1  # Start from 1 for better visibility
    plt.imshow(heatmap, origin="lower", cmap="hot", interpolation='nearest')
    plt.show()

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



    data_to_save = {
        'home_pos_gps': DRONE_START,
        'traversal_order_gps': traversal_order_gps
    }
    # Save traversal order to a file using pickle
    with open('traversal_order_gps.pkl', 'wb') as fp:
        pickle.dump(data_to_save, fp)










    import folium
    import matplotlib.cm as cm
    import matplotlib.colors as colors
    from matplotlib import colormaps
    # List of coordinates (using a shortened sample for demonstration; will replace with full list)

    PLOT_TYPE = 'points' # 'points' or 'line'

    # Center the map around the average of coordinates
    avg_lat = sum(lat for lat, lon in traversal_order_gps) / len(traversal_order_gps)
    avg_lon = sum(lon for lat, lon in traversal_order_gps) / len(traversal_order_gps)

    # Create the map
    m = folium.Map(location=[avg_lat, avg_lon], zoom_start=18)

    # Set up a colormap
    amount_of_colored_points = len(traversal_order_gps)  # You can change this to a lower number if you want to "zoom in" on the colormap
    #amount_of_colored_points = 60
    cmap = cm.get_cmap('inferno', amount_of_colored_points)  # You can change colormap to what you want

    if(PLOT_TYPE == 'points'):
        #Add points to the map with colors based on their order
        for i, (lat, lon) in enumerate(traversal_order_gps):
            # Convert RGBA to hex
            color = colors.rgb2hex(cmap(i)[:3])
            folium.CircleMarker(location=[lat, lon], radius=5, color=color, fill=True, fill_opacity=0.7).add_to(m)
    if (PLOT_TYPE == 'line'):
        # Add a line connecting the points
        folium.PolyLine(locations=traversal_order_gps, color="blue", weight=2.5, opacity=1, ).add_to(m)

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