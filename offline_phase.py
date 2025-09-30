


import numpy as np
from shapely.geometry import Point, Polygon
from breadth_first_traversal import breadth_first_traversal
import matplotlib.pyplot as plt


import csv

polygon_coords = []
with open('testy_area.poly','r') as f:
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
grid_res = 0.00005  # TODO NOT TRUE: I CHANGE IT - ~11 meters // https://lucidar.me/en/online-unit-converter-length-to-angle/convert-degrees-to-meters/ (earth radius = 6373000 m) 

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

#print(grid)
traversal_order = breadth_first_traversal(grid, 12, 12) # start somewhere in the middle TODO: make sure start point is valid (inside polygon and not in no-fly zone)
print(traversal_order)

#print(grid)
#print("Grid shape:", grid.shape)

# heatmap plot
heatmap = np.zeros((len(y_coords), len(x_coords)), dtype=int)
for idx, (i, j) in enumerate(traversal_order):
    heatmap[i, j] = idx + 1  # Start from 1 for better visibility

plt.imshow(heatmap, origin="lower", cmap="hot", interpolation='nearest')
plt.show()

# TODO, VI SKAL LIGE VÆLGE OM ORIGIN I VOReS GRID I OPPE I VENSTRE HJØRNE ELLER NEDRE VENSTRE HJØRNE
# NÅR VI PRINTER GRID, SER DEN UD PÅ EN MÅDE (forket i forhold til missionplanner)
# NÅR VI PRITNER GRID MED origin="lower" SER DEN UD PÅ EN ANDEN MÅDE (rigtig i forhold til missionplanner)
# DET SKAL VI LIGE FINDE UD AF HVAD DER SKER DER
# MEN DET GØR MÅSKE IKKE NOGET AT ARRAYED ER FLIPPED NÅR VI PLOTTER DET (det er bare sådan python plotter numpy array tænker jeg)









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