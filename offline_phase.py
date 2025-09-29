


import numpy as np
from shapely.geometry import Point, Polygon


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
#print("Grid shape:", grid.shape)






import matplotlib.pyplot as plt

plt.imshow(grid, origin="lower", cmap="Greens")
plt.title("Polygon Rasterized to Grid")
plt.show()


#Link til wavefront "breath first traversal (BFS)":
# https://www.geeksforgeeks.org/dsa/breadth-first-traversal-bfs-on-a-2d-array/
# tænker bare vi bruger den order som BFS traversal returner.
    # lige nu printer den grid elementerne. vi vil lave den så den returner en liste af tuples med koordinaterne i x,y i griddet.
# og så har vi samtidligt vores grid (hver element bool) som holder styr på om hver celle skal besøges eller ej (skal ikke være i no-fly-zone/obstacle eller allerede besøgt).
# dronen kigger på på traversal listen og tjekker om næste punkt er valid udfra vores grid