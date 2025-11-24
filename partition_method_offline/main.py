
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1
# NOTE: when "the paper" is mentioned in comments, it refers to the above linked paper

import csv
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import matplotlib.pyplot as plt
import pickle
import os
import sys

# Add the parent directory to the path (for our imports)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Our imports
from shared_tools.custom_cell_tools import dx_4way, dy_4way, dx_8way, dy_8way, convert_grid_to_gps
from shared_tools.create_grid_from_poly import create_grid_from_polygon_and_noflyzones
from partition_method_offline.swarm_path_planning import path_plan_swarm
from partition_method_offline.culling_merging import culling_merging
from partition_method_offline.plotting import plot_path_per_uav
from partition_method_offline.extract_regular_subgrids import extract_regular_subgrids

base_folder = "partition_method_offline"


#DRONE_START = (37.4135766590003, -121.997506320477) # (lat, lon) aka (y,x)
#DRONE_START = (56.1672192716924, 10.152786411345) # for "paper_recreate.poly"
DRONE_START = (37.4122067992952, -121.998909115791) # for "baylands_polygon_v3.poly"

#CAMERA_COVERAGE_LEN = 1 # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)
CAMERA_COVERAGE_LEN = 10 # for baylands_polygon_v3.poly
UAV_COUNT = 3

BEST_SWEEP_LINE_METHOD = 'area_balance' # 'gap_severity' or 'area_balance'.
                                        # Gap-severity is fine when a single drone covers all polygon partitions (as it does in the paper) (it yields good results).
                                        # For our system, however, work is split across multiple UAVs - so ensuring partition areas are balanced between the UAVs is 
                                        # more important than optimizing gap-severity. That is, we use resulting partition area instead of gap severity to pick the "best" sweep line.
                                        # HOWEVER: At this step, simply choosing the sweep line that yields the best area balance between resulting partitions is not ideal either. 
                                        #          This is because of subsequent steps:
                                        #               1: subsequent culling merging steps can merge partitions if the resulting partition is still a "regular" polygon
                                        #                   (i.e. the partition areas are changed)
                                        #               2: subsequent path planning steps can combine the coverage of a single UAV across multiple partitions. 
                                        #                   (i.e. the actual partion area does not change. But, the coverage area for each UAV does - which is what we actually care about)
                                        #          Also:
                                        #               * If a similar area balance exists between a horizontal and a vertical sweep line, this method will simply select the first (if their balances are equal) 
                                        #                 instead of choosing the one that would result in better future paths (e.g., lower cost).
                                        #          Solving these issues is non-trivial, as it would require information about the results of future processing steps.
                                        
ALLOW_VALID_MONOTONE_IN_SECTION_SCAN = False # also allow "valid" monotone sweep lines when scanning for non-monotone sections (valid: 2 regular partitions when using it to split)
                                            # why? setting this to True can potentially result in better splits. HOWEVER, processing time is longer.
                                            # (THIS OPTION IS NOT RELEVANT IF "gap_severity" METHOD IS USED FOR BEST SWEEP LINE SELECTION- since monotone sweep lines has no gaps)
                                            # (the https://arxiv.org/abs/2505.08060v1 paper done use this option (i.e. False), but they also only have a single UAV covering all partitions - so gap severity is used instead)

ENABLE_PLOTTING = True


# NOTE: All indexing of grids is done as (y,x) - to match lat, lon convention
#       APART FROM: Shapely stuff (e.g. Polygon, Linestring), that is (x,y) to match shapely convention


# NOTE: the definitions of "regular" and "irregular" polygons used here are based on the definitions in the paper:
#       a polygon is "regular" if it is monotone for all sweep lines in at least one direction (x or y)


n_neighbors = 8
dx_nway = dx_8way
dy_nway = dy_8way

# for debugging
def _plot_grid(grid: np.ndarray, title: str = "Grid"):
    plt.imshow(grid, cmap='Greys', origin='lower', alpha=0.5)
    plt.title(title)
    plt.show()





def main(args=None) -> None:

    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    # TODO horizontal_nonmono_only
    # TODO irregular_poly
    # TODO totally_mono.py
    # TODO paper_recreate.poly
    # TODO baylands_polygon_v3.poly
    with open(base_folder + '/baylands_polygon_v3.poly','r') as f: 
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            if(row[0] == '#saved'): continue # skip header
            polygon_coords.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    polygon = Polygon(polygon_coords)
    if not polygon.is_valid:
        raise ValueError("Polygon is not valid")
    
    no_fly_zones = []
    for filename in os.listdir(base_folder + "/no_fly_zones"):
        if filename.endswith('.poly'):
            with open(os.path.join(base_folder + "/no_fly_zones", filename), 'r') as f:
                reader = csv.reader(f,delimiter=' ')
                current_no_fly_zone = []
                for row in reader:
                    if(row[0] == '#saved'): continue # skip header
                    current_no_fly_zone.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)
                no_fly_zones.append(Polygon(current_no_fly_zone)) # convert no-fly zone coordinates to a Shapely polygon

    # This decomp method does not allow for holes (i.e. no "no fly zones" inside the polygon)

    fly_nofly_grid, home_cell , x_axis_coords , y_axis_coords , grid_res_x , grid_res_y  = create_grid_from_polygon_and_noflyzones(polygon, no_fly_zones, DRONE_START, CAMERA_COVERAGE_LEN)
    # (fly_grid is a 2D numpy array where 1 = flyable, 0 = no-fly zone - (y,x) indexing)

    regular_grids_result = extract_regular_subgrids(fly_nofly_grid, BEST_SWEEP_LINE_METHOD, ALLOW_VALID_MONOTONE_IN_SECTION_SCAN)

    # After processing all grids, perform culling/merging step 
    culling_merged_grids = culling_merging(regular_grids_result)
    
    for i, grid in enumerate(culling_merged_grids):
        print(f"Grid {i}: area={np.sum(grid)}, shape={grid.shape}")

    # Plan path(s) over the resulting sub-grids
    path_per_uav = path_plan_swarm(culling_merged_grids, uav_count=UAV_COUNT)

    # Compute fly_nofly_grid in gps for saving
    fly_nofly_grid_gps = convert_grid_to_gps(fly_nofly_grid, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y)

    # Pickle path_per_uav:
    paths_only = [x[0] for x in path_per_uav] # (we only need the paths for later use)
    data_to_save = {
        'home_cell': home_cell,
        'home_gps': DRONE_START,
        'fly_nofly_grid': fly_nofly_grid,
        'fly_nofly_grid_gps': fly_nofly_grid_gps,
        'uav_paths' : paths_only,
    }
    with open(base_folder + '/partition_offline_data.pkl', 'wb') as f:
        pickle.dump(data_to_save, f)

    #print(f"path2: {path_per_uav[1][0]}")

    # Plot if enabled
    if ENABLE_PLOTTING:
        plot_path_per_uav(fly_nofly_grid, culling_merged_grids, path_per_uav)

    # DEBUG
    #best_path_debug , start_cell, end_cell, _ = path_plan_swarm(culling_merged_grids, uav_count=3)
    # DEBUG END

    # Plot the resulting sub-grids
    # plot_subgrid(fly_grid, culling_merged_grids, plot_paths=True, best_path_debug=best_path_debug, start_cell=start_cell, end_cell=end_cell)








if __name__ == '__main__':
    main()