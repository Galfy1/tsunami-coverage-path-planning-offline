
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
                                        
                                        # TODO i rapporten vis et plot med gap severity.. og hvis hvorfor det ikke er så godt for vores system
                                                # og så selvfølgelig nævn alt ovenfor. og også nævn det i en future work afsnit, hvad man kunne gøre.
ALLOW_VALID_MONOTONE_IN_SECTION_SCAN = False # also allow "valid" monotone sweep lines when scanning for non-monotone sections (valid: 2 regular partitions when using it to split)
                                            # why? setting this to True can potentially result in better splits. HOWEVER, processing time is longer.
                                            # (THIS OPTION IS NOT RELEVANT IF "gap_severity" METHOD IS USED FOR BEST SWEEP LINE SELECTION- since monotone sweep lines has no gaps)
                                            # (the https://arxiv.org/abs/2505.08060v1 paper done use this option (i.e. False), but they also only have a single UAV covering all partitions - so gap severity is used instead)
                                            # TODO i rapporten. vis et eksempel på hvor denne option gør tingene en del bedre (irregular_poly med UAV_COUNT=2, CAMERA_COVERAGE_LEN=1, area_balance og så True og False til den her option)

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

# TODO.. deres approach er at (forklar i rapporten. og hvordan vores er andereldes):
#                         computer vision terrain i forskellige polygons (ydre poly) arcoing to terrain type
#                         For hver polygon, brug alt det halløj her i filen til at partiion den ind (inder poly)
#                         find bedste path, der connected dem bedst, så hver polygon har 1 lang path
#                         Assign hver ydre poly til en UAV. 
#                               AKA: den deler ikke indre polygons mellem uavs...
# 
#                      Hmmm... hvad vi måske kan gøre! (igen.. vi har bare based den på deres.. vi laver ikke dikrete deres)
#                           Partion et polygon op (hvad jeg tidligere kaldre "indre poly")
#                           Brug antallen af droner til at del partitions op. 
#                               HVIS, der er flere partitions end droner, så del assign partitnes ud fra deres area, så det er ca lige fordel
#                               AKA en/flere droner får flere partions på samme tid! og så kan man bruge deres halløj i paperd til at finde den bedste path der connecter dem

#       HELE DET DER HALLØJ MED AT HVIS MAN HAR FLERE DRONER EN PARTITIONS:
#                   For dem er det ikke et problem... for igen... i uddeler ydre polygons til droner, ikke indre! og der er med stor sansynlighed nok ydre polygons (så alle droner kan aktiveres)
#                       MEN VORES APPROACH.. er det et reelt problem.. der skal løses TODO
                                # ALTSÅ: at vi højst sandsyngligt har flere droner en partiitons.. og derfor low UAV utilization.
            # POTENTIEL LØSNING: aner ikke om den er god... men stop culling merging når den har et subgrid antal der matcher drone antal... aka ikke vent den til er helt done
                                # og hvis den får under det, så tag den tidligere subgrid pack (og så får vi nok flere partinals end droner, men det kan vi fikse med deres path halløj)
                                # (hvis man skal konbinere partitions, så kombiner dem med mindste area)
                                # (MEN HUSK: ikke bare merge dem... hold dem adskilt, men brug deres metode til at cover begge.. så de får individuelle lawnmover path retninger der passer bedste (og med mindst overgange mellem dem))
                    # PROBLEMER MED DEN LØSNING: hvis man bare stopper culling merging tidligt, så kan partionals have meget stor forskel i area
            # EN ANDEN POTETNTEL LØSNIGN (måske bedre): lad culling merging køre færdig.. og så simpel bare split de største partionals op indtil vi har nok partionals til alle droner (split så ca samme area i hver)
                            # hver partition vil stadig være "regular" (monotone i mindst en retning)
            # måske nævn begge løsninger i rapporten. og hvorfor nummer 2 er bedre og valgt
            #   så efter culling merging har vi 3 scenarier:
                    # antal partian matcher uav count: alt godt.
                    # antal partian mindre end uav count: split de største partian op indtil match
                    # antal partian større end uav count: assign flere partians til nogle uavs (basret på area) og brug deres halløj i paper til at finde bedste patch når 1 drone skal cover over flere partions.
        
        
# TODO skriv at den ikke altid finder den bedste solution... f.eks. for irregular_poly med uav 2 --> her ville det være beder bare at cutte polyen i 2 på midten.
                                                                                                    # men vi cutter kun ved ikke-monotone sweel lines.
    





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