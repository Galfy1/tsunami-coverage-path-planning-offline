
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math
from collections import deque as queue
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from typing import List
import pickle
import os
import sys

# Add the parent directory to the path (for our imports)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Our imports
from shared_tools.custom_cell_tools import dx_4way, dy_4way, dx_8way, dy_8way
from shared_tools.create_grid_from_poly import create_grid_from_polygon_and_noflyzones
from alternative_method_poly_decomp.lawnmower import lawnmower
from alternative_method_poly_decomp.scan_for_monotonicity import scan_for_non_monotone_sections, scan_for_monotone_sections
from alternative_method_poly_decomp.shared_grid_tools import are_grids_adjacent, split_grid_along_sweep_line
from alternative_method_poly_decomp.swarm_path_planning import path_plan_swarm
from matplotlib.collections import LineCollection


#DRONE_START = (37.4135766590003, -121.997506320477) # (lat, lon) aka (y,x)
DRONE_START = (56.1672192716924, 10.152786411345) # for "paper_recreate.poly"
CAMERA_COVERAGE_LEN = 1 # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)
UAV_COUNT = 1

ENABLE_PLOTTING = True


# NOTE: All indexing of grids is done as (y,x) - to match lat, lon convention
#       APART FROM: Shapely stuff (e.g. Polygon, Linestring), that is (x,y) to match shapely convention


# TODO er ikke sikker på "regular" og "irregular" er defineret korrekt .. men det er sådan de kalder det i artiklen?


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
        
        
#UAV_COUNT_HMMM = 10






def merge_grids(grid1: np.ndarray, grid2: np.ndarray) -> np.ndarray:
    # error check
    if grid1.shape != grid2.shape:
        raise ValueError("Grids must be of the same shape to merge")

    merged_grid = np.copy(grid1)

    for y in range(grid1.shape[0]):
        for x in range(grid1.shape[1]):
            
            if grid2[y][x] == 1:
                merged_grid[y][x] = 1

    return merged_grid



def culling_merging(all_sub_grids):
    # "To prevent excessive fragmentation, a culling step is performed in which adjacent sub-polygons
    # that share a common boundary segment are merged if their union satisfies the acceptability criterion"

    sub_grid_pack_queue = queue() # each element in the queue is a list of sub-grids to be processed
    sub_grid_pack_queue.append(all_sub_grids)

    #sub_grids_temp = []
    all_sub_grids_after_culling = []

    while(len(sub_grid_pack_queue) > 0):

        sub_grids_pack = sub_grid_pack_queue.popleft()

        new_sub_grids_pack = []
        ignore_grids = []
        merge_happened = False

        # Go through each pair of sub-grids and check if they are adjacent (i.e., share a common boundary segment)
        for sub_grid in sub_grids_pack:

            #print("PEPSI MAX")

            # SKIP anything already marked to be ignored/consumed in a merge
            if any(np.array_equal(sub_grid, ig) for ig in ignore_grids):
                continue

            local_merge_happened = False

            # check all other subgrids:
            for sub_grid_other in sub_grids_pack:
                if np.array_equal(sub_grid, sub_grid_other):
                    continue # skip self-comparison
                if any(np.array_equal(sub_grid_other, ig) for ig in ignore_grids):
                    continue # skip already merged grids

                if are_grids_adjacent(sub_grid, sub_grid_other):
                    # Check if their union satisfies the acceptability criterion
                    merged_grid = merge_grids(sub_grid, sub_grid_other)
                    _, grid_is_irregular, _, _ = scan_for_non_monotone_sections(merged_grid)
                    if grid_is_irregular == False:
                        print("WWWWWWWWWWW")
                        # Acceptability criterion satisfied!
                        # Merge the two sub-grids
                        #sub_grid = merged_grid # SAMME HER
                        new_sub_grids_pack.append(merged_grid) 

                        # make sure we dont check the two merged grids again:
                        ignore_grids.append(sub_grid)
                        ignore_grids.append(sub_grid_other)
                        local_merge_happened = True
                        merge_happened = True
                        break # exit inner loop to restart checking with new merged grid
                    else:
                        print("LLLLLLLLLLLL")
 

            if local_merge_happened == False:
                # no merge happened for this sub-grid, keep it as is
                new_sub_grids_pack.append(sub_grid)
                ignore_grids.append(sub_grid)


        if merge_happened:
            sub_grid_pack_queue.append(new_sub_grids_pack)
            print("777. subgrids len:", len(new_sub_grids_pack))
        else:
            # no merges happened - our work here is done
            all_sub_grids_after_culling = sub_grids_pack.copy()
            print("888. subgrids len:", len(sub_grids_pack))

    #print(f"asdasd {all_sub_grids_after_culling}")

    return all_sub_grids_after_culling





def main(args=None) -> None:

    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    # TODO horizontal_nonmono_only
    # TODO irregular_poly
    # TODO totally_mono.py
    # TODO paper_recreate.poly
    with open('alternative_method_poly_decomp/paper_recreate.poly','r') as f: 
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            if(row[0] == '#saved'): continue # skip header
            polygon_coords.append((float(row[1]), float(row[0]))) # (lat, lon)(aka y,x) to (lon, lat)(aka x,y)

    polygon = Polygon(polygon_coords)
    if not polygon.is_valid:
        raise ValueError("Polygon is not valid")

    # This decomp method does not allow for holes (i.e. no "no fly zones" inside the polygon)

    fly_grid, _ , _ , _ , _ , _  = create_grid_from_polygon_and_noflyzones(polygon, [], DRONE_START, CAMERA_COVERAGE_LEN)
    # (fly_grid is a 2D numpy array where 1 = flyable, 0 = no-fly zone - (y,x) indexing)

    regular_grids_result = []

    banned_sweep_lines = [] # to avoid selecting the same sweep line multiple times TODO

    # go though each "candidate sweep line" and check for non-monotone sections

    grid_queue = queue() # queue of grid/subgrids to be processed
    grid_queue.append(fly_grid)

    while(len(grid_queue) > 0):

        grid = grid_queue.popleft()

        non_monotone_sweep_lines, grid_is_irregular, _, _ = scan_for_non_monotone_sections(grid)


        # Check if "polygon" is irregular (i.e., non–monotone in both directions)
        if grid_is_irregular:
            print("Polygon is irregular (non-monotone in both directions)")
            # We need to split!

            # Find candidate sweep line with largest gap severity
            max_gap_severity = -1
            selected_sweep_line = None
            for sweep_line, _ , gap_severity in non_monotone_sweep_lines:
                if sweep_line in banned_sweep_lines:
                    continue # skip already used sweep lines
                if gap_severity > max_gap_severity:
                    max_gap_severity = gap_severity
                    selected_sweep_line = sweep_line

            if selected_sweep_line is None:
                print("No valid sweep line found for splitting (all previously used). Stopping further decomposition.")
                regular_grids_result.append(grid)
                continue

            banned_sweep_lines.append(selected_sweep_line)
            print(f"Selected sweep line for splitting: {selected_sweep_line}")

            # Split grid along selected sweep line
            sub_grids = split_grid_along_sweep_line(grid, selected_sweep_line)

            if len(regular_grids_result) == 44:
                _plot_grid(grid, "Original grid to split")

            print(f"Split grid into {len(sub_grids)} sub-grids along sweep line")

            # add sub-grids to queue for further processing
            for sub_grid in sub_grids:
                grid_queue.append(sub_grid)
        else:
            print("Polygon is regular (monotone in at least one direction)")
            # store the regular grid
            regular_grids_result.append(grid)


    # After processing all grids, perform culling/merging step 
    culling_merged_grids = culling_merging(regular_grids_result)

    # Plan path(s) over the resulting sub-grids
    path_per_uav = path_plan_swarm(culling_merged_grids, uav_count=UAV_COUNT)

    # Pickle path_per_uav:
    with open('poly_decomp_paths.pkl', 'wb') as f:
        pickle.dump(path_per_uav, f)

    # Plot if enabled
    if ENABLE_PLOTTING:
        plot_path_per_uav(fly_grid, culling_merged_grids, path_per_uav)

    # DEBUG
    #best_path_debug , start_cell, end_cell, _ = path_plan_swarm(culling_merged_grids, uav_count=3)
    # DEBUG END

    # Plot the resulting sub-grids
    # plot_subgrid(fly_grid, culling_merged_grids, plot_paths=True, best_path_debug=best_path_debug, start_cell=start_cell, end_cell=end_cell)



def plot_path_per_uav(fly_grid: np.ndarray, culling_merged_grids: list, path_per_uav: list):

    n = len(culling_merged_grids)
    print(f"Decomposition resulted in {n} regular sub-polygons")

    fig, ax = plt.subplots(figsize=(8, 8))

    # Build combined label grid: 0 = background, 1..n = sub-grids
    combined = np.zeros_like(fly_grid, dtype=int)
    for idx, sub_grid in enumerate(culling_merged_grids, start=1):
        combined = np.where(sub_grid == 1, idx, combined)

    # Show all sub-grids in the same fill color (single color mask)
    if np.any(combined > 0):
        mask = (combined > 0).astype(int)
        # Use a colormap where 0 is transparent and 1 is the chosen fill color
        # fill_color = (0.85, 0.9, 1.0, 0.8)  # subgrid color (rgba) - light bluish
        # cmap = ListedColormap([(1, 0, 0, 1), fill_color]) # background color
        fill_color = (0.85, 0.9, 1.0, 0.7)  # subgrid color (rgba) - light bluish
        cmap = ListedColormap([(0.941, 0.894, 0.569, 0.05), fill_color]) # background color
        ax.imshow(mask, cmap=cmap, origin='lower', vmin=0, vmax=1)

    # Draw divider lines between different sub-grids
    h, w = combined.shape
    vert_segments = []
    horz_segments = []
    for y in range(h):
        for x in range(w):
            lbl = combined[y, x]
            # vertical edge between pixels (boundary at x+0.5) for row y: span from y-0.5 to y+0.5
            if x + 1 < w:
                if combined[y, x + 1] != lbl:
                    # only draw if at least one side is part of a sub-grid (non-zero)
                    if lbl != 0 or combined[y, x + 1] != 0:
                        x_pos = x + 0.5
                        vert_segments.append([(x_pos, y - 0.5), (x_pos, y + 0.5)])
            # horizontal edge between pixels (boundary at y+0.5) for column x: span from x-0.5 to x+0.5
            if y + 1 < h:
                if combined[y + 1, x] != lbl:
                    if lbl != 0 or combined[y + 1, x] != 0:
                        y_pos = y + 0.5
                        horz_segments.append([(x - 0.5, y_pos), (x + 0.5, y_pos)])

    segments = vert_segments + horz_segments
    if segments:
        lc = LineCollection(segments, colors='black', linewidths=2)
        ax.add_collection(lc)

    ax.set_title("Regular sub-grids (single fill) with divider lines")

    # Plot paths for each UAV
    for uav_idx, path in enumerate(path_per_uav):
        if not path:
            continue
        try:
            xs = [p[1] for p in path]
            ys = [p[0] for p in path]
        except Exception:
            # if path is not a list of (y,x) coordinates, skip
            continue
        ax.plot(xs, ys, linewidth=1.5, marker='o', markersize=3, label=f'UAV {uav_idx+1} path')

    ax.legend(loc='upper right', fontsize='small')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()


def _debug_plot_subgrid(fly_grid: np.ndarray, culling_merged_grids: list, plot_paths: bool = True, best_path_debug=None, start_cell=None, end_cell=None):

    n = len(culling_merged_grids)
    print(f"Decomposition resulted in {n} regular sub-polygons")

    fig, ax = plt.subplots(figsize=(8, 8))
    #ax.imshow(fly_grid, cmap='Greys', origin='lower', alpha=0.3, vmin=0, vmax=1)

    if n > 0:
        combined = np.zeros_like(fly_grid, dtype=int)
        for idx, sub_grid in enumerate(culling_merged_grids, start=1):
            combined = np.where(sub_grid == 1, idx, combined)

        base_cmap = plt.cm.get_cmap('tab20', n)
        color_positions = np.linspace(0.2, 0.8, n)
        colors = [(0, 0, 0, 0)] + [base_cmap(pos) for pos in color_positions]
        cmap = ListedColormap(colors)

        ax.imshow(combined, cmap=cmap, origin='lower', alpha=0.9, vmin=0, vmax=n)
        ax.set_title("Regular sub-grids overlayed on original grid")

    if plot_paths and best_path_debug:
        xs = [p[1] for p in best_path_debug]
        ys = [p[0] for p in best_path_debug]
        ax.plot(xs, ys, color='black', linewidth=1.5, marker='o', markersize=3, label='path')

        if start_cell is not None:
            ax.scatter([start_cell[1]], [start_cell[0]], color='green', s=40, label='start')
        if end_cell is not None:
            ax.scatter([end_cell[1]], [end_cell[0]], color='red', s=40, label='end')

        ax.legend(loc='upper right', fontsize='small')

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()




def _debug_plot_subgrid_individually(fly_grid: np.ndarray, culling_merged_grids: list, plot_paths: bool = True):

    # if plot_paths is True, plot a fixed lawnmower path for each sub-grid for debugging (some sub-grids may not have valid paths with this fixed approach)

    # fixed lawnmower parameters
    start_corner = 'nw'
    direction = 'vertical'

    print (f"Decomposition resulted in {len(culling_merged_grids)} regular sub-polygons")
    # Plot all the regular sub-polygons with at most 5 subplots per row (multiple rows allowed)
    n = len(culling_merged_grids)
    if n > 0:
        max_cols = 5
        ncols = min(max_cols, n)
        nrows = math.ceil(n / ncols)
        fig, axs = plt.subplots(nrows, ncols, figsize=(5 * ncols, 5 * nrows))
        axs = np.array(axs).reshape(-1)  # flatten to 1D for easy indexing

        for i, sub_grid in enumerate(culling_merged_grids):
            # Plot original grid as light blue background
            axs[i].imshow(fly_grid, cmap='Blues', origin='lower', alpha=1, vmin=0, vmax=1)
            # Overlay the sub-grid in orange/red
            axs[i].imshow(sub_grid, cmap='Oranges', origin='lower', alpha=0.8, vmin=0, vmax=1)
            axs[i].set_title(f"Regular sub-grid {i+1} (overlay on original)")

            if plot_paths == True:
                # Attempt to compute and plot a lawnmower path for this sub-grid for debugging.
                try:
                    path, path_len, start_cell, end_cell, turn_count = lawnmower(sub_grid, start_corner=start_corner, direction=direction)
                    if path_len > 0:
                        xs = [p[1] for p in path]  # x coordinates for plotting
                        ys = [p[0] for p in path]  # y coordinates for plotting
                        axs[i].plot(xs, ys, color='cyan', linewidth=1.5, marker='o', markersize=3, label='lawnmower path')
                        # mark start and end
                        axs[i].scatter([start_cell[1]], [start_cell[0]], color='green', s=40, label='start')
                        axs[i].scatter([end_cell[1]], [end_cell[0]], color='red', s=40, label='end')
                        axs[i].legend(loc='upper right', fontsize='small')
                except Exception as e:
                    # If no valid path or other error, just skip plotting path for this sub-grid
                    print(f"Could not compute lawnmower path for sub-grid {i+1}: {e}")

        # Hide any unused subplot axes
        for j in range(n, nrows * ncols):
            axs[j].axis('off')

        plt.tight_layout()
        plt.show()




if __name__ == '__main__':
    main()