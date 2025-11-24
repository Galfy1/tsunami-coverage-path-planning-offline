import math
from typing import List
import numpy as np
from shapely.geometry import LineString
from collections import deque as queue
from partition_method_offline.scan_for_monotonicity import scan_for_non_monotone_sections
from partition_method_offline.shared_grid_tools import split_grid_along_sweep_line



# find best sweel line (to split on) based on gap severity
def find_best_sweep_line_gap_severity(non_monotone_sweep_lines: List[LineString], banned_sweep_lines: List[LineString]) -> LineString:
    max_gap_severity = -1
    selected_sweep_line = None
    for sweep_line, _ , gap_severity in non_monotone_sweep_lines:
        if sweep_line in banned_sweep_lines:
            continue # skip already used sweep lines
        if gap_severity > max_gap_severity:
            max_gap_severity = gap_severity
            selected_sweep_line = sweep_line
    return selected_sweep_line


def find_best_sweep_line_area_balance(non_monotone_sweep_lines: List[LineString], grid: np.ndarray, banned_sweep_lines: List[LineString]) -> LineString:
    best_balance = math.inf
    selected_sweep_line = None
    for sweep_line, _ , _ in non_monotone_sweep_lines:
        if sweep_line in banned_sweep_lines:
            continue # skip already used sweep lines

        #print(f"Evaluating sweep line: {sweep_line}")

        # Split grid along this sweep line
        sub_grids = split_grid_along_sweep_line(grid, sweep_line)
        if len(sub_grids) < 2:  # TODO 
            continue 

        # find sweep line that results in best area balance between all sub-grids (remember, there might be more than 2 sub-grids)
        areas = [np.sum(sg) for sg in sub_grids]
        max_area = max(areas)
        min_area = min(areas)
        if max_area == 0:
            continue # avoid division by zero
        balance = (max_area - min_area) / max_area  # relative balance measure
        # print(f"Sweep line {sweep_line} results in area balance: {balance}. best so far: {best_balance}")
        if balance < best_balance:
            best_balance = balance
            selected_sweep_line = sweep_line


    return selected_sweep_line



def extract_regular_subgrids(fly_grid: np.ndarray, best_sweep_line_method: str = 'area_balance', allow_valid_monotone: bool = False) -> List[np.ndarray]:
    regular_grids_result = []

    banned_sweep_lines = [] # to avoid selecting the same sweep line multiple times

    # go though each "candidate sweep line" and check for non-monotone sections

    grid_queue = queue() # queue of grid/subgrids to be processed
    grid_queue.append(fly_grid)

    while(len(grid_queue) > 0):

        grid = grid_queue.popleft()

        non_monotone_sweep_lines, grid_is_irregular, _, _ = scan_for_non_monotone_sections(grid, allow_valid_monotone=allow_valid_monotone)


        # Check if "polygon" is irregular (i.e., non–monotone in both directions)
        if grid_is_irregular:
            print("Polygon is irregular (non-monotone in both directions)")
            # We need to split!

            # Find best candidate sweep line to split on (see BEST_SWEEP_LINE_METHOD definition in main.py for details):
            if best_sweep_line_method == 'gap_severity':
                selected_sweep_line = find_best_sweep_line_gap_severity(non_monotone_sweep_lines, banned_sweep_lines)
            elif best_sweep_line_method == 'area_balance':
                selected_sweep_line = find_best_sweep_line_area_balance(non_monotone_sweep_lines, grid, banned_sweep_lines)
            else:
                raise ValueError(f"Unknown BEST_SWEEP_LINE_METHOD: {best_sweep_line_method}")


            if selected_sweep_line is None:
                print("No valid sweep line found for splitting (all previously used). Stopping further decomposition.")
                regular_grids_result.append(grid)
                continue

            banned_sweep_lines.append(selected_sweep_line)
            print(f"Selected sweep line for splitting: {selected_sweep_line}")

            # Split grid along selected sweep line
            sub_grids = split_grid_along_sweep_line(grid, selected_sweep_line)

            #print(f"Split grid into {len(sub_grids)} sub-grids along sweep line")

            # add sub-grids to queue for further processing
            for sub_grid in sub_grids:
                grid_queue.append(sub_grid)
        else:
            print("Polygon is regular (monotone in at least one direction)")
            # store the regular grid
            regular_grids_result.append(grid)


    return regular_grids_result