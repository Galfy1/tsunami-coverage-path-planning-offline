
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math
from collections import deque as queue
from custom_cell_tools import dx_4way, dy_4way, dx_8way, dy_8way
import matplotlib.pyplot as plt

# reuse code from offline_phase.py
from offline_phase import create_grid_from_polygon_and_noflyzones


#DRONE_START = (37.4135766590003, -121.997506320477) # (lat, lon) aka (y,x)
DRONE_START = (56.1672192716924, 10.152786411345) # for "paper_recreate.poly"
CAMERA_COVERAGE_LEN = 1 # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)


# NOTE: All indexing of grids is done as (y,x) - to match lat, lon convention
#       APART FROM: Shapely stuff (e.g. Polygon, Linestring), that is (x,y) to match shapely convention


# TODO er ikke sikker på "regular" og "irregular" er defineret korrekt .. men det er sådan de kalder det i artiklen?


n_neighbors = 8
dx_nway = dx_8way
dy_nway = dy_8way


def are_grids_adjacent(grid1: np.ndarray, grid2: np.ndarray) -> bool:
    # Both grids are same size. they either have 1 or 0. we want to check if they share a common boundary segment of 1s

    # error check
    if grid1.shape != grid2.shape:
        raise ValueError("Grids must be of the same shape to check adjacency in this function")

    # check all cells:
    for y in range(grid1.shape[0]):
        for x in range(grid1.shape[1]):

            if grid1[y][x] == 1:

                # check neighbors in grid2
                for direction in range(n_neighbors):

                    # adjacent cell coordinates
                    adjx = x + dx_nway[direction]
                    adjy = y + dy_nway[direction]

                    # bounds check
                    if (adjx < 0 or adjy < 0 or adjx >= grid2.shape[1] or adjy >= grid2.shape[0]):
                        continue # out of bounds

                    # check if adjacent cell in grid2 is 1
                    if grid2[adjy][adjx] == 1:
                        return True
        
    return False


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
                    _, grid_is_irregular = scan_for_non_monotone_sections(merged_grid)
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
                    # else:
                    #     # no merge happened, keep the sub-grid as is
                    #     new_sub_grids_pack.append(sub_grid)
                    #     ignore_grids.append(sub_grid)
                # else:
                #     # not adjacent, keep the sub-grid as is
                #     new_sub_grids_pack.append(sub_grid)
                #     ignore_grids.append(sub_grid)

            if local_merge_happened == False:
                # no merge happened for this sub-grid, keep it as is
                new_sub_grids_pack.append(sub_grid)
                ignore_grids.append(sub_grid)

        # Keep going through the new sub-grids pack until no merges happen
        if merge_happened:
            sub_grid_pack_queue.append(new_sub_grids_pack)
            print("777. subgrids len:", len(new_sub_grids_pack))
        else:
            # no merges happened - our work here is done
            all_sub_grids_after_culling = sub_grids_pack.copy()
            print("888. subgrids len:", len(sub_grids_pack))

    #print(f"asdasd {all_sub_grids_after_culling}")

    return all_sub_grids_after_culling






def split_grid_with_disconnected_sections(grid: np.ndarray):
    # check for disconnected sections in the grid and split into multiple sub-grids if found

    disconnected_sections = []
    
    #print("BURGER")

    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if grid[y][x] == 1:
                #print(f"SUSHI {x}, {y}")
                # found a flyable cell, start flood fill to find all connected cells
                visited = np.zeros_like(grid)
                to_visit = queue()
                to_visit.append( (y,x) )
                visited[y][x] = 1

                # flood fill flyable area (1s in grid)
                while(len(to_visit) > 0):
                    cy, cx = to_visit.popleft()

                    # check neighbors
                    for direction in range(n_neighbors):
                        adjx = cx + dx_nway[direction]
                        adjy = cy + dy_nway[direction]

                        # bounds check
                        if (adjx < 0 or adjy < 0 or adjx >= grid.shape[1] or adjy >= grid.shape[0]):
                            continue # out of bounds

                        if grid[adjy][adjx] == 1 and visited[adjy][adjx] == 0:
                            visited[adjy][adjx] = 1
                            to_visit.append( (adjy, adjx) )
                
                # Now, visited contains all connected cells from the starting point
                # Create a new sub-grid for this connected section
                sub_grid = np.zeros_like(grid)
                for yy in range(grid.shape[0]):
                    for xx in range(grid.shape[1]):
                        if visited[yy][xx] == 1:
                            sub_grid[yy][xx] = 1
                
                disconnected_sections.append(sub_grid)

                # Remove the visited cells from the original grid to avoid re-processing
                for yy in range(grid.shape[0]):
                    for xx in range(grid.shape[1]):
                        if visited[yy][xx] == 1:
                            grid[yy][xx] = 0


    return disconnected_sections


def split_grid_along_sweep_line(grid: np.ndarray, sweep_line: LineString):

    sub_grids = []

    # (remember, shapely LineString coords are in (x,y) format, while our grid is in (y,x) format)

    # Split the grid along the selected sweep line into two sub-grids
    if sweep_line.coords[0][1] == sweep_line.coords[1][1]:  # horizontal line (if p1 y matches p2 y)
        split_y = int(sweep_line.coords[0][1]) # "height"(y) if horizontal line
        # create sub-grids (same dimensions as original grid, but with 0s outside the sub-area
        sub_grid1 = np.copy(grid)
        sub_grid1[split_y:,:] = 0
        sub_grid2 = np.copy(grid)
        sub_grid2[:split_y,:] = 0
    else:  # vertical line
        split_x = int(sweep_line.coords[0][0]) # "width"(x) if vertical line
        sub_grid1 = np.copy(grid)
        sub_grid1[:, split_x:] = 0
        sub_grid2 = np.copy(grid)
        sub_grid2[:, :split_x] = 0


    # sub_grids.append(sub_grid1)
    # sub_grids.append(sub_grid2)
    # TODO indkommenter når det andet optil virker (og fjern det lige ovenfor)
    # If any of the sub-grids have disconnected sections, split them further
    sub_grids.extend(split_grid_with_disconnected_sections(sub_grid1))
    sub_grids.extend(split_grid_with_disconnected_sections(sub_grid2))

    # make sure we dont return the original grid as a sub-grid
    # TODO find ud af hvorfor det her er krævet.. er de tvirkelig the root cause vi løser her?
    # filtered_sub_grids = []
    # for sub_grid in sub_grids:
    #     if not np.array_equal(sub_grid, grid):
    #         filtered_sub_grids.append(sub_grid)
    return sub_grids

    return filtered_sub_grids



def scan_for_non_monotone_sections(grid: np.ndarray):

    non_monotone_sweep_lines = []
    non_monotone_in_x = False
    non_monotone_in_y = False

    # go through all horizontal sweep lines:
    p1_x = 0
    p2_x = grid.shape[1]
    for y in range(grid.shape[0]):
        intersection_points = []
        # check intersection with grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
        val = 0
        for x in range(grid.shape[1]):
            cell_val = grid[y][x]
            if cell_val != val:
                # transition detected
                intersection_points.append( (y,x) )
                val = cell_val
        
        #print(f"leng: {len(intersection_points)}")
        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            sweep_line = LineString([ (p1_x, y), (p2_x, y) ])
            gap_severity = 0
            for i in range(1, len(intersection_points)-1, 2):
                start_pt = intersection_points[i]
                end_pt = intersection_points[i+1]
                gap_severity += abs(end_pt[0] - start_pt[0]) # y coordinate difference
                pass

            non_monotone_sweep_lines.append( (sweep_line, intersection_points, gap_severity) )
            non_monotone_in_y = True


    # go through all vertical sweep lines:
    p1_y = 0
    p2_y = grid.shape[0]
    for x in range(grid.shape[1]):
        intersection_points = []
        # check intersection with grid by going through the sweep line and check for 1-->0 or 0-->1 transitions
        val = 0
        for y in range(grid.shape[0]):
            cell_val = grid[y][x]
            if cell_val != val:
                # transition detected
                intersection_points.append( (y,x) )
                val = cell_val
        
        #print(f"leng: {len(intersection_points)}")
        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            sweep_line = LineString([ (x, p1_y), (x, p2_y) ])
            gap_severity = 0
            for i in range(1, len(intersection_points)-1, 2):
                start_pt = intersection_points[i]
                end_pt = intersection_points[i+1]
                gap_severity += abs(end_pt[1] - start_pt[1]) # x coordinate difference

            non_monotone_sweep_lines.append( (sweep_line, intersection_points, gap_severity) )
            non_monotone_in_x = True

    is_irregular = non_monotone_in_x and non_monotone_in_y

    return non_monotone_sweep_lines, is_irregular


def main(args=None) -> None:

    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    with open('paper_recreate.poly','r') as f: 
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

        non_monotone_sweep_lines, grid_is_irregular = scan_for_non_monotone_sections(grid)

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

            # # Print grid to process visually for debugging
            # plt.imshow(grid, cmap='Greys', origin='lower', alpha=0.5)
            # plt.title("Original grid to split")
            # plt.show()

            # # Print sub-grids visually for debugging
            # for i, sub_grid in enumerate(sub_grids):
            #     plt.imshow(sub_grid, cmap='Greys', origin='lower', alpha=0.5)
            #     plt.title(f"Sub-grid {i+1}")
            #     plt.show()

            # TODO ISSUE. Når den (anden gang) skal process top griddet, finde den samme sweel line som tidligere.. fordi det er den samme pixels med stor gap (fordi den evaller i midten af pixelene - aka det noget med hvor man splitter? og det er over eller under.. men det skal kunne virke i begge retninger..)
            # Når det er løst, kan vi måske også fjerne filtered_sub_grids[] halløjet igen.
            # måske kan man løse det med: en ban liste af sweep lines der allerede er taget tidligere.

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

    # PLOTTING MADE USING AI:
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
            axs[i].imshow(fly_grid, cmap='Blues', origin='lower', alpha=0.25, vmin=0, vmax=1)
            # Overlay the sub-grid in orange/red
            axs[i].imshow(sub_grid, cmap='Oranges', origin='lower', alpha=0.9, vmin=0, vmax=1)
            axs[i].set_title(f"Regular sub-grid {i+1} (overlay on original)")

        # Hide any unused subplot axes
        for j in range(n, nrows * ncols):
            axs[j].axis('off')

        plt.tight_layout()
        plt.show()

    #print(f"Selected sweep line for splitting: {selected_sweep_line} with gap severity {max_gap_severity}")

    # # Plot grid and sweep lines for visualization:
    # plt.imshow(fly_grid, cmap='Greys', origin='lower')
    # # for sweep_line, points in non_monotone_sweep_lines:
    # #     x_coords = [p[1] for p in points]
    # #     plt.plot(x_coords, [sweep_line.y] * len(x_coords), color='red')
    # plt.show()

    # # Now process the non-monotone sweep lines to find the actual non-monotone sections
    # non_monotone_sections = []
    # for sweep_line, points in non_monotone_sweep_lines:
    #     points.sort(key=lambda p: p[0])  # Sort by x coordinate
    #     for i in range(len(points)-1):
    #         current_end = points[i+1]
    #         next_start = points[i+2] if i+2 < len(points) else None
    #         if next_start is not None and next_start < current_end:
    #             non_monotone_sections.append((sweep_line, current_end, next_start)) # (sweep line, current end, next start)

if __name__ == '__main__':
    main()