
# THIS HAS NOTHING TO DO WITH TSUNAMI - ITS AN POLYGON DECOMPOSITION FOR COMPARISON PURPOSES
# Decomp method based on https://arxiv.org/abs/2505.08060v1

import csv
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math
from collections import deque as queue
from custom_cell_tools import dx_4way, dy_4way, dx_8way, dy_8way
import matplotlib.pyplot as plt
import itertools

# reuse code from offline_phase.py
from offline_phase import create_grid_from_polygon_and_noflyzones
from lawnmower import lawnmower
from scan_for_non_monotone_sections import scan_for_non_monotone_sections


DRONE_START = (37.4135766590003, -121.997506320477) # (lat, lon) aka (y,x)
#DRONE_START = (56.1672192716924, 10.152786411345) # for "paper_recreate.poly"
CAMERA_COVERAGE_LEN = 1 # meters. coverage of the drone camera in the narrowest dimension (i.e. the bottleneck dimension) (e.g. the width coverage if flying in landscape mode)



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



        #  Stop prematurely if we have equal or less sub-grids than UAVs (to avoid low UAV utilization)
        # if len(new_sub_grids_pack) <= UAV_COUNT_HMMM:
        #     # if we have less sub-grids than UAVs, keep the previous pack instead
        #     all_sub_grids_after_culling = sub_grids_pack.copy()
        #     print("999. subgrids len:", len(sub_grids_pack))
        #     break



        if merge_happened:
            sub_grid_pack_queue.append(new_sub_grids_pack)
            print("777. subgrids len:", len(new_sub_grids_pack))
        else:
            # no merges happened - our work here is done
            all_sub_grids_after_culling = sub_grids_pack.copy()
            print("888. subgrids len:", len(sub_grids_pack))

    #print(f"asdasd {all_sub_grids_after_culling}")

    return all_sub_grids_after_culling





# (see paper for loss function definition)
def compute_total_path_losses(candidate_list):

    first_term = sum(c['internal_loss'] for c in candidate_list)

    second_term = 0
    for i in range(0, len(candidate_list)-1):
        end_cell_current = candidate_list[i]['end_cell']
        start_cell_next = candidate_list[i+1]['start_cell']
        # euclidean distance between end cell of current and start cell of next
        dist = np.linalg.norm(np.array(end_cell_current) - np.array(start_cell_next))
        second_term += dist
    
    total_cost = first_term + second_term

    # TODO OPTIMIZATION HALLØJ

    return total_cost 


# def total_cost(candidate_list, order):
#     first_term = sum(c['internal_loss'] for c in candidate_list)
#     second_term = 0
#     for i in range(len(order)-1):
#         end_cell_current = np.array(candidate_list[order[i]]['end_cell'])
#         start_cell_next = np.array(candidate_list[order[i+1]]['start_cell'])
#         dist = np.linalg.norm(end_cell_current - start_cell_next)
#         second_term += dist
#     return first_term + second_term


# def find_best_order(candidate_list):
#     best_order = None
#     best_cost = float('inf')
#     for order in itertools.permutations(range(len(candidate_list))):
#         cost = total_cost(candidate_list, order)
#         if cost < best_cost:
#             best_cost = cost
#             best_order = order
#     return best_order, best_cost


# (see paper for loss function definition)
def compute_internal_path_losses(candidate_list, alpha = 1.0, len_tolerance = 10): # TODO find en okay default alpha og turn tol

    for candidate in candidate_list:
        turn_count = candidate['turn_count']

        # first term: path length
        first_term = candidate['path_len_euclidean']

        # second term: turns (if any other candidate has similar path length)
        valid_turn_counts = []
        for candidate_other in candidate_list:
            if abs(candidate_other['path_len_euclidean']-first_term) < len_tolerance:
                valid_turn_counts.append(candidate_other['turn_count'])
        second_term = turn_count - (min(valid_turn_counts) if valid_turn_counts else 0)
        second_term = alpha * max(0, second_term)

        candidate['internal_loss'] = first_term + second_term

    return candidate_list


def multi_partition_path_loss(candidate_list, alpha = 1.0, len_tolerance = 10): # TODO find en okay default alpha og turn tol
    
    # VORES FUNKTION INPUT ER (dem vi skal brute force alle kombinationer af):
        # en liste med candidate. hver candidate har input variabler vi skal tjekke alle kombinationer af.


    # hver canddiate har :
    #{
    #'path': path, # hmm den her skal ikke bruges som input... 
    #'path_len_euclidean': path_len_euclidean,
    #'start_cell': start_cell,
    #'end_cell': end_cell,
    #'turn_count': turn_count,
    #}


    ########## Compute internal path losses ##########
    internal_losses = []
    for candidate in candidate_list:
        turn_count = candidate['turn_count']

        # first term: path length
        first_term_internal = candidate['path_len_euclidean']

        # second term: turns (if any other candidate has similar path length)
        valid_turn_counts = []
        for candidate_other in candidate_list:
            if abs(candidate_other['path_len_euclidean']-first_term_internal) < len_tolerance:
                valid_turn_counts.append(candidate_other['turn_count'])
        second_term_internal = turn_count - (min(valid_turn_counts) if valid_turn_counts else 0)
        second_term_internal = alpha * max(0, second_term_internal)

        internal_losses.append(first_term_internal + second_term_internal)


    ########## Compute total path loss ##########
    first_term_total = sum(internal_losses)

    second_term_total = 0
    for i in range(0, len(candidate_list)-1):
        end_cell_current = candidate_list[i]['end_cell']
        start_cell_next = candidate_list[i+1]['start_cell']
        # euclidean distance between end cell of current and start cell of next
        dist = np.linalg.norm(np.array(end_cell_current) - np.array(start_cell_next))
        second_term_total += dist

    total_cost = first_term_total + second_term_total

    return total_cost

    # usefra skal vi kalde multi_partition_path_loss en masse gange med alle input variationer.
    #   og se hvem der giver mindste loss. hver gang der er en kombination der slår mindste loos rekorden, så gem den candidate_list vi bruge som input til den
    # Når vi til sidste har fundet den mindste loss, er det candidate_list der gav den loss som har svaret! med hivlket kombination der er bedst!




# def brute_force_evaluation(all_candidates_per_grid):



# def path_plan_swarm(all_sub_grids, uav_count):
#     start_corners = ['nw', 'ne', 'sw', 'se']
#     directions = ['horizontal', 'vertical']
#     final_paths = []

#     all_candidates_per_grid = []

#     # Step 1: Generate all candidates for each sub-grid
#     for sub_grid in all_sub_grids:
#         candidate_list = []
#         for corner in start_corners:
#             for direction in directions:
#                 path, path_len_euclidean, start_cell, end_cell, turn_count = lawnmower(
#                     sub_grid, start_corner=corner, direction=direction
#                 )
#                 if path is not None:
#                     candidate_list.append({
#                         'path': path,
#                         'path_len_euclidean': path_len_euclidean,
#                         'start_cell': start_cell,
#                         'end_cell': end_cell,
#                         'turn_count': turn_count,
#                         'internal_loss': None
#                     })
#         # Compute internal losses for this grid’s candidates
#         candidate_list = compute_internal_path_losses(candidate_list)
#         all_candidates_per_grid.append(candidate_list)

#     # Step 2: Generate all possible combinations across grids
#     # (Cartesian product of candidate options per grid)
#     all_combinations = itertools.product(*all_candidates_per_grid)

#     # Step 3: Brute-force evaluation
#     best_combination = None
#     best_cost = float('inf')

#     for combo in all_combinations:
#         combo_list = list(combo)
#         cost = compute_total_path_losses(combo_list)
#         if cost < best_cost:
#             best_cost = cost
#             best_combination = combo_list

#     # Step 4: Store best result
#     final_paths = [c['path'] for c in best_combination]

#     print("final path example:", final_paths[0] if final_paths else "No paths")
#     print(f"✅ Best total cost: {best_cost:.2f}")
#     return final_paths, best_cost



# def select_best_path_among_candidates(candidate_list: list):

#     # (use loss function from paper)

#     # calculate loss for each candidate:
#     for candidate in candidate_list:
#         candidate['internal_loss'] = path_loss(candidate['path'], candidate['start_cell'], candidate['end_cell'], candidate['turn_count'])

#     # # select the best candidate based on the lowest loss
#     # best_candidate = min(candidate_list, key=lambda x: x['loss']) # TODO check if this is correct

#     # return best_candidate



# def path_plan_swarm(all_sub_grids, uav_count):
#     # Match the number of partitions to the number of UAVs

#     final_paths = []
    
#     # if len(all_sub_grids) == uav_count:
#     #     # Perfect match. simple select the best lawnmower path for each partition

#     # Define all combinations of start corners and directions
#     start_corners = ['nw', 'ne', 'sw', 'se']
#     directions = ['horizontal', 'vertical']

#     # For each sub-grid, generate all candidate paths:
#     for sub_grid in all_sub_grids:
#         candidate_list = []
#         for corner in start_corners:
#             for direction in directions:
#                 path, path_len_euclidean, start_cell, end_cell, turn_count = lawnmower(
#                     sub_grid, start_corner=corner, direction=direction
#                 )
#                 if path is not None:
#                     candidate_list.append({
#                         'path': path,
#                         'path_len_euclidean': path_len_euclidean,
#                         'start_cell': start_cell,
#                         'end_cell': end_cell,
#                         'turn_count': turn_count,
#                         'internal_loss': None # to be computed later
#                     })

#     # Now, compute internal path losses for all candidates
#     candidate_list = compute_internal_path_losses(candidate_list)

#     best_order, best_cost = find_best_order(candidate_list)
#     print("###############################################")
#     print(best_order, best_cost)
#     print("###############################################")


#         # # Now select the best candidate for each sub-grid based on loss function
#         # best_candidate = select_best_path_among_candidates(candidate_list)
#         # final_paths.append(best_candidate)


    
#     # elif len(all_sub_grids) < uav_count:
#     #     # Not enough partitions, need to split some
#     #     # TODO implement splitting logic
#     #     pass
#     # elif len(all_sub_grids) > uav_count:
#     #     # Too many partitions, need to merge some
#     #     # TODO implement merging logic
#     #     pass
#     # return all_sub_grids 
    


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

    return sub_grids




def main(args=None) -> None:

    # Note: polygons can for example for created in Mission Planner and exported as .poly files
    polygon_coords = []
    # TODO horizontal_nonmono_only
    # TODO irregular_poly
    # TODO totally_mono.py
    with open('irregular_poly.poly','r') as f: 
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

    # DEBUG
    path_plan_swarm(culling_merged_grids, uav_count=3)
    # DEBUG END

    # Plot the resulting sub-grids
    plot_subgrid(fly_grid, culling_merged_grids, plot_paths=True)





def plot_subgrid(fly_grid: np.ndarray, culling_merged_grids: list, plot_paths: bool = True):


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
                    path, path_len, start_cell, end_cell, turn_count = lawnmower(sub_grid, start_corner='nw', direction='vertical')
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