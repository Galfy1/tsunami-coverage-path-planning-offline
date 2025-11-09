# Decomp method based on https://arxiv.org/abs/2505.08060v1

from typing import List
import numpy as np
import itertools
import math


from alternative_method_poly_decomp.lawnmower import lawnmower
from alternative_method_poly_decomp.shared_grid_tools import are_grids_adjacent, split_grid_along_sweep_line
from alternative_method_poly_decomp.scan_for_monotonicity import scan_for_monotone_sections

# (see paper for loss function definition)
def compute_total_path_loss(candidate_per_partition):

    first_term = sum(c['internal_loss'] for c in candidate_per_partition)

    second_term = 0
    for i in range(0, len(candidate_per_partition)-1):
        end_cell_current = candidate_per_partition[i]['end_cell']
        start_cell_next = candidate_per_partition[i+1]['start_cell']
        # euclidean distance between end cell of current and start cell of next
        dist = np.linalg.norm(np.array(end_cell_current) - np.array(start_cell_next))
        second_term += dist
    
    total_cost = first_term + second_term

    return total_cost 


#(see paper for loss function definition)
def compute_internal_path_losses(candidate_list, alpha = 0.5, len_tolerance = 10): # TODO find en okay default alpha og turn tol

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


def one_uav_multi_partitions_path_plan(sub_grids: List[np.ndarray]):
    # (note: "sub-grids" and "partitions" are used interchangeably here)

    ######### STEP 1: For each partition, generate all candidate lawnmower paths (and compute internal losses for each candidate) #########

    start_corners = ['nw', 'ne', 'sw', 'se']
    directions = ['horizontal', 'vertical']

    all_candidates_per_partition = [] # list of lists. each sub-list contains all valid candidates for that partition

    # Generate all candidates for each sub-grid
    for partition in sub_grids:
        candidate_list = []
        for corner in start_corners:
            for direction in directions:
                path, path_len_euclidean, start_cell, end_cell, turn_count = lawnmower(
                    partition, start_corner=corner, direction=direction
                )
                if path is not None:
                    candidate_list.append({
                        'path': path,
                        'path_len_euclidean': path_len_euclidean,
                        'start_cell': start_cell,
                        'end_cell': end_cell,
                        'turn_count': turn_count,
                        'internal_loss': None
                    })
        # Compute internal losses for this partition’s candidates
        candidate_list = compute_internal_path_losses(candidate_list)
        all_candidates_per_partition.append(candidate_list)


    ######### STEP 2: Brute-force evaluation to find best combination (and order) of candidates across all partitions #########

    best_loss = float('inf')
    best_combination = None


    # Try every permutation of partition order
    for order in itertools.permutations(range(len(all_candidates_per_partition))): # .permutations: generates all possible ordered arrangements of a given iterable (see https://www.geeksforgeeks.org/python/python-itertools-permutations/)
        ordered_partitions = [all_candidates_per_partition[i] for i in order]

        # Try every combination (one candidate per partition)
        for combo in itertools.product(*ordered_partitions): # itertools.product: produces all possible combinations of input iterables (see https://www.geeksforgeeks.org/python/python-itertools-product/)
            total_loss = compute_total_path_loss(combo)
            if total_loss < best_loss:
                best_loss = total_loss
                best_combination = combo

    ######### STEP 3: Output final paths from best combination (in the best order) #########

    # Build the final concatenated path
    best_path = []
    for cand in best_combination:
        best_path.extend(cand['path'])  # assume cand['path'] is a list of coordinates
    start_cell = best_combination[0]['start_cell']
    end_cell = best_combination[-1]['end_cell']

    print("final path example:", best_path[0] if best_path else "No paths")
    print(f"Best total cost: {best_loss:.2f}")
    return best_path, start_cell, end_cell, best_loss

def one_uav_single_partition_path_plan(grid: np.ndarray):
    # # Simple lawnmower path for single partition
    
    start_corners = ['nw', 'ne', 'sw', 'se']
    directions = ['horizontal', 'vertical']

    candidate_list = []
    for corner in start_corners:
        for direction in directions:
            path, path_len, start_cell, end_cell, turn_count = lawnmower(
                grid, start_corner=corner, direction=direction
            )
            if path is not None:
                candidate_list.append({
                    'path': path,
                    'path_len_euclidean': path_len,
                    'start_cell': start_cell,
                    'end_cell': end_cell,
                    'turn_count': turn_count,
                    'internal_loss': None
                })

    # Compute internal losses for this partition’s candidates
    candidate_list = compute_internal_path_losses(candidate_list)

    # find best candidate
    best_candidate = min(candidate_list, key=lambda c: c['internal_loss'])
    
    # return best path and associated info
    return best_candidate['path'], best_candidate['start_cell'], best_candidate['end_cell'], best_candidate['internal_loss']

def _grid_area(grid):
    # compute area of grid (number of flyable cells)
    return np.sum(grid == 1)

def _remove_grids_from_list(grid_list, grids_to_remove):
    filtered_grids = []
    for grid in grid_list:
        if not any(np.array_equal(grid, g) for g in grids_to_remove):
            filtered_grids.append(grid)
    return filtered_grids


# find all adjacent combinations of partitions (sub-grids) of size partition_count
def find_all_adjacent_partitions(grid, partition_count):
    combo_and_area = []
    for combination in itertools.combinations(range(len(grid)), partition_count): # (combination holds a list of indexes into all_sub_grids)
        # check if all partitions in combination are adjacent
        adjacentcies = [False] * len(combination) # each partition has an entry in this list. True = found atleast 1 adjacent partition, False = no adjacent partition found
        for i in range (len(combination)): # for each partition
            for j in range(len(combination)): # check all other partitions
                # skip self-comparison:
                if i == j: 
                    continue
                if are_grids_adjacent(grid[combination[i]], grid[combination[j]]):
                    adjacentcies[i] = True
                    break # no need to check other partitions for this one

        # if an adjacency chain exists (i.e., all partitions have at least one adjacent partition)
        if all(adjacentcies):

            # compute total area of this combination
            total_area = sum(_grid_area(grid[i]) for i in combination)
            combo_and_area.append((combination, total_area))
    
    return combo_and_area



def path_plan_swarm(all_sub_grids, uav_count):
    # (note: "sub-grids" and "partitions" are used interchangeably here)

    # TODO !!!!!! HUSK LIGE DEN DER 1px FEJL DER !!!!

    path_per_uav = []

    sub_grids_left = all_sub_grids.copy() 
    uavs_left = uav_count
    print("!!!!!!!!Initial subgrids len:", len(sub_grids_left))

    while len(sub_grids_left) > 0:

        if len(sub_grids_left) == uavs_left:
            print("XXXXXXXXXX")
            # simply assign one partition per UAV
            for grid in sub_grids_left:
                path, start_cell, end_cell, _ = one_uav_single_partition_path_plan(grid)
                path_per_uav.append((path, start_cell, end_cell))
            break # all UAVs assigned

        elif len(sub_grids_left) < uavs_left:
            print("YYYYYYYYYY")
            # Not enough partitions, need to split some
            # We split the largest partitions (by area) until we have enough (the resulting partitions will still be "regular")

            ########## STEP 1: Find largest partition ##########
            largest_partition = max(sub_grids_left, key=_grid_area)

            ########## STEP 2: Find sweep line that splits it best (in terms of area balance) ##########
            # we only consider monotone sweep lines (just to decrease search space) 
            # (many non-monotone sweep lines would produce more than 2 sub-partitions when splitting, which we dont want)
            monotone_sweep_lines, _, _, _ = scan_for_monotone_sections(largest_partition)
            # find best sweep line that splits the partition into two sub-partitions with most balanced area
            best_sweep_line = None
            best_area_balance = float('inf')
            for sweep_line, _ , _ in monotone_sweep_lines:
                # split partition along sweep line
                sub_partitions = split_grid_along_sweep_line(largest_partition, sweep_line)
                if len(sub_partitions) != 2:
                    continue # we only want to split into two partitions

                area1 = _grid_area(sub_partitions[0])
                area2 = _grid_area(sub_partitions[1])
                area_balance = abs(area1 - area2)
                if area_balance < best_area_balance:
                    best_area_balance = area_balance
                    best_sweep_line = sweep_line

            ########## STEP 3: Split the largest partition along the best sweep line ##########
            if best_sweep_line is not None:
                sub_partitions = split_grid_along_sweep_line(largest_partition, best_sweep_line)
                # Update sub_grids_left - replace the largest partition with the new partitions:
                sub_grids_left = _remove_grids_from_list(sub_grids_left, [largest_partition])
                sub_grids_left.extend(sub_partitions)
            else:
                raise ValueError("Could not find a valid sweep line to split the largest partition")

            # Continue loop to re-evaluate partition count

        elif len(sub_grids_left) > uavs_left:
            # Too many partitions, one/more UAVs need to cover multiple partitions
            print("ZZZZZZZZZZZ")

            partition_count_for_uav = math.ceil(len(sub_grids_left) / uavs_left)

            ########## STEP 1: Find best adjacent combinations of partitions for each UAV (best = smallest combined area) ##########
            # Note: the combination also has to be "valid". 
            # (here, "invalid" would meaning that if its removal from sub_grids_left would result in partitions that cannot form adjacent combinations for remaining UAVs)

            all_combos_and_areas = find_all_adjacent_partitions(sub_grids_left, partition_count_for_uav)

            subsequent_1uav_multi_partition_coming = (len(sub_grids_left)-partition_count_for_uav) > (uavs_left-1) # bool

            valid_combo_with_smallest_area = None
            while len(all_combos_and_areas) > 0:
                combo_with_smallest_area = min(all_combos_and_areas, key=lambda x: x[1], default=None)

                if subsequent_1uav_multi_partition_coming:
                    # check if its removal would still allow valid combinations for remaining UAVs:
                    
                    grids_to_remove = [sub_grids_left[i] for i in combo_with_smallest_area[0]] # (remember, combo_with_smallest_area[0] holds indexes into sub_grids_left)
                    modified_grid = _remove_grids_from_list(sub_grids_left, grids_to_remove)
                    # check if we can still form valid combinations for remaining UAVs
                    partition_count_for_remaining_uavs = math.ceil(len(modified_grid) / (uavs_left - 1)) # notice: we have one less UAV in this check
                    remaining_combos_and_areas = find_all_adjacent_partitions(modified_grid, partition_count_for_remaining_uavs)
                    if len(remaining_combos_and_areas) > 0:
                        valid_combo_with_smallest_area = combo_with_smallest_area
                        break # found valid combo
                    else: 
                        # if not valid, remove this combo from all_combos_and_areas and try again
                        all_combos_and_areas.remove(combo_with_smallest_area) # (.remove is allowed here, as all_combos_and_areas just holds indexes and areas)
                else: 
                    valid_combo_with_smallest_area = combo_with_smallest_area
                    break # no need to check validity, as remaining UAVs will only cover single partitions

            if valid_combo_with_smallest_area is None:
                raise ValueError("Could not find any valid partition combinations for UAV assignment")  # this should not happen

            ########## STEP 2: For 1 UAV, plan path over assigned partitions ##########

            best_combination = valid_combo_with_smallest_area[0]
            best_partitions = [sub_grids_left[i] for i in best_combination] # (remember, best_combination holds indexes into sub_grids_left)
            path, start_cell, end_cell, _ = one_uav_multi_partitions_path_plan(best_partitions)
            path_per_uav.append((path, start_cell, end_cell))

            
            ########## STEP 3: Remove assigned partitions from sub_grids_left ##########
            grids_to_remove = [sub_grids_left[i] for i in best_combination]
            print("????????grids to remove:", len(grids_to_remove))
            sub_grids_left = _remove_grids_from_list(sub_grids_left, grids_to_remove)

            ########## STEP 4: Update uavs_left ##########
            # (a single UAV has just been assigned)
            uavs_left -= 1

    return path_per_uav