import numpy as np
from collections import deque as queue
from partition_method_offline.scan_for_monotonicity import scan_for_non_monotone_sections
from partition_method_offline.shared_grid_tools import are_grids_adjacent



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

        print("culling_merging() processing...")

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
                        #print("WWWWWWWWWWW")
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
 
            if local_merge_happened == False:
                # no merge happened for this sub-grid, keep it as is
                new_sub_grids_pack.append(sub_grid)
                ignore_grids.append(sub_grid)


        if merge_happened:
            sub_grid_pack_queue.append(new_sub_grids_pack)
            #print("777. subgrids len:", len(new_sub_grids_pack))
        else:
            # no merges happened - our work here is done
            all_sub_grids_after_culling = sub_grids_pack.copy()
            #print("888. subgrids len:", len(sub_grids_pack))

    #print(f"asdasd {all_sub_grids_after_culling}")

    return all_sub_grids_after_culling
