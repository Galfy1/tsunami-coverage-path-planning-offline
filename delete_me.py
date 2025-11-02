# TODO analyser hvorfor nederstående virker. og haps de deler der giver mening.
def culling_merging(all_sub_grids):
    # "To prevent excessive fragmentation, a culling step is performed in which adjacent sub-polygons
    # that share a common boundary segment are merged if their union satisfies the acceptability criterion"

    sub_grid_pack_queue = queue() # each element in the queue is a list of sub-grids to be processed
    sub_grid_pack_queue.append(all_sub_grids)

    all_sub_grids_after_culling = []

    # helper to compare packs by content to detect stalls
    def pack_signature(grids):
        try:
            # fast signature per grid; good enough within a single run
            return tuple(sorted((g.shape, g.dtype.str, hash(g.tobytes())) for g in grids))
        except Exception:
            # fallback: lengths only
            return (len(grids),)

    while(len(sub_grid_pack_queue) > 0):

        sub_grids_pack = sub_grid_pack_queue.popleft()

        old_sig = pack_signature(sub_grids_pack)

        new_sub_grids_pack = []
        ignore_grids = []
        merge_happened = False

        # Go through each pair of sub-grids and check if they are adjacent (i.e., share a common boundary segment)
        for sub_grid in sub_grids_pack:

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
                        # Acceptability criterion satisfied: merge the two sub-grids
                        new_sub_grids_pack.append(merged_grid)

                        # mark both originals as consumed so they won't be added later
                        ignore_grids.append(sub_grid)
                        ignore_grids.append(sub_grid_other)

                        local_merge_happened = True
                        merge_happened = True
                        break # restart with next sub_grid
                    else:
                        # merge not acceptable; try next candidate
                        pass

            if local_merge_happened == False:
                # no merge happened for this sub-grid, keep it as is
                new_sub_grids_pack.append(sub_grid)
                # don't add to ignore here; we want it to remain in the pack

        # Deduplicate identical grids (by content) to avoid ping-ponging
        deduped_pack = []
        for g in new_sub_grids_pack:
            if any(np.array_equal(g, h) for h in deduped_pack):
                continue
            deduped_pack.append(g)

        # Detect no-progress to avoid infinite loops (same content before/after pass)
        new_sig = pack_signature(deduped_pack)

        if merge_happened and new_sig != old_sig:
            sub_grid_pack_queue.append(deduped_pack)
            print("777. subgrids len:", len(deduped_pack))
        else:
            # no merges happened or no effective change - done
            all_sub_grids_after_culling = deduped_pack.copy()
            print("888. subgrids len:", len(deduped_pack))
            break

    return all_sub_grids_after_culling
# ...existing code...