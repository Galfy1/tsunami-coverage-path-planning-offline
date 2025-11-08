from shapely.geometry import Polygon, Point, LineString
import numpy as np


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
                intersection_points.append((y, x))
                val = cell_val

        #print(f"leng: {len(intersection_points)}")
        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            sweep_line = LineString([(p1_x, y), (p2_x, y)])
            gap_severity = 0
            for i in range(1, len(intersection_points) - 1, 2):
                start_pt = intersection_points[i]
                end_pt = intersection_points[i + 1]
                gap_severity += abs(end_pt[0] - start_pt[0])  # y coordinate difference
                pass

            # because  of the nature of the grid.. we cant split exactly on the line... so we have to split just above or below it..
            # (excessive splitting will be corrected later in the "culling merging" step)
            above_line = LineString([(p1_x, y + 1), (p2_x, y + 1)])
            below_line = LineString([(p1_x, y - 1), (p2_x, y - 1)])
            non_monotone_sweep_lines.append((above_line, intersection_points, gap_severity))
            non_monotone_sweep_lines.append((below_line, intersection_points, gap_severity))

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
                intersection_points.append((y, x))
                val = cell_val

        #print(f"leng: {len(intersection_points)}")
        if len(intersection_points) > 2:
            # Sweep line is does not pass criteria! (its intersecting non-monotone sections)
            sweep_line = LineString([(x, p1_y), (x, p2_y)])
            gap_severity = 0
            for i in range(1, len(intersection_points) - 1, 2):
                start_pt = intersection_points[i]
                end_pt = intersection_points[i + 1]
                gap_severity += abs(end_pt[1] - start_pt[1])  # x coordinate difference

            above_line = LineString([(p1_x, y + 1), (p2_x, y + 1)])
            below_line = LineString([(p1_x, y - 1), (p2_x, y - 1)])
            non_monotone_sweep_lines.append((above_line, intersection_points, gap_severity))
            non_monotone_sweep_lines.append((below_line, intersection_points, gap_severity))

            non_monotone_in_x = True

    is_irregular = non_monotone_in_x and non_monotone_in_y

    return non_monotone_sweep_lines, is_irregular, non_monotone_in_x, non_monotone_in_y



def scan_for_monotone_sections(grid: np.ndarray):
    monotone_sweep_lines = []
    monotone_in_x = True
    monotone_in_y = True

    p1_x = 0
    p2_x = grid.shape[1]
    for y in range(grid.shape[0]):
        intersection_points = []
        val = 0
        for x in range(grid.shape[1]):
            cell_val = grid[y][x]
            if cell_val != val:
                intersection_points.append((y, x))
                val = cell_val
        if len(intersection_points) <= 2:
            sweep_line = LineString([(p1_x, y), (p2_x, y)])
            monotone_sweep_lines.append((sweep_line, intersection_points, 0))
        else:
            monotone_in_y = False

    p1_y = 0
    p2_y = grid.shape[0]
    for x in range(grid.shape[1]):
        intersection_points = []
        val = 0
        for y in range(grid.shape[0]):
            cell_val = grid[y][x]
            if cell_val != val:
                intersection_points.append((y, x))
                val = cell_val
        if len(intersection_points) <= 2:
            sweep_line = LineString([(x, p1_y), (x, p2_y)])
            monotone_sweep_lines.append((sweep_line, intersection_points, 0))
        else:
            monotone_in_x = False

    is_regular = monotone_in_x and monotone_in_y
    return monotone_sweep_lines, is_regular, monotone_in_x, monotone_in_y
