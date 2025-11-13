import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from shapely.geometry import LineString
from alternative_method_poly_decomp.lawnmower import lawnmower





def plot_path_per_uav(fly_grid: np.ndarray, culling_merged_grids: list, path_per_uav: list):
    n = len(culling_merged_grids)
    print(f"Decomposition resulted in {n} regular sub-polygons")

    fig, ax = plt.subplots(figsize=(8, 8))

    # Build a combined label grid so each cell knows its sub-grid id
    combined = np.zeros_like(fly_grid, dtype=int)
    for idx, sub_grid in enumerate(culling_merged_grids, start=1):
        combined = np.where(sub_grid == 1, idx, combined)

    # Lightly fill areas covered by any sub-grid
    if np.any(combined > 0):
        mask = (combined > 0).astype(int)
        fill_color = (0.85, 0.9, 1.0, 0.7)
        cmap = ListedColormap([(0.941, 0.894, 0.569, 0.05), fill_color])
        ax.imshow(mask, cmap=cmap, origin='lower', vmin=0, vmax=1)

    # Collect grid divider segments between different sub-grids
    h, w = combined.shape
    vert_segments = []
    horz_segments = []
    for y in range(h):
        for x in range(w):
            lbl = combined[y, x]
            if x + 1 < w and combined[y, x + 1] != lbl:
                if lbl != 0 or combined[y, x + 1] != 0:
                    x_pos = x + 0.5
                    vert_segments.append([(x_pos, y - 0.5), (x_pos, y + 0.5)])
            if y + 1 < h and combined[y + 1, x] != lbl:
                if lbl != 0 or combined[y + 1, x] != 0:
                    y_pos = y + 0.5
                    horz_segments.append([(x - 0.5, y_pos), (x + 0.5, y_pos)])

    # Ensure outer boundaries of sub-grids are plotted
    boundary_segments = []
    for y in range(h):
        for x in range(w):
            if combined[y, x] == 0:
                continue
            if x == 0:
                boundary_segments.append([(x - 0.5, y - 0.5), (x - 0.5, y + 0.5)])
            if x == w - 1:
                boundary_segments.append([(x + 0.5, y - 0.5), (x + 0.5, y + 0.5)])
            if y == 0:
                boundary_segments.append([(x - 0.5, y - 0.5), (x + 0.5, y - 0.5)])
            if y == h - 1:
                boundary_segments.append([(x - 0.5, y + 0.5), (x + 0.5, y + 0.5)])

    segments = vert_segments + horz_segments + boundary_segments
    if segments:
        lc = LineCollection(segments, colors='black', linewidths=2.5)
        ax.add_collection(lc)

    # ax.set_title("Regular sub-grids (single fill) with divider lines")

    start_points = []
    end_points = []

    # # Plot each UAV path and record their start/end cells
    # for uav_idx, entry in enumerate(path_per_uav):
    #     if not entry:
    #         continue
    #     path, start_cell, end_cell = entry
    #     if path:
    #         xs = [p[1] for p in path]
    #         ys = [p[0] for p in path]
    #         ax.plot(xs, ys, linewidth=1.5, marker='o', markersize=3, label=f'UAV {uav_idx+1} path')
    #     if start_cell is not None:
    #         start_points.append(start_cell)
    #     if end_cell is not None:
    #         end_points.append(end_cell)

    # # Show start/end points on top of the paths
    # if start_points:
    #     sx = [p[1] for p in start_points]
    #     sy = [p[0] for p in start_points]
    #     ax.scatter(sx, sy, color='green', s=30, label='Start (green)', zorder=5)
    # if end_points:
    #     ex = [p[1] for p in end_points]
    #     ey = [p[0] for p in end_points]
    #     ax.scatter(ex, ey, color='red', s=30, label='End (red)', zorder=5)

    ax.legend(loc='upper right', fontsize='small')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()


# (used for the explanation in the report)
def plot_sweepline_example(fly_grid: np.ndarray, culling_merged_grids: list):
    n = len(culling_merged_grids)
    print(f"Decomposition resulted in {n} regular sub-polygons")

    fig, ax = plt.subplots(figsize=(8, 8))

    # Build a combined label grid so each cell knows its sub-grid id
    combined = np.zeros_like(fly_grid, dtype=int)
    for idx, sub_grid in enumerate(culling_merged_grids, start=1):
        combined = np.where(sub_grid == 1, idx, combined)

    # Lightly fill areas covered by any sub-grid
    if np.any(combined > 0):
        mask = (combined > 0).astype(int)
        fill_color = (0.376, 0.376, 0.376, 1.0)
        cmap = ListedColormap([(0.941, 0.894, 0.569, 0.05), fill_color])
        ax.imshow(mask, cmap=cmap, origin='lower', vmin=0, vmax=1)

    h, w = fly_grid.shape
    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(-0.5, h - 0.5)
    # plot some example sweep lines:
    sweepline = LineString([(-0.5, 3), (w - 0.5, 3)]) #horizontal
    x_sweepline, y_sweepline = sweepline.xy
    ax.plot(x_sweepline, y_sweepline, color='mediumblue', linewidth=2, label='example monotone sweep lines')
    sweepline = LineString([(5, -0.5), (5, h - 0.5)]) #vertical
    x_sweepline, y_sweepline = sweepline.xy
    ax.plot(x_sweepline, y_sweepline, color='mediumblue', linewidth=2)
    sweepline = LineString([(12, -0.5), (12, h - 0.5)]) #vertical
    x_sweepline, y_sweepline = sweepline.xy
    ax.plot(x_sweepline, y_sweepline, color='tomato', linewidth=2, label='example non-monotone sweep lines')
    sweepline = LineString([(-0.5, 25), (w - 0.5, 25)]) #horizontal
    x_sweepline, y_sweepline = sweepline.xy
    ax.plot(x_sweepline, y_sweepline, color='tomato', linewidth=2)

    ax.legend(loc='upper right')#, fontsize='small')
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