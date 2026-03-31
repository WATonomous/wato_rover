#!/usr/bin/env python3
"""Preprocess a .las elevation file into a CSV cost grid for the global costmap.

Reads a LAS point cloud, bins points into a 2D grid, computes per-cell elevation
range (slope/roughness proxy), and maps it to cost values 0-89 for use by the
ROS2 map_memory node. If more than 89 then its an obstacle

Usage:
    python3 preprocess_elevation.py \
        --input assets/fargate_utah_mdrs.las \
        --output /elevation_grid.csv \
        --width 60 --height 60 --resolution 0.5 \
        --origin_x -15.0 --origin_y -15.0 \
        --max_slope 2.0
"""

import argparse
import sys

import laspy
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Convert .las to elevation cost grid CSV")
    parser.add_argument("--input", required=True, help="Path to .las file")
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--width", type=int, default=60, help="Grid width in cells")
    parser.add_argument("--height", type=int, default=60, help="Grid height in cells")
    parser.add_argument("--resolution", type=float, default=0.5, help="Meters per cell")
    parser.add_argument("--origin_x", type=float, default=-15.0, help="Grid origin X in sim_world")
    parser.add_argument("--origin_y", type=float, default=-15.0, help="Grid origin Y in sim_world")
    
    # Coordinates on LAS are UTM so we subtract the center of the LAS bounds to get sim_world coordinates in meters
    parser.add_argument(
        "--las_offset_x", type=float, default=None,
        help="LAS X coordinate that maps to sim_world X=0. Default: center of LAS bounds.",
    )
    parser.add_argument(
        "--las_offset_y", type=float, default=None,
        help="LAS Y coordinate that maps to sim_world Y=0. Default: center of LAS bounds.",
    )
    parser.add_argument(
        "--max_slope", type=float, default=2.0,
        help="Elevation range (m) within a cell that saturates to max cost (89)",
    )
    args = parser.parse_args()

    # Load LAS file
    print(f"Loading {args.input}...")
    las = laspy.read(args.input)
    x = np.array(las.x)
    y = np.array(las.y)
    z = np.array(las.z)
    print(f"Loaded {len(x)} points")

    # Precision of 2 decimal places is sufficient for debugging and sanity checks
    print(f"  X: {x.min():.2f} to {x.max():.2f}")
    print(f"  Y: {y.min():.2f} to {y.max():.2f}")
    print(f"  Z: {z.min():.2f} to {z.max():.2f}")

    # Determine LAS-to-sim_world offset (default: center of LAS bounding box)
    las_offset_x = args.las_offset_x if args.las_offset_x is not None else (x.min() + x.max()) / 2.0
    las_offset_y = args.las_offset_y if args.las_offset_y is not None else (y.min() + y.max()) / 2.0
    print(f"LAS offset (sim_world origin in LAS coords): ({las_offset_x:.2f}, {las_offset_y:.2f})")

    # Transform LAS coordinates to sim_world frame
    sx = x - las_offset_x
    sy = y - las_offset_y

    # Grid parameters
    width = args.width
    height = args.height
    res = args.resolution
    ox = args.origin_x
    oy = args.origin_y

    # Compute grid cell indices for each point
    cell_x = ((sx - ox) / res).astype(int)
    cell_y = ((sy - oy) / res).astype(int)

    # Filter points within grid bounds
    mask = (cell_x >= 0) & (cell_x < width) & (cell_y >= 0) & (cell_y < height)
    cell_x = cell_x[mask]
    cell_y = cell_y[mask]
    z_filtered = z[mask]
    print(f"Points within grid bounds: {mask.sum()} / {len(x)}")

    if mask.sum() == 0:
        print("WARNING: No LAS points fall within the grid bounds!", file=sys.stderr)
        print("Check --las_offset_x/y, --origin_x/y, and grid dimensions.", file=sys.stderr)

    # Compute per-cell elevation range (max_z - min_z) as slope proxy
    flat_idx = cell_y * width + cell_x
    cost_grid = np.zeros(width * height, dtype=np.int8)

    # Use numpy bincount-based approach for min/max per cell
    total_cells = width * height
    cell_min = np.full(total_cells, np.inf)
    cell_max = np.full(total_cells, -np.inf)

    # Updates highest and lowest z for each cell index
    np.minimum.at(cell_min, flat_idx, z_filtered)
    np.maximum.at(cell_max, flat_idx, z_filtered)

    # Cells that had points
    has_points = cell_max > cell_min - 1e-9  # at least one point set both
    has_points = np.isfinite(cell_min) & np.isfinite(cell_max) & (cell_max > -np.inf)

    elev_range = np.zeros(total_cells)
    elev_range[has_points] = cell_max[has_points] - cell_min[has_points]

    # Min-max Normalize and map to cost 0-89
    normalized = np.clip(elev_range / args.max_slope, 0.0, 1.0)
    cost_grid = (normalized * 89).astype(np.int8)

    # Cells without points get cost 0
    no_points = ~has_points
    cost_grid[no_points] = 0

    # Stats
    cells_with_data = has_points.sum()
    print(f"Cells with elevation data: {cells_with_data} / {total_cells}")
    print(f"Cost distribution: min={cost_grid.min()}, max={cost_grid.max()}, "
          f"mean={cost_grid[has_points].mean():.1f}")

    # Write CSV output
    with open(args.output, "w") as f:
        # Header line for validation at load time
        f.write(f"{width},{height},{res},{ox},{oy}\n")
        # Flat cost values, row-major order (row 0 = lowest y)
        for i, val in enumerate(cost_grid):
            f.write(f"{val}")
            if i < len(cost_grid) - 1:
                f.write(",")
        f.write("\n")

    print(f"Wrote elevation grid to {args.output}")


if __name__ == "__main__":
    main()