import argparse

import numpy as np
import rasterio
from rasterio.transform import from_origin

from cerulean_surveyor_mapping.postprocess.bag_reader import read_pointcloud_map_points
from cerulean_surveyor_mapping.postprocess.projection import project_to_utm

NODATA = -9999.0


def build_bathymetry_map(bag_path, topic, output_path, cell_size=1.0):
    points = read_pointcloud_map_points(bag_path, topic)

    easting, northing, crs = project_to_utm(points['lat'], points['lon'])
    depth = points['z']

    min_e, max_e = easting.min(), easting.max()
    min_n, max_n = northing.min(), northing.max()

    n_cols = max(1, int(np.ceil((max_e - min_e) / cell_size)) + 1)
    n_rows = max(1, int(np.ceil((max_n - min_n) / cell_size)) + 1)

    # raster row 0 = top = max northing; row index increases downward (decreasing northing)
    col_idx = np.clip(((easting - min_e) / cell_size).astype(int), 0, n_cols - 1)
    row_idx = np.clip(((max_n - northing) / cell_size).astype(int), 0, n_rows - 1)

    sums = np.zeros((n_rows, n_cols), dtype=np.float64)
    counts = np.zeros((n_rows, n_cols), dtype=np.int64)

    np.add.at(sums, (row_idx, col_idx), depth)
    np.add.at(counts, (row_idx, col_idx), 1)

    grid = np.full((n_rows, n_cols), NODATA, dtype=np.float32)
    has_data = counts > 0
    grid[has_data] = (sums[has_data] / counts[has_data]).astype(np.float32)

    transform = from_origin(min_e, max_n, cell_size, cell_size)

    with rasterio.open(
        output_path, 'w',
        driver='GTiff',
        height=n_rows,
        width=n_cols,
        count=1,
        dtype=grid.dtype,
        crs=crs,
        transform=transform,
        nodata=NODATA,
    ) as dst:
        dst.write(grid, 1)

    return n_rows, n_cols, int(has_data.sum())


def main():
    parser = argparse.ArgumentParser(
        description="Build a gridded GeoTIFF bathymetry map from a surveyor/pointcloud_map bag")
    parser.add_argument('bag_path', help="Path to the rosbag2 bag directory")
    parser.add_argument('--topic', default='/fildlab_asv_1/surveyor/pointcloud_map')
    parser.add_argument('--output', default='bathymetry.tif')
    parser.add_argument('--cell-size', type=float, default=1.0, help="Grid cell size in meters")
    args = parser.parse_args()

    n_rows, n_cols, n_filled = build_bathymetry_map(args.bag_path, args.topic, args.output, args.cell_size)
    print(f"Wrote {n_rows}x{n_cols} grid ({n_filled} cells with data) to {args.output}")


if __name__ == '__main__':
    main()
