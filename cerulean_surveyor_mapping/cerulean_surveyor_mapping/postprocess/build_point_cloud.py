import argparse

import laspy

from cerulean_surveyor_mapping.postprocess.bag_reader import read_pointcloud_map_points
from cerulean_surveyor_mapping.postprocess.projection import project_to_utm


def build_point_cloud(bag_path, topic, output_path):
    points = read_pointcloud_map_points(bag_path, topic)

    easting, northing, crs = project_to_utm(points['lat'], points['lon'])
    depth = points['z']  # map-frame Z (REP-105 up): negative underwater

    header = laspy.LasHeader(point_format=3, version="1.2")
    header.add_crs(crs)

    las = laspy.LasData(header)
    las.x = easting
    las.y = northing
    las.z = depth
    las.write(str(output_path))

    return len(easting)


def main():
    parser = argparse.ArgumentParser(
        description="Build a georeferenced LAS/LAZ point cloud from a surveyor/pointcloud_map bag")
    parser.add_argument('bag_path', help="Path to the rosbag2 bag directory")
    parser.add_argument('--topic', default='/fildlab_asv_1/surveyor/pointcloud_map')
    parser.add_argument('--output', default='pointcloud.laz',
                         help="Output path; extension (.las or .laz) selects compression")
    args = parser.parse_args()

    count = build_point_cloud(args.bag_path, args.topic, args.output)
    print(f"Wrote {count} points to {args.output}")


if __name__ == '__main__':
    main()
