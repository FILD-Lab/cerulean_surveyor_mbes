import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from sensor_msgs.msg import PointCloud2, PointField, NavSatFix

from cerulean_surveyor_mapping.geodesy import enu_to_lat_lon
from cerulean_surveyor_mapping.postprocess.pointcloud_decode import decode_pointcloud2

# x,y,z (float32) + lat,lon (float64), tightly packed, no padding
OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='lat', offset=12, datatype=PointField.FLOAT64, count=1),
    PointField(name='lon', offset=20, datatype=PointField.FLOAT64, count=1),
]
OUT_POINT_STEP = 28
OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'lat', 'lon'],
    'formats': ['<f4', '<f4', '<f4', '<f8', '<f8'],
    'offsets': [0, 4, 8, 12, 20],
    'itemsize': OUT_POINT_STEP,
})


class PointcloudGeoreferencerNode(Node):
    def __init__(self):
        super().__init__('pointcloud_georeferencer_node')

        self.declare_parameter('map_frame', 'map')
        self.map_frame = self.get_parameter('map_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.datum = None  # (lat, lon, alt), from lla_datum

        self.pub_pointcloud_map = self.create_publisher(PointCloud2, 'surveyor/pointcloud_map', 10)
        self.create_subscription(NavSatFix, 'lla_datum', self.datum_callback, 1)
        self.create_subscription(PointCloud2, 'surveyor/pointcloud', self.pointcloud_callback, 10)

    def datum_callback(self, msg):
        self.datum = (msg.latitude, msg.longitude, msg.altitude)

    def pointcloud_callback(self, msg):
        if self.datum is None:
            self.get_logger().warn("No lla_datum received yet; dropping pointcloud", throttle_duration_sec=5.0)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException as exc1:
            # The sonar's own timestamp runs ~1-1.5s ahead of the Pi's clock
            # (a roughly-constant device-side quirk, not accumulating drift -
            # confirmed by restarting the driver without the offset resetting).
            # Blocking to wait for "future" TF data would just back up the
            # subscription queue and drop messages, so fall back to the
            # latest available transform instead. This trades a small,
            # bounded position error (~offset x vehicle speed) for never
            # blocking or silently dropping pings.
            try:
                latest_time = self.tf_buffer.get_latest_common_time(self.map_frame, msg.header.frame_id)
                transform = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, latest_time)
                self.get_logger().warn(
                    f"Exact-stamp TF lookup failed ({exc1}); used fallback at latest_time={latest_time}",
                    throttle_duration_sec=5.0)
            except tf2_ros.TransformException as exc2:
                self.get_logger().warn(
                    f"Both exact-stamp and fallback TF lookups failed. exact: {exc1} | fallback: {exc2}",
                    throttle_duration_sec=5.0)
                return

        # do_transform_cloud is fully vectorized (numpy einsum) as long as the
        # input cloud's fields are all one dtype, which x/y/z-only clouds are.
        cloud_map = do_transform_cloud(msg, transform)

        decoded = decode_pointcloud2(cloud_map, ('x', 'y', 'z'))
        xs, ys, zs = decoded['x'], decoded['y'], decoded['z']
        if xs.size == 0:
            return

        datum_lat, datum_lon, datum_alt = self.datum
        lats, lons = enu_to_lat_lon(xs, ys, zs, datum_lat, datum_lon, datum_alt)

        out_array = np.empty(xs.shape[0], dtype=OUT_DTYPE)
        out_array['x'] = xs
        out_array['y'] = ys
        out_array['z'] = zs
        out_array['lat'] = lats
        out_array['lon'] = lons

        out_cloud = PointCloud2()
        out_cloud.header = cloud_map.header
        out_cloud.height = 1
        out_cloud.width = xs.shape[0]
        out_cloud.fields = OUT_FIELDS
        out_cloud.is_bigendian = False
        out_cloud.point_step = OUT_POINT_STEP
        out_cloud.row_step = OUT_POINT_STEP * xs.shape[0]
        out_cloud.is_dense = True
        out_cloud.data = out_array.tobytes()

        self.pub_pointcloud_map.publish(out_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudGeoreferencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
