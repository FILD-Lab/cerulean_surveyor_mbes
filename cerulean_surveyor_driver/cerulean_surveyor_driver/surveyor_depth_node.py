import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from cerulean_surveyor_interfaces.msg import Float32Stamped


class Surveyor_Depth_Node(Node):
    def __init__(self):
        super().__init__('surveyor_depth_node')

        self.pub_depth = self.create_publisher(Float32Stamped, 'surveyor/depth_below_vehicle', 10)
        self.create_subscription(PointCloud2, 'surveyor/pointcloud', self.pointcloud_callback, 10)

    # Naive nadir depth: each point is x=0, y=-r*sin(angle), z=r*cos(angle) in the
    # sensor frame, so the beam closest to angle=0 is the one pointing straight
    # down the sensor's boresight. Does not correct for the sensor's offset from
    # base_link, nor for vehicle roll/pitch.
    def pointcloud_callback(self, msg):
        best_range = None
        best_abs_angle = None

        for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            angle = math.atan2(-y, z)
            abs_angle = abs(angle)

            if best_abs_angle is None or abs_angle < best_abs_angle:
                best_abs_angle = abs_angle
                best_range = math.sqrt(y * y + z * z)

        if best_range is None:
            return

        depth_msg = Float32Stamped()
        depth_msg.header = msg.header
        depth_msg.data = best_range
        self.pub_depth.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Surveyor_Depth_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
