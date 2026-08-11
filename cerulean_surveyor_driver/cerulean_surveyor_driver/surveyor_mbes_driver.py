from brping import definitions
from brping import Surveyor240
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import PointCloud2, PointField, Temperature, FluidPressure, Imu
from std_msgs.msg import Header
from cerulean_surveyor_interfaces.msg import Float32MultiArrayStamped, Float32Stamped


class Surveyor_MBES_Driver(Node):
    def __init__(self):
        super().__init__('surveyor_mbes_driver')

        # Interface parameters
        self.declare_parameter('interface.device',           Parameter.Type.STRING)
        self.declare_parameter('interface.baudrate',         Parameter.Type.INTEGER)
        self.declare_parameter('interface.connection_type',  Parameter.Type.STRING)
        self.declare_parameter('interface.udp_address',      Parameter.Type.STRING)
        self.declare_parameter('interface.udp_port',         Parameter.Type.INTEGER)

        # Configuration parameters
        self.declare_parameter('configuration.frame_id',     Parameter.Type.STRING)
        self.declare_parameter('configuration.auto_range',        Parameter.Type.BOOL)
        self.declare_parameter('configuration.range',        Parameter.Type.DOUBLE)
        self.declare_parameter('configuration.sound_speed',        Parameter.Type.DOUBLE)

        # Get interface parameters
        device             = self.get_parameter('interface.device').value
        baudrate           = self.get_parameter('interface.baudrate').value
        connection_type    = self.get_parameter('interface.connection_type').value
        udp_address        = self.get_parameter('interface.udp_address').value
        udp_port           = self.get_parameter('interface.udp_port').value

        # Get configuration parameters
        frame_id           = self.get_parameter('configuration.frame_id').value
        sound_speed        = self.get_parameter('configuration.sound_speed').value
        auto_range         = self.get_parameter('configuration.auto_range').value
        
        if auto_range:
            range_mm = -1
        else:
            range_m            = self.get_parameter('configuration.range').value
            range_mm           = int(range_m * 1000)
        
        # ROS Stuff
        self.header = Header()
        self.header.frame_id = frame_id

        self.pub_imu   = self.create_publisher(Imu,          'surveyor/imu',          10)
        self.pub_pitch = self.create_publisher(Float32Stamped, 'surveyor/euler/pitch', 10)
        self.pub_roll  = self.create_publisher(Float32Stamped, 'surveyor/euler/roll',  10)
        self.pub_temperature = self.create_publisher(Temperature, 'surveyor/temperature', 10)
        self.pub_pressure = self.create_publisher(FluidPressure, 'surveyor/pressure', 10)
        self.pub_atof = self.create_publisher(Float32MultiArrayStamped, 'surveyor/atof', 10)
        self.pub_pointcloud = self.create_publisher(PointCloud2, 'surveyor/pointcloud', 10)

        self.pc_msg = PointCloud2()
        self.pc_msg.height      = 1
        self.pc_msg.fields      = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        self.pc_msg.is_bigendian = False
        self.pc_msg.point_step   = 12  # 3 x float32
        self.pc_msg.is_dense     = True


        # Connect based on connection_type
        self.surveyor = Surveyor240()
        if connection_type == 'serial':
            self.surveyor.connect_serial(device, baudrate)
        elif connection_type == 'udp':
            self.surveyor.connect_udp(udp_address, udp_port)
        else:
            self.get_logger().error(f"Unknown connection_type: '{connection_type}'. Use 'serial' or 'udp'.")

        if not self.surveyor.initialize():
            self.get_logger().error("Failed to initialize Surveyor240")
            exit(1)

        else:
            self.surveyor.control_set_ping_parameters(end_mm = int(range_mm),
                                                    ping_enable=True,
                                                    enable_yz_point_data=False,
                                                    enable_atof_data=True,
                                                    sos_mps=sound_speed)
            
            self.get_logger().info(f"Initialized Surveyor240 with {'auto_range' if range_mm == -1 else f'range={range_mm/1000} mm'}")
            
            self.create_timer(0.0, self.run)
        
    # The Surveyor240 reports utc_msec as 0 (or, if its UTC time request to the
    # host was never answered, an underflowed huge value) when it has no valid
    # UTC reference. Fall back to the host clock rather than crash on an
    # out-of-range stamp.
    def _safe_stamp_seconds(self, seconds):
        if -2147483648 <= seconds < 2147483648:
            return seconds
        self.get_logger().warn(
            "Surveyor240 reported an invalid UTC time; falling back to host clock",
            once=True)
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def run(self):
        try:
            data = self.surveyor.wait_message([definitions.SURVEYOR240_ATOF_POINT_DATA,
                                                definitions.SURVEYOR240_ATTITUDE_REPORT,
                                                definitions.SURVEYOR240_WATER_STATS])
        except KeyboardInterrupt:
            return

        if data.message_id == definitions.SURVEYOR240_ATTITUDE_REPORT:
            vector = (data.up_vec_x, data.up_vec_y, data.up_vec_z)
            utc_sec = self._safe_stamp_seconds(data.utc_msec / 1000.0)
            self.header.stamp.sec = int(utc_sec)
            self.header.stamp.nanosec = int((utc_sec % 1.0) * 1e9)
            pitch = math.asin(vector[0])
            roll = math.atan2(vector[1], vector[2])

            # Convert roll/pitch (yaw=0) to quaternion
            cy, sy = 1.0, 0.0  # yaw = 0
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            imu_msg = Imu()
            imu_msg.header = self.header
            imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
            imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
            imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
            imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
            # Covariance unknown
            imu_msg.orientation_covariance[0] = -1

            self.pub_imu.publish(imu_msg)

            pitch_msg = Float32Stamped()                                                                        
            pitch_msg.header = self.header                                                                      
            pitch_msg.data = pitch                                                                            
            self.pub_pitch.publish(pitch_msg)                                                                   
            
            roll_msg = Float32Stamped()                                                                         
            roll_msg.header = self.header                                                                       
            roll_msg.data = roll                                                                                
            self.pub_roll.publish(roll_msg)  

        # Pressure and temperature data from the Bar30 sensor are transmitted in the Surveyor 240.
        # https://docs.ceruleansonar.com/c/surveyor-240-16/temperature-and-pressure-sensors
        if data.message_id == definitions.SURVEYOR240_WATER_STATS:
            temperature_msg = Temperature()
            temperature_msg.header = self.header
            temperature_msg.temperature = data.temperature

            pressure_msg = FluidPressure()
            pressure_msg.header = self.header
            pressure_msg.fluid_pressure = data.pressure

            self.pub_temperature.publish(temperature_msg)
            self.pub_pressure.publish(pressure_msg)
        
        # Publish A_TOF data as Float32MultiArrayStamped msg
        """
        #   data = [ angle_0, angle_1, ..., angle_N-1,
        #              tof_0,   tof_1, ...,   tof_N-1 ]
        #
        # Access:
        #   angle[i] = data[i]
        #   tof[i]   = data[N + i]
        """
        if data.message_id == definitions.SURVEYOR240_ATOF_POINT_DATA:
            atof_list = Surveyor240.create_atof_list(data)
            n = len(atof_list)
            ping_time_sec = self._safe_stamp_seconds(data.utc_msec / 1000.0 + data.listening_sec)
            self.header.stamp.sec = int(ping_time_sec)
            self.header.stamp.nanosec = int((ping_time_sec % 1.0) * 1e9)
            atof_msg = Float32MultiArrayStamped()
            atof_msg.header = self.header
            atof_msg.layout.dim = [
                MultiArrayDimension(label='row', size=2, stride=2*n),
                MultiArrayDimension(label='col', size=n, stride=n),
            ]
            atof_msg.layout.data_offset = 0
            atof_msg.data = [p.angle for p in atof_list] + [p.tof for p in atof_list]
            self.pub_atof.publish(atof_msg)

            #  Convert to cartesian and publish as PointCloud2
            #  float distance = 0.5 * speed_of_sound * tof_sec;
            #  float y = distance * sin(angle);
            #  float z = -distance * cos(angle);
            sound_speed = data.sos_mps
            cloud_data  = bytearray()
            for p in atof_list:
                r = 0.5 * sound_speed * p.tof
                x = 0.0
                y = -r * math.sin(p.angle)
                z = r * math.cos(p.angle)
                cloud_data += struct.pack('<fff', x, y, z)

            self.pc_msg.header   = self.header
            self.pc_msg.width    = n
            self.pc_msg.row_step = self.pc_msg.point_step * n
            self.pc_msg.data     = bytes(cloud_data)

            self.pub_pointcloud.publish(self.pc_msg)

    def shutdown(self):
        self.get_logger().info("Shutting down Surveyor240")
        self.surveyor.control_set_ping_parameters(ping_enable=False)
        if self.surveyor.iodev:
            try:
                self.surveyor.iodev.close()
            except Exception as e:
                self.get_logger().error(f"Failed to close device: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Surveyor_MBES_Driver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
