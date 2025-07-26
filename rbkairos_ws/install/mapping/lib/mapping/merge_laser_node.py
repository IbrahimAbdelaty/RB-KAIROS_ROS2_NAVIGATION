#!/usr/bin/env python3
#########################################################3

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from rclpy.time import Time
# from sensor_msgs.msg import LaserScan
# import numpy as np
# from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import PointStamped
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# from tf2_geometry_msgs import do_transform_point
# import tf2_ros

# class ScanMerger(Node):
#     def __init__(self):
#         super().__init__('scan_merger')
        
#         # Parameters
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('output_frame', 'robot_base_link'),
#                 ('front_scan_topic', 'robot/front_laser/scan'),
#                 ('rear_scan_topic', 'robot/rear_laser/scan'),
#                 ('merged_scan_topic', 'merged_scan'),
#                 ('angle_min', -3.14159),  # -π radians
#                 ('angle_max', 3.14159),   # π radians
#                 ('angle_increment', 0.0174533),  # ~1 degree
#                 ('range_min', 0.1),
#                 ('range_max', 30.0),
#                 ('scan_time', 0.05),      # 20 Hz 0.05
#                 ('transform_timeout', 1.0),  # Increased timeout
#                 ('buffer_duration', 10.0)   # Buffer history duration
#             ])
#         self.declare_parameter('base_frame', 'robot_base_link')
#         self.declare_parameter('front_laser_frame', 'robot_front_laser_base_link')
#         self.declare_parameter('rear_laser_frame', 'robot_rear_laser_base_link')

#         self.base_frame = self.get_parameter('base_frame').value
#         self.front_laser_frame = self.get_parameter('front_laser_frame').value
#         self.rear_laser_frame = self.get_parameter('rear_laser_frame').value            
        
        
#         # TF setup with longer buffer duration
#         self.tf_buffer = Buffer(cache_time=Duration(
#             seconds=self.get_parameter('buffer_duration').value
#         ))
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         # Publisher for merged scan
#         self.merged_pub = self.create_publisher(
#             LaserScan,
#             self.get_parameter('merged_scan_topic').value,
#             10
#         )
        
#         # Synchronized subscribers
#         front_sub = Subscriber(
#             self, 
#             LaserScan,
#             self.get_parameter('front_scan_topic').value
#         )
#         rear_sub = Subscriber(
#             self, 
#             LaserScan,
#             self.get_parameter('rear_scan_topic').value
#         )
        
#         # Time synchronizer with larger queue
#         self.sync = ApproximateTimeSynchronizer(
#             [front_sub, rear_sub],
#             queue_size=30,  # Larger queue
#             slop=0.2       # Larger time slop
#         )
#         self.sync.registerCallback(self.scan_callback)
        
#         self.get_logger().info("2D Scan Merger Node Initialized")

#     def transform_point(self, x, y, source_frame, target_frame, stamp):
#         """Transform a point between frames using TF2"""
#         point_stamped = PointStamped()
#         point_stamped.header.frame_id = source_frame
#         point_stamped.header.stamp = stamp
#         point_stamped.point.x = x
#         point_stamped.point.y = y
#         point_stamped.point.z = 0.0
        
#         try:
#             # Get transform with timeout
#             timeout_duration = Duration(seconds=self.get_parameter('transform_timeout').value)
            
#             # Lookup transform at the specific time of the scan
#             transform = self.tf_buffer.lookup_transform(
#                 target_frame,
#                 source_frame,
#                 stamp,
#                 timeout_duration
#             )
            
#             # Apply transform
#             transformed = do_transform_point(point_stamped, transform)
#             return transformed.point.x, transformed.point.y
            
#         except tf2_ros.LookupException as e:
#             self.get_logger().error(f'TF lookup failed: {str(e)}', throttle_duration_sec=5)
#             return None, None
#         except tf2_ros.ConnectivityException as e:
#             self.get_logger().error(f'TF connectivity issue: {str(e)}', throttle_duration_sec=5)
#             return None, None
#         except tf2_ros.ExtrapolationException as e:
#             # Try to get the latest available transform
#             try:
#                 self.get_logger().warning("TF extrapolation error, using latest transform", 
#                                          throttle_duration_sec=5)
#                 transform = self.tf_buffer.lookup_transform(
#                     target_frame,
#                     source_frame,
#                     rclpy.time.Time(),  # Latest available
#                     timeout_duration
#                 )
#                 transformed = do_transform_point(point_stamped, transform)
#                 return transformed.point.x, transformed.point.y
#             except Exception as e2:
#                 self.get_logger().error(f'Fallback TF failed: {str(e2)}', throttle_duration_sec=5)
#                 return None, None
#         except Exception as e:
#             self.get_logger().error(f'TF transform error: {str(e)}', throttle_duration_sec=5)
#             return None, None

#     def polar_to_cartesian(self, range_val, angle):
#         """Convert polar coordinates to cartesian"""
#         x = range_val * np.cos(angle)
#         y = range_val * np.sin(angle)
#         return x, y

#     def cartesian_to_polar(self, x, y):
#         """Convert cartesian coordinates to polar"""
#         if x == 0 and y == 0:
#             return 0.0, 0.0
#         range_val = np.sqrt(x**2 + y**2)
#         angle_val = np.arctan2(y, x)
#         return range_val, angle_val

#     def scan_callback(self, front_msg, rear_msg):
#         """Process synchronized scan messages"""
#         try:
#             # Use the latest timestamp
#             stamp = self.get_clock().now().to_msg()
            
#             # Initialize merged ranges
#             angle_min = self.get_parameter('angle_min').value
#             angle_max = self.get_parameter('angle_max').value
#             angle_inc = self.get_parameter('angle_increment').value
#             num_bins = int((angle_max - angle_min) / angle_inc)
#             merged_ranges = [float('nan')] * num_bins
            
#             # Output frame
#             output_frame = self.get_parameter('output_frame').value
            
#             # Process both scans
#             for scan_msg in [front_msg, rear_msg]:
#                 for i, r in enumerate(scan_msg.ranges):
#                     # Skip invalid ranges
#                     if (r < scan_msg.range_min or r > scan_msg.range_max or
#                         r < self.get_parameter('range_min').value or
#                         r > self.get_parameter('range_max').value or
#                         np.isnan(r)):
#                         continue
                    
#                     # Calculate original angle
#                     angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    
#                     # Convert to cartesian
#                     x, y = self.polar_to_cartesian(r, angle)
                    
#                     # Transform to output frame
#                     x_tf, y_tf = self.transform_point(
#                         x, y,
#                         scan_msg.header.frame_id,
#                         output_frame,
#                         stamp
#                     )

                    
#                     if x_tf is None or y_tf is None:
#                         continue
                    
#                     # Convert back to polar in output frame
#                     new_range, new_angle = self.cartesian_to_polar(x_tf, y_tf)
                    
#                     # Normalize angle to [-π, π]
#                     if new_angle > np.pi:
#                         new_angle -= 2 * np.pi
#                     elif new_angle < -np.pi:
#                         new_angle += 2 * np.pi
                    
#                     # Skip if outside merged scan range
#                     if new_angle < angle_min or new_angle > angle_max:
#                         continue
                    
#                     # Find bin index
#                     bin_idx = int((new_angle - angle_min) / angle_inc)
                    
#                     # Ensure bin index is within bounds
#                     bin_idx = max(0, min(num_bins - 1, bin_idx))
                    
#                     # Update bin with closest range
#                     current = merged_ranges[bin_idx]
#                     if np.isnan(current) or new_range < current:
#                         merged_ranges[bin_idx] = new_range
            
#             # Create merged scan message
#             merged_scan = LaserScan()
#             merged_scan.header.stamp = self.get_clock().now().to_msg()
#             merged_scan.header.frame_id = output_frame
#             merged_scan.angle_min = angle_min
#             merged_scan.angle_max = angle_max
#             merged_scan.angle_increment = angle_inc
#             merged_scan.time_increment = 0.0
#             merged_scan.scan_time = self.get_parameter('scan_time').value
#             merged_scan.range_min = self.get_parameter('range_min').value
#             merged_scan.range_max = self.get_parameter('range_max').value
#             merged_scan.ranges = merged_ranges
            
#             # Publish merged scan
#             self.merged_pub.publish(merged_scan)
            
#         except Exception as e:
#             self.get_logger().error(f'Processing error: {str(e)}', throttle_duration_sec=5)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ScanMerger()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
##################################################################
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter  # Make sure this is imported

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')

        # Set use_sim_time to True without declaring it again
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('output_frame', 'robot_base_footprint'),
                ('front_scan_topic', 'robot/front_laser/scan'),
                ('rear_scan_topic', 'robot/rear_laser/scan'),
                ('merged_scan_topic', 'merged_scan'),
                ('angle_min', -3.14159),
                ('angle_max', 3.14159),
                ('angle_increment', 0.0174533),
                ('range_min', 0.1),
                ('range_max', 30.0),
                ('scan_time', 0.05),
                ('transform_timeout', 0.1),
                ('buffer_duration', 10.0),
                ('front_laser_frame', 'robot_front_laser_link'),
                ('rear_laser_frame', 'robot_rear_laser_link'),
            ]
        )

############################################3
        # self.set_parameters([rclpy.parameter.Parameter(
        #     'use_sim_time',
        #     rclpy.Parameter.Type.BOOL,
        #     self.get_parameter('use_sim_time').value
        # )])

        self.latest_merged_scan = None
        self.timer = self.create_timer(0.05, self.publish_timer_callback)  # 20 Hz
##############################################

        self.output_frame = self.get_parameter('output_frame').value
        self.front_frame = self.get_parameter('front_laser_frame').value
        self.rear_frame = self.get_parameter('rear_laser_frame').value

        self.tf_buffer = Buffer(cache_time=Duration(seconds=self.get_parameter('buffer_duration').value))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.merged_pub = self.create_publisher(
            LaserScan,
            self.get_parameter('merged_scan_topic').value,
            qos
        )

        front_sub = Subscriber(
            self, LaserScan,
            self.get_parameter('front_scan_topic').value,
            qos_profile=qos
        )
        rear_sub = Subscriber(
            self, LaserScan,
            self.get_parameter('rear_scan_topic').value,
            qos_profile=qos
        )

        self.sync = ApproximateTimeSynchronizer(
            [front_sub, rear_sub],
            queue_size=10,
            slop=0.3
        )
        self.sync.registerCallback(self.scan_callback)

        self.get_logger().info("Scan Merger Initialized with QoS")

    def get_transform(self, target_frame, source_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.get_parameter('transform_timeout').value)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            try:
                self.get_logger().warning("Using latest transform", throttle_duration_sec=2)
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().error(f'Transform failed: {str(e)}', throttle_duration_sec=1)
                return None
        except tf2_ros.ExtrapolationException:
            self.get_logger().warning("TF extrapolation error", throttle_duration_sec=1)
            return None

    def transform_scan(self, scan_msg, target_frame):
        transform = self.get_transform(target_frame, scan_msg.header.frame_id, scan_msg.header.stamp)
        if not transform:
            return []

        ranges = []
        angles = []

        for i, r in enumerate(scan_msg.ranges):
            if (r < scan_msg.range_min or r > scan_msg.range_max or 
                np.isnan(r) or r < self.get_parameter('range_min').value):
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            point = PointStamped()
            point.header = scan_msg.header
            point.point.x = x
            point.point.y = y
            transformed = do_transform_point(point, transform)

            new_r = np.sqrt(transformed.point.x**2 + transformed.point.y**2)
            new_angle = np.arctan2(transformed.point.y, transformed.point.x)

            ranges.append(new_r)
            angles.append(new_angle)

        return list(zip(ranges, angles))

    def scan_callback(self, front_msg, rear_msg):
        try:
            front_data = self.transform_scan(front_msg, self.output_frame)
            rear_data = self.transform_scan(rear_msg, self.output_frame)
            all_data = front_data + rear_data

            if not all_data:
                return

            angle_min = self.get_parameter('angle_min').value
            angle_max = self.get_parameter('angle_max').value
            angle_inc = self.get_parameter('angle_increment').value
            num_bins = int((angle_max - angle_min) / angle_inc) + 1
            merged_ranges = [float('nan')] * num_bins

            for r, angle in all_data:
                if angle > np.pi:
                    angle -= 2 * np.pi
                elif angle < -np.pi:
                    angle += 2 * np.pi

                if not (angle_min <= angle <= angle_max):
                    continue

                bin_idx = int((angle - angle_min) / angle_inc)
                bin_idx = max(0, min(num_bins - 1, bin_idx))

                current = merged_ranges[bin_idx]
                if np.isnan(current) or r < current:
                    merged_ranges[bin_idx] = r

            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = self.output_frame
            merged_scan.angle_min = angle_min
            merged_scan.angle_max = angle_max
            merged_scan.angle_increment = angle_inc
            merged_scan.time_increment = 0.0
            merged_scan.scan_time = self.get_parameter('scan_time').value
            merged_scan.range_min = self.get_parameter('range_min').value
            merged_scan.range_max = self.get_parameter('range_max').value
            merged_scan.ranges = merged_ranges

            # self.merged_pub.publish(merged_scan)
            self.latest_merged_scan = merged_scan


        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}', throttle_duration_sec=1)

    def publish_timer_callback(self):
        if self.latest_merged_scan:
            self.merged_pub.publish(self.latest_merged_scan)

            
def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

