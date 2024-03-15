import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import transformations
import tf2_ros

class OdomIMUFusion(Node):
    def __init__(self):
        super().__init__('odom_imu_fusion')
        # Subscribers
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
        # Publisher
        self.publisher_odom = self.create_publisher(Odometry, '/fused_odom', 10)
        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Current state
        self.current_position_x = 0.0
        self.current_velocity_x = 0.0
        self.last_odom_time = self.get_clock().now()
        self.current_orientation = Quaternion()
        # Initialize orientation
        self.current_orientation.w = 1.0  # Assume initial orientation is neutral
        
    def odom_callback(self, msg: Odometry):
        # Log received odometry data
        self.get_logger().info(f'Received odometry: position={msg.pose.pose.position.x}, velocity={msg.twist.twist.linear.x}')

        # Calculate time difference
        now = self.get_clock().now()
        dt = now.seconds_nanoseconds()[0] - self.last_odom_time.seconds_nanoseconds()[0]
        dt += (now.seconds_nanoseconds()[1] - self.last_odom_time.seconds_nanoseconds()[1]) * 1e-9
        self.last_odom_time = now

        # Update position based on velocity
        self.current_velocity_x = msg.twist.twist.linear.x
        self.current_position_x += self.current_velocity_x * dt

        # Publish fused odometry and TF
        self.publish_fused_odom()


    def imu_callback(self, msg: Imu):
        # Log received IMU data
        self.get_logger().info(f'Received IMU: orientation={msg.orientation}')

        self.current_orientation = msg.orientation
        # Publish fused odometry and TF every IMU update for simplicity
        self.publish_fused_odom()



    def publish_fused_odom(self):
        # Log publishing fused odometry
        self.get_logger().info(f'Publishing fused odometry: position={self.current_position_x}, velocity={self.current_velocity_x}, orientation={self.current_orientation}')

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.current_position_x
        odom_msg.pose.pose.orientation = self.current_orientation
        odom_msg.twist.twist.linear.x = self.current_velocity_x
        self.publisher_odom.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.current_position_x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = self.current_orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_imu_fusion = OdomIMUFusion()
    rclpy.spin(odom_imu_fusion)
    # Cleanup
    odom_imu_fusion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
