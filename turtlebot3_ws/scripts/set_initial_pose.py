import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class RobotPoseInitializer(Node):
    def __init__(self):
        super().__init__('robot_pose_initializer')
        
        # Create publisher for initial pose
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Timer to publish pose once
        self.pose_timer = self.create_timer(0.5, self.set_robot_initial_pose)
        self.pose_published = False
        self.publish_attempts = 0
        self.max_attempts = 3
        
        self.get_logger().info('Robot Pose Initializer node started')

    def set_robot_initial_pose(self):
        """Publish the initial pose of the robot"""
        if self.pose_published or self.publish_attempts >= self.max_attempts:
            return

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        
        # Set header information
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # ===============================================
        # 🔧 MODIFY THESE VALUES FOR YOUR INITIAL POSE
        # ===============================================
        initial_x = 0.5      # X position in meters
        initial_y = 0.0      # Y position in meters
        initial_z = 0.0      # Z position (usually 0.0 for ground robots)
        initial_yaw = 0.0    # Yaw angle in radians (0 = facing positive X)
        # ===============================================

        # Set position
        pose_msg.pose.pose.position.x = initial_x
        pose_msg.pose.pose.position.y = initial_y
        pose_msg.pose.pose.position.z = initial_z

        # Convert yaw to quaternion and set orientation
        quat_z = math.sin(initial_yaw / 2.0)
        quat_w = math.cos(initial_yaw / 2.0)
        
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = quat_z
        pose_msg.pose.pose.orientation.w = quat_w

        # Set covariance matrix for pose uncertainty
        # Diagonal elements: [x, y, z, roll, pitch, yaw]
        pose_uncertainty = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,          # x variance
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,          # y variance  
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,           # z variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,           # roll variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,           # pitch variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068539       # yaw variance (~15 degrees)
        ]
        pose_msg.pose.covariance = pose_uncertainty

        # Publish the pose
        self.get_logger().info(
            f'Setting robot initial pose: x={initial_x}, y={initial_y}, yaw={initial_yaw:.2f} rad'
        )
        self.pose_publisher.publish(pose_msg)
        
        self.publish_attempts += 1
        
        # Mark as published after successful attempt
        if self.publish_attempts >= 2:  # Publish twice for reliability
            self.pose_published = True
            self.get_logger().info('Initial pose successfully published')


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down robot pose initializer')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()