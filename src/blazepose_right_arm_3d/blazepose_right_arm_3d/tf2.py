from rclpy.node import Node
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # <-- Add this import!

# ...existing code...

class WristTransformNode(Node):
    def __init__(self):
        super().__init__('wrist_transform_node')
        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Set up publisher
        self.publisher = self.create_publisher(PointStamped, '/teleop_wrist_position_base', 10)
        
        # Set up subscription
        self.subscription = self.create_subscription(
            PointStamped, 
            '/blazepose/right_wrist_3d', 
            self.listener_callback, 
            10
        )
        self.get_logger().info('WristTransformNode initialized.')
    
    def listener_callback(self, msg: PointStamped):
        try:
            # Attempt to transform msg to the 'base' frame
            transform = self.tf_buffer.transform(msg, 'base', timeout=rclpy.duration.Duration(seconds=1.0))
            self.publisher.publish(transform)
            self.get_logger().info('Published transformed wrist position.')
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            
# ...existing code...

def main(args=None):
    rclpy.init(args=args)
    node = WristTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
