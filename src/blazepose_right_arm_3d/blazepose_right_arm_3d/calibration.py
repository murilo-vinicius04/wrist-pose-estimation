import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
import math

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        # Subscribe to 3D wrist positions
        self.sub_wrist = self.create_subscription(PointStamped, '/blazepose/right_wrist_3d', self.wrist_callback, 10)
        self.latest_wrist = None

        # Create a service to trigger the calibration workflow
        self.srv = self.create_service(Trigger, 'calibrate', self.calibrate_callback)

        # Calibration state
        self.calib_count = 0
        self.origin_cam = None
        self.x_axis_cam = None

        # Static transform broadcaster for publishing the transform
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.get_logger().info("Calibration node initialized.")

    def wrist_callback(self, msg: PointStamped):
        self.latest_wrist = msg.point

    def calibrate_callback(self, request, response):
        if self.latest_wrist is None:
            response.success = False
            response.message = "No wrist data received yet."
            return response

        if self.calib_count == 0:
            self.origin_cam = self.latest_wrist
            self.calib_count += 1
            response.success = True
            response.message = (f"First calibration point stored: origin_cam=({self.origin_cam.x:.2f}, "
                                f"{self.origin_cam.y:.2f}, {self.origin_cam.z:.2f}). "
                                "Call service again to set second calibration point.")
            self.get_logger().info(response.message)
        elif self.calib_count == 1:
            self.x_axis_cam = self.latest_wrist

            # Compute yaw correction from the vector between origin and x_axis points
            delta_x = self.x_axis_cam.x - self.origin_cam.x
            delta_z = self.x_axis_cam.z - self.origin_cam.z
            theta = math.atan2(delta_z, delta_x)

            # Compute the rotated origin
            rotated_origin_x = self.origin_cam.x * math.cos(theta) - self.origin_cam.z * math.sin(theta)
            rotated_origin_y = self.origin_cam.y  # unchanged
            rotated_origin_z = self.origin_cam.x * math.sin(theta) + self.origin_cam.z * math.cos(theta)

            # Compute translation T = -R * origin_cam (using rotated origin)
            trans_x = -rotated_origin_x
            trans_y = -rotated_origin_y
            trans_z = -rotated_origin_z

            # Build transform message
            transform = TransformStamped()
            now = self.get_clock().now().to_msg()
            transform.header.stamp = now
            transform.header.frame_id = "base"
            transform.child_frame_id = "camera_color_optical_frame"
            transform.transform.translation.x = trans_x
            transform.transform.translation.y = trans_y
            transform.transform.translation.z = trans_z

            # Compute quaternion from yaw (rotation only about z)
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            # Publish the static transform
            self.tf_broadcaster.sendTransform(transform)

            response.success = True
            response.message = (f"Calibration complete. Published transform with translation "
                                f"({trans_x:.2f}, {trans_y:.2f}, {trans_z:.2f}) and yaw {theta:.2f} rad.")
            self.get_logger().info(response.message)

            # Reset calibration state for potential future calibrations
            self.calib_count = 0
            self.origin_cam = None
            self.x_axis_cam = None
        else:
            response.success = False
            response.message = "Unexpected calibration state."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
