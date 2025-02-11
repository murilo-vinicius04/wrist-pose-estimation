import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
import numpy as np
from transforms3d.quaternions import mat2quat  # Import the new library function

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # Subscribe to the topic that sends the 3D wrist position
        self.sub_wrist = self.create_subscription(
            PointStamped,
            '/blazepose/right_wrist_3d',
            self.wrist_callback,
            10
        )
        self.latest_wrist = None

        # Calibration state variables:
        # 0: origin, 1: X-axis, 2: Y-axis, 3: Z-axis
        self.calib_count = 0  
        self.origin_cam = None
        self.x_axis_cam = None
        self.y_axis_cam = None
        self.z_axis_cam = None

        # Object for publishing static transform
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.get_logger().info("Calibration node initialized.")

        # Interval (in seconds) between calibration steps
        self.step_interval = 5.0

        # Start calibration procedure
        self.get_logger().info(
            f"[STEP 0] Place the wrist at the ORIGIN. Calibration will execute automatically in {self.step_interval} seconds."
        )
        self.schedule_next_step()

    def wrist_callback(self, msg: PointStamped):
        # Update the latest wrist position
        self.latest_wrist = msg.point

    def schedule_next_step(self):
        """
        Schedules the next calibration step using a one-shot timer.
        """
        self.timer = self.create_timer(self.step_interval, self.calibration_timer_callback)

    def calibration_timer_callback(self):
        # Cancel the timer to ensure a one-shot execution
        self.timer.cancel()

        if self.latest_wrist is None:
            self.get_logger().warn("No wrist data received. Check the sensor and position the wrist correctly.")
            # Reschedule the same step
            self.schedule_next_step()
            return

        # Execute the calibration step based on the counter
        if self.calib_count == 0:
            self.origin_cam = self.latest_wrist
            self.get_logger().info(
                f"[STEP 1] Origin stored. Now, move the hand to the FRONTAL position (X-axis). Calibration will execute in {self.step_interval} seconds."
            )
        elif self.calib_count == 1:
            self.x_axis_cam = self.latest_wrist
            self.get_logger().info(
                f"[STEP 2] X-axis stored. Now, move the hand to the LATERAL position (Y-axis). Calibration will execute in {self.step_interval} seconds."
            )
        elif self.calib_count == 2:
            self.y_axis_cam = self.latest_wrist
            self.get_logger().info(
                f"[STEP 3] Y-axis stored. Now, move the hand to the VERTICAL position (Z-axis). Calibration will execute in {self.step_interval} seconds."
            )
        elif self.calib_count == 3:
            self.z_axis_cam = self.latest_wrist
            # After the final step, compute and publish the transform
            self.compute_and_broadcast_transform()
            self.get_logger().info("Calibration complete!")
            self.calib_count = 0  # Optionally restart the process
            return
        else:
            self.get_logger().error("Unexpected calibration state.")
            self.calib_count = 0
            return

        # Increment the step counter and schedule the next step
        self.calib_count += 1
        self.schedule_next_step()

    def compute_and_broadcast_transform(self):
        """
        Calculates the rotation matrix and translation vector from the calibration points
        and publishes the transform.
        """
        # Compute base vectors from the differences of the calibration points
        u = np.array([
            self.x_axis_cam.x - self.origin_cam.x,
            self.x_axis_cam.y - self.origin_cam.y,
            self.x_axis_cam.z - self.origin_cam.z
        ])
        v = np.array([
            self.y_axis_cam.x - self.origin_cam.x,
            self.y_axis_cam.y - self.origin_cam.y,
            self.y_axis_cam.z - self.origin_cam.z
        ])
        w = np.array([
            self.z_axis_cam.x - self.origin_cam.x,
            self.z_axis_cam.y - self.origin_cam.y,
            self.z_axis_cam.z - self.origin_cam.z
        ])

        # Orthonormalize the vectors using the Gram-Schmidt process
        u = u / np.linalg.norm(u)
        v = v - np.dot(v, u) * u
        v = v / np.linalg.norm(v)
        w = w - np.dot(w, u) * u - np.dot(w, v) * v
        w = w / np.linalg.norm(w)

        # Create the rotation matrix with columns as the orthonormal basis vectors
        R = np.column_stack((u, v, w))

        # Convert the 3x3 rotation matrix to a quaternion.
        # mat2quat returns (w, x, y, z); we need to reorder to (x, y, z, w) for ROS.
        quat = mat2quat(R)
        quat_ros = [quat[1], quat[2], quat[3], quat[0]]

        # Compute the translation: transform the origin to the new frame
        rotated_origin = R.T @ np.array([self.origin_cam.x, self.origin_cam.y, self.origin_cam.z])
        trans = -rotated_origin

        # Prepare the transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = "base"
        transform.transform.translation.x = float(trans[0])
        transform.transform.translation.y = float(trans[1])
        transform.transform.translation.z = float(trans[2])
        transform.transform.rotation.x = float(quat_ros[0])
        transform.transform.rotation.y = float(quat_ros[1])
        transform.transform.rotation.z = float(quat_ros[2])
        transform.transform.rotation.w = float(quat_ros[3])

        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
