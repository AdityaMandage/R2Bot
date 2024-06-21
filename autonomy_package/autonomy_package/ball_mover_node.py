import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import Detections  # Ensure this import is correct
from yolo_msgs.msg import Detection  # Ensure this import is correct
from collections import deque

class BallMoverNode(Node):
    def __init__(self):
        super().__init__('ball_mover_node')
        self.subscription = self.create_subscription(
            Detections,
            'yolov8_detections',
            self.detection_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.target_ball_class_id = 0  # Assuming class_id 0 is the target ball (redball, for example)
        self.close_enough_distance = 25000  # Adjust bounding box area threshold for stopping
        self.tracking_ball = False
        self.lost_tracking_counter = 0
        self.lost_tracking_threshold = 10  # Adjust for how many cycles to wait before considering the ball lost
        self.detection_history = deque(maxlen=5)  # History for smoothing detections

    def detection_callback(self, msg):
        target_detections = [d for d in msg.detections if d.class_id == self.target_ball_class_id]

        if not target_detections:
            self.lost_tracking_counter += 1
            if self.lost_tracking_counter > self.lost_tracking_threshold:
                self.tracking_ball = False
                self.get_logger().info('Lost tracking of the ball.')
            return

        self.lost_tracking_counter = 0
        self.tracking_ball = True

        # Choose the closest ball (largest bounding box area)
        chosen_detection = max(target_detections, key=lambda d: (d.x_max - d.x_min) * (d.y_max - d.y_min))

        self.detection_history.append(chosen_detection)
        smoothed_detection = self.smooth_detections(self.detection_history)

        self.move_towards_ball(smoothed_detection)

    def smooth_detections(self, detection_history):
        # Average the bounding box coordinates over the history
        avg_x_min = sum(d.x_min for d in detection_history) / len(detection_history)
        avg_y_min = sum(d.y_min for d in detection_history) / len(detection_history)
        avg_x_max = sum(d.x_max for d in detection_history) / len(detection_history)
        avg_y_max = sum(d.y_max for d in detection_history) / len(detection_history)
        
        return Detection(
            class_id=detection_history[0].class_id,
            confidence=detection_history[0].confidence,
            x_min=avg_x_min,
            y_min=avg_y_min,
            x_max=avg_x_max,
            y_max=avg_y_max
        )

    def move_towards_ball(self, detection):
        twist = Twist()

        # Calculate the center of the detection
        x_center = (detection.x_min + detection.x_max) / 2
        y_center = (detection.y_min + detection.y_max) / 2

        # Assuming the camera resolution is 640x480
        image_width = 640
        image_height = 480
        x_offset = x_center - (image_width / 2)
        y_offset = y_center - (image_height / 2)

        # Logging detection details
        self.get_logger().info(f'Detection - x_center: {x_center}, y_center: {y_center}, x_offset: {x_offset}, y_offset: {y_offset}')

        # Calculate angular velocity
        if abs(x_offset) > 20:  # Adjust threshold as needed
            twist.angular.z = -0.004 * x_offset  # Increase gain for faster turning
        else:
            twist.angular.z = 0.0

        # Logging angular velocity
        self.get_logger().info(f'Setting angular.z: {twist.angular.z}')

        # For simplicity, assuming a constant forward velocity
        twist.linear.x = 0.4  # Increase speed

        # Calculate the area of the bounding box
        bounding_box_area = (detection.x_max - detection.x_min) * (detection.y_max - detection.y_min)

        # Adjust the linear velocity based on the bounding box area
        if bounding_box_area >= self.close_enough_distance:
            twist.linear.x = 0.0

        # Logging linear velocity
        self.get_logger().info(f'Setting linear.x: {twist.linear.x}')

        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallMoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
