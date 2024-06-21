import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from yolo_msgs.msg import Detections, Detection  # Import custom messages
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.detection_publisher = self.create_publisher(Detections, 'yolov8_detections', 10)  # New publisher
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO("/home/jetson/test/best_by_ayush.pt")  # Replace with your model path

        # Open the text file containing class names
        with open('/home/jetson/test/coco1.txt', 'r') as f:
            self.class_list = f.read().splitlines()

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Perform object detection
        results = self.model.predict(source=cv_image, stream=True, conf=0.7)

        # Publish detection results
        self.publish_detections(results)

    def publish_detections(self, results):
        detection_msg = Detections()
        detection_msg.header = Header()
        detection_msg.header.stamp = self.get_clock().now().to_msg()

        for result in results:
            boxes = result.boxes
            for box in boxes:
                detection = Detection()
                detection.class_id = int(box.cls)
                detection.confidence = float(box.conf)
                detection.x_min = float(box.xyxy[0][0])
                detection.y_min = float(box.xyxy[0][1])
                detection.x_max = float(box.xyxy[0][2])
                detection.y_max = float(box.xyxy[0][3])
                detection_msg.detections.append(detection)

        self.detection_publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    yolov8_node = YOLOv8Node()
    rclpy.spin(yolov8_node)
    yolov8_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
