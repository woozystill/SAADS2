import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import time


class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_ros_node')

        self.bridge = CvBridge()

        self.model = YOLO("yolo11n.pt")

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.last_time = time.time()

        self.get_logger().info("YOLO ROS node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        results = self.model(
            frame,
            device=0,
            conf=0.25,
            verbose=False
        )

        annotated = results[0].plot()

        now = time.time()
        fps = 1.0 / (now - self.last_time)
        self.last_time = now

        cv2.putText(
            annotated,
            f"FPS: {fps:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        cv2.imshow("YOLO ROS", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloRosNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
