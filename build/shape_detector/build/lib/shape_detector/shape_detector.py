import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')
        print("Going 5 by 5")
        cam = cv2.VideoCapture(2)
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.process_image(cv_image)

    def process_image(self, cv_image):
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color range for detection (example: red color)
        color_ranges = {
            "red": ([0, 120, 70], [10, 255, 255], [170, 120, 70], [180, 255, 255]),
            "green": ([40, 70, 70], [80, 255, 255]),
            "blue": ([100, 150, 0], [140, 255, 255]),
            "yellow": ([25, 150, 150], [35, 255, 255]),
            "cyan": ([85, 150, 150], [35, 255, 255]),
            "magenta": ([145, 150, 150], [155, 255, 255]),
        }

        detected_colors = []

        for color, (lower1, upper1, *rest) in color_ranges.items():
            mask1 = cv2.inRange(hsv_image, np.array(lower1), np.array(upper1))

            if rest:
                lower2, upper2 = rest
                mask2 = cv2.inRange(hsv_image, np.array(lower2), np.array(upper2))
                mask = mask1 + mask2

            else:
                mask = mask1

            if cv2.countNonZero(mask) > 0:
                detected_colors.append(color)


        # Detect shapes

        detected_shapes = []
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            cv2.drawContours(cv_image, [approx], 0, (0, 255, 0), 5)
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 10
            if len(approx) == 3:
                detected_shapes.append("triangle")
            elif len(approx) == 4:
                detected_shapes.append("Ractangle") #cv2.putText(cv_image, "Rectangle", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            elif len(approx) > 12:
                detected_shapes.append("Circle") #cv2.putText(cv_image, "Circle", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image
        cv2.imshow("Color and Shape Detection", cv_image)
        cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    color_shape_detector = ShapeDetector()
    rclpy.spin(color_shape_detector)
    color_shape_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
