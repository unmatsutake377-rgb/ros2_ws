#Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          Image,
          'video_frames',
          self.listener_callback,
          10)
        self.subscription # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Display image
        self.process_image(current_frame)
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

    def process_image(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

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
        if len(detected_colors) > 0:
            print(detected_colors[0])

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
                detected_shapes.append("Ractangle")
            elif len(approx) > 12:
                detected_shapes.append("Circle")

        if len(detected_shapes) > 0:
            print(detected_shapes[0])


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()