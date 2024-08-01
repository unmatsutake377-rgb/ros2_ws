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

        # Create empty list of detected color and shapes
        self.detected_color_list = []
        self.detected_shape_list = []

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Process method that detect shapes and color
        processed_frame = self.process_image_callback(current_frame)


        # Display image
        cv2.imshow("camera", current_frame) # Origin Image
        cv2.imshow("shape detected", processed_frame) # Processed Image

        cv2.waitKey(1)


    def set_label(self, image, label, contour):
        """
        Put labels on image.
        """
        fontface = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.5
        thickness = 1
        baseline = 0

        text_size, baseline = cv2.getTextSize(label, fontface, scale, thickness)
        r = cv2.boundingRect(contour)

        pt = (r[0] + ((r[2] - text_size[0]) // 2), r[1] + ((r[3] + text_size[1]) // 2))
        cv2.rectangle(image, (pt[0], pt[1] + baseline), (pt[0] + text_size[0], pt[1] - text_size[1]), (200, 200, 200), cv2.FILLED)
        cv2.putText(image, label, pt, fontface, scale, (0, 0, 0), thickness, 8)

    def process_image_callback(self, msg, response):
        """
        Detecte shapes and color.
        """
        self.get_logger().info('Processing image at perfect coordinate.')
        img_input = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_gray = cv2.cvtColor(img_input, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(img_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        img_result = img_input.copy()

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Find shape's moment
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
            else:
                cX, cY = 0, 0

            # Detecte color.
            color = img_input[cY, cX]
            b, g, r = color
            color_str = f"R: {r} G: {g}, B: {b}"

            #Define detected color's name.
            if b == 0 and g == 0 and r == 0: # Black
                self.detected_color_list.append("black")
                detected_color = "black"
                print(color_str)
            elif r > g and r > b and (g + b) < (r / 2): # Orange
                self.detected_color_list.append("orange")
                detected_color = "orange"
                print(color_str)
            elif r > g and r > b: # Red
                self.detected_color_list.append("red")
                detected_color = "red"
                print(color_str)
            elif g > r and b > g: #Green
                self.detected_color_list.append("green")
                detected_color = "green"
                print(color_str)
            elif b > r and b > g: # Blue
                self.detected_color_list.append("blue")
                detected_color = "blue"
                print(color_str)
            elif b == 255 and g == 255 and r == 255: # White
                self.detected_color_list.append("white")
                detected_color = "white"
                print(color_str)
            else: # Can't find Color
                print("Error") #TODO: Add Error message

            if cv2.contourArea(approx) > 100:
                size = len(approx)

                if size % 2 == 0:
                    cv2.line(img_result, tuple(approx[0][0]), tuple(approx[-1][0]), (0, 255, 0), 3)
                    for k in range(size - 1):
                        cv2.line(img_result, tuple(approx[k][0]), tuple(approx[k + 1][0]), (0, 255, 0), 3)
                    for k in range(size):
                        cv2.circle(img_result, tuple(approx[k][0]), 3, (0, 0, 255))

                else:
                    cv2.line(img_result, tuple(approx[0][0]), tuple(approx[-1][0]), (0, 255, 0), 3)
                    for k in range(size - 1):
                        cv2.line(img_result, tuple(approx[k][0]), tuple(approx[-1][0]), (0, 255, 0), 3)
                    for k in range(size):
                        cv2.circle(img_result, tuple(approx[k][0]), 3, (0, 0, 255))


                # Three Vertex
                if size == 3:
                    self.detected_shape_list.append("triangle")
                    detected_shape = "triangle"
                    self.set_label(img_result, "triangle", contour)

                # Four Vertex
                elif size == 4:
                    self.detected_shape_list.append("rectangle")
                    detected_color = "rectangle"
                    self.set_label(img_result, "rectangle", contour)

                # FIve Vertex
                elif size == 5:
                    self.detected_shape_list.append("pentagon")
                    detected_shape = "pentagon"
                    self.set_label(img_result, "pentagon", contour)

                # Six Vertex
                elif size == 6:
                    self.detected_shape_list.append("hexagon")
                    detected_shape = "hexagon"
                    self.set_label(img_result, "hexagon", contour)

                # Heart (Or Eight Vertex)
                # Origin meaning is Eight Vertex
                # but in this case we need to detect heart
                # and heart be detected that has 8 vertex
                elif size == 8:
                    self.detected_shape_list.append("heart")
                    detected_shape = "heart"
                    self.set_label(img_result, "heart", contour)

                # Cross (Or 12 Vertex)
                # Same with the heart case
                # we need to detect cross
                elif size == 12:
                    self.detected_shape_list.append("cross")
                    detected_shape = "cross"
                    self.set_label(img_result, "cross", contour)

                else:
                    self.detected_shape_list.append(str(size)+"_angle_shape")
                    detected_shape = str(size)+"_angle_shape"
                    self.set_label(img_result, str(size), contour)


                # Find the correct way
                if detected_color == "Goal color here" and detected_shape == "goal shape here":
                    
                    # Left way
                    if cX < int("experimental value(line 1)"):
                        response.way = 1
                        print("Image process completed successfully.")
                    
                    # Middle Way
                    elif int("experimental value(line 1)") < cX < int("experimental value(line 2)"):
                        response.way = 2
                        print("Image process completed successfully.")
                    
                    # Right Way
                    elif int("experimental value(line 2)") - cX < int("experimental value(line 3)"):
                        response.way = 3
                        print("Image process completed successfully.")
                    else:
                        print("Error") #TODO Add Error message

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