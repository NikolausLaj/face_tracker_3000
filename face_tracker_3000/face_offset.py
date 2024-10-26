import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import cv2

# TODO: Add keyword argument, which alows to set BOOL value to display video stream.

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_offset_node')

        # Declare and get parameters
        self.declare_parameter('camera_input', 0)
        self.declare_parameter('face_detection.scale_factor', 1.1)
        self.declare_parameter('face_detection.min_neighbors', 5)

        self.declare_parameter('colors.center_dot.radius', 3)
        self.declare_parameter('colors.center_dot.color', [0, 255, 0])
        self.declare_parameter('colors.center_dot.thickness', -1)

        self.declare_parameter('colors.face_dot.radius', 3)
        self.declare_parameter('colors.face_dot.color', [0, 0, 255])
        self.declare_parameter('colors.face_dot.thickness', -1)

        self.declare_parameter('colors.connection_line.color', [255, 255, 255])
        self.declare_parameter('colors.connection_line.thickness', 2)

        # Retrieve parameters
        self.camera_input = self.get_parameter('camera_input').get_parameter_value().integer_value
        self.scale_factor = self.get_parameter('face_detection.scale_factor').get_parameter_value().double_value
        self.min_neighbors = self.get_parameter('face_detection.min_neighbors').get_parameter_value().integer_value

        self.center_dot = {
            "radius": self.get_parameter('colors.center_dot.radius').get_parameter_value().integer_value,
            "color": tuple(self.get_parameter('colors.center_dot.color').get_parameter_value().integer_array_value),
            "thickness": self.get_parameter('colors.center_dot.thickness').get_parameter_value().integer_value
        }

        self.face_dot = {
            "radius": self.get_parameter('colors.face_dot.radius').get_parameter_value().integer_value,
            "color": tuple(self.get_parameter('colors.face_dot.color').get_parameter_value().integer_array_value),
            "thickness": self.get_parameter('colors.face_dot.thickness').get_parameter_value().integer_value
        }

        self.connection_line = {
            "color": tuple(self.get_parameter('colors.connection_line.color').get_parameter_value().integer_array_value),
            "thickness": self.get_parameter('colors.connection_line.thickness').get_parameter_value().integer_value
        }

        # Log retrieved parameters
        self.get_logger().info(f"Camera input: {self.camera_input}")
        self.get_logger().info(f"Face detection scale factor: {self.scale_factor}")
        self.get_logger().info(f"Face detection min neighbors: {self.min_neighbors}")
        self.get_logger().info(f"Center dot settings: {self.center_dot}")
        self.get_logger().info(f"Face dot settings: {self.face_dot}")
        self.get_logger().info(f"Connection line settings: {self.connection_line}")


        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.cap = cv2.VideoCapture(self.camera_input)

        self.image_center = (
            int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2),
            int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // 2)
            )

        self.coord_pub = self.create_publisher(Point, 'face_offset', 10)

        if not self.cap.isOpened():
            self.get_logger().error("Unable to access the webcam")
        else:
            self.get_logger().info("Webcam successfully opened")

    def spin(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                self.frame_callback(frame)
            else:
                self.get_logger().error("Failed to grab frame from webcam")

            # just for visualization
            cv2.imshow('WebCam', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def frame_callback(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray_frame, self.scale_factor, self.min_neighbors)

        for (x, y, w, h) in faces:
            fx = (2 * x + w) // 2
            fy = (2 * y + h) // 2

            ex = self.image_center[0] - fx
            ey = self.image_center[1] - fy

            offset = Point(x=float(ex), y=float(ey), z=0.0)
            
            self.coord_pub.publish(offset)

            # just for visualization
            cv2.circle(frame, (fx, fy), self.center_dot["radius"], self.center_dot["color"], self.center_dot["thickness"])
            cv2.circle(frame, (self.image_center[0], self.image_center[1]), self.face_dot["radius"], self.face_dot["color"], self.face_dot["thickness"])
            cv2.line(frame, (fx, fy), (ex + fx, ey + fy), self.connection_line["color"], self.connection_line["thickness"])
            # self.get_logger().info(f"{offset.x}, {offset.y}")



    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()

    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
