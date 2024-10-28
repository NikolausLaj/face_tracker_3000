import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import cv2
import math

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

        # for Logitech C920
        self.declare_parameter('camera_parameters.focal_length', 3.67)
        self.declare_parameter('camera_parameters.sensor_width', 4.8)
        self.declare_parameter('camera_parameters.sensor_height', 3.6)

        self.declare_parameter('distance.average_eye_distance', 0.0063)

        # Retrieve parameters
        self.camera_input = self.get_parameter('camera_input').get_parameter_value().integer_value
        self.scale_factor = self.get_parameter('face_detection.scale_factor').get_parameter_value().double_value
        self.min_neighbors = self.get_parameter('face_detection.min_neighbors').get_parameter_value().integer_value
        self.AVR_EYE_DIST = self.get_parameter('distance.average_eye_distance').get_parameter_value().double_value

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

        self.camera_parameters = {
            "focal_length": self.get_parameter('camera_parameters.focal_length').get_parameter_value().double_value,
            "sensor_width": self.get_parameter('camera_parameters.sensor_width').get_parameter_value().double_value,
            "sensor_height": self.get_parameter('camera_parameters.sensor_height').get_parameter_value().double_value
        }

        # Log retrieved parameters
        self.get_logger().info(f"Camera input: {self.camera_input}")
        self.get_logger().info(f"Face detection scale factor: {self.scale_factor}")
        self.get_logger().info(f"Face detection min neighbors: {self.min_neighbors}")
        self.get_logger().info(f"Center dot settings: {self.center_dot}")
        self.get_logger().info(f"Face dot settings: {self.face_dot}")
        self.get_logger().info(f"Connection line settings: {self.connection_line}")
        self.get_logger().info(f"Average eye distance: {self.AVR_EYE_DIST}")
        self.get_logger().info(f"Camera parameters: {self.camera_parameters}")

        self.eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.cap = cv2.VideoCapture(self.camera_input)

        self.SENSOR_WIDTH_PIXEL = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.SENSOR_HEIGTH_PIXEL = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.image_center = (
            int( self.SENSOR_WIDTH_PIXEL // 2 ),
            int( self.SENSOR_HEIGTH_PIXEL // 2 )
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
                self.frameCallback(frame)
            else:
                self.get_logger().error("Failed to grab frame from webcam")

            # just for visualization
            cv2.imshow('WebCam', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def D2C(self, dist_eye_m, dist_eye_pixel, f_mm, sensor_width_mm, sensor_width_pixel):
        f_pixel = (f_mm * sensor_width_pixel) / sensor_width_mm
        dist_2_camera = (dist_eye_m * f_pixel) / dist_eye_pixel
        return dist_2_camera

    def frameCallback(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray_frame, self.scale_factor, self.min_neighbors)

        for (x, y, w, h) in faces:
            # Compute face center
            fx = (2 * x + w) // 2
            fy = (2 * y + h) // 2

            ex = self.image_center[0] - fx
            ey = self.image_center[1] - fy

            offset = Point(x=float(ex), y=float(ey), z=0.0)
            self.coord_pub.publish(offset)

            # Compute eye distance
            roi_gray = gray_frame[y:y + h, x:x + w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)

            if len(eyes) >= 2:
                eye1, eye2 = eyes[:2]

                # Calculate interocular distance in pixels in the reference frame
                eye_distance_pixels = int(math.sqrt((eye1[0] - eye2[0]) ** 2 + (eye1[1] - eye2[1]) ** 2))
                
                dist_2_camera = self.D2C(
                        self.AVR_EYE_DIST, 
                        eye_distance_pixels,
                        self.camera_parameters["focal_length"],
                        self.camera_parameters["sensor_width"],
                        self.SENSOR_WIDTH_PIXEL
                    )

                cv2.putText(frame, f"Distance: {dist_2_camera:2f} m",
                                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
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
