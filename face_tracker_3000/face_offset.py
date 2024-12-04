import cv2
import math
import rclpy
# import struct
import numpy as np

from rclpy.node import Node

from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

# TODO: Change to this camera node https://index.ros.org/p/camera_ros/#humble-overview

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_offset_node')

        self.joint_state_ = JointState()
        self.joint_state_.name = ['x_angle', 'y_angle']

        # Config file parameters
        self.declare_parameter('face_detection.scale_factor', 1.1)
        self.declare_parameter('face_detection.min_neighbors', 5)
        self.declare_parameter('distance.average_eye_distance', 0.0063)

        self.scale_factor = self.get_parameter('face_detection.scale_factor').get_parameter_value().double_value
        self.min_neighbors = self.get_parameter('face_detection.min_neighbors').get_parameter_value().integer_value
        self.AVR_EYE_DIST = self.get_parameter('distance.average_eye_distance').get_parameter_value().double_value

        # Subscriber
        self.image_sub = self.create_subscription( Image, 'image_raw', self.frameCallback, 10 )

        # Publischer
        # self.offset_pub = self.create_publisher( Point, 'face_offset', 10 )
        self.dist_2_face_pub = self.create_publisher( Float64, 'dist_2_face', 10 )
        self.angle_2_face_pub = self.create_publisher( JointState, 'angle_2_face', 10 )

        # CvBridge
        self.bridge = CvBridge()

        # Cascades for face and eye tracking
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

        # TODO: Replace camera matrix and dist coeffs with subsriber values of camera_node
        self.camera_mat = np.array([
                            [603.3565980559212, 0.0, 314.98325947311577],
                            [0.0, 603.8525136324047, 233.97939039940036],
                            [ 0.0, 0.0,  1.0 ]], dtype=np.float64)
        
        self.dist_coeffs = np.array([0.07904876200503386, -0.2473724352661824, 1.4961220610951097e-05, 0.0024014898368810893, 0.2125031132173052])

        self.SENSOR_WIDTH_PIXEL = 640
        self.SENSOR_HEIGTH_PIXEL = 480

        self.image_center = (
            int( self.SENSOR_WIDTH_PIXEL // 2 ),
            int( self.SENSOR_HEIGTH_PIXEL // 2 )
            )

        self.get_logger().info("FaceTracker node has been started!")


    def frameCallback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            undistorted_gray_frame = cv2.undistort(gray_frame, self.camera_mat, self.dist_coeffs)

            self.detectFace(undistorted_gray_frame)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


    def D2C(self, dist_eye_m, dist_eye_pixel, f_pixel):
        dist_2_camera = (dist_eye_m * f_pixel) / dist_eye_pixel
        return dist_2_camera


    def faceError2Angle(self, offset, servo_max, max_pixel): # TODO: Should it be changed to radiant?
        d = servo_max // 2
        k = d / ( max_pixel // 2 )
        y = k * offset + d
        return y

    def detectFace(self, frame):
        faces = self.face_cascade.detectMultiScale(frame, self.scale_factor, self.min_neighbors)

        for (x, y, w, h) in faces:
            # Compute face center
            fx = (2 * x + w) // 2
            fy = (2 * y + h) // 2

            ex = self.image_center[0] - fx
            ey = self.image_center[1] - fy

            # Detect eyes
            roi_gray = frame[y:y + h, x:x + w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)

            # Calculate interocular distance in pixels in the reference frame
            if len(eyes) >= 2:
                eye1, eye2 = eyes[:2]
                eye_distance_pixels = int(math.sqrt((eye1[0] - eye2[0]) ** 2 + (eye1[1] - eye2[1]) ** 2))
                dist_2_camera = self.D2C(self.AVR_EYE_DIST, eye_distance_pixels, self.camera_mat[0, 0])
                self.dist_2_face_pub.publish(Float64(data=float(dist_2_camera)))

            angle_x = self.faceError2Angle(ex, 180, self.SENSOR_WIDTH_PIXEL)
            angle_y = self.faceError2Angle(ey, 180, self.SENSOR_HEIGTH_PIXEL)
            self.joint_state_.position = [angle_x, angle_y]
            self.angle_2_face_pub.publish(self.joint_state_) 


def main(args=None):
    rclpy.init(args=args)

    node = FaceTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up when shutting down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
