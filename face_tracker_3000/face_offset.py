import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import cv2

# TODO: Add keyword argument, which alows to set BOOL value to display video stream.

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_offset_node')

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.cap = cv2.VideoCapture(2)

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

        faces = self.face_cascade.detectMultiScale(gray_frame, 1.1, 7)

        for (x, y, w, h) in faces:
            fx = (2 * x + w) // 2
            fy = (2 * y + h) // 2

            ex = self.image_center[0] - fx
            ey = self.image_center[1] - fy

            offset = Point(x=float(ex), y=float(ey), z=0.0)

            # just for visualization
            cv2.circle(frame, (fx, fy), 3, (0,255,0), -1)
            cv2.circle(frame, (self.image_center[0], self.image_center[1]), 3, (0,0,254), -1)
            cv2.line(frame, (fx, fy), (ex + fx, ey + fy), (255, 255, 255), 2)

            self.coord_pub.publish(offset)
            self.get_logger().info(f"{offset.x}, {offset.y}")


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
