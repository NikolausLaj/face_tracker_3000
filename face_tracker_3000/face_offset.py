import rclpy
from rclpy.node import Node
import cv2

class FaceTracker(Node):
    def __init__(self):
        super().__init__('webcam_node')

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.cap = cv2.VideoCapture(0)

        self.image_center = (
            int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2),
            int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // 2)
            )

        if not self.cap.isOpened():
            self.get_logger().error("Unable to access the webcam")
        else:
            self.get_logger().info("Webcam successfully opened")

    def spin(self):
        while rclpy.ok():
            ret, frame = self.cap.read()  # Capture a frame from the webcam
            if ret:
                self.frame_callback(frame)  # Call the callback with the frame
            else:
                self.get_logger().error("Failed to grab frame from webcam")
            
            # Show the frame for testing purposes
            cv2.imshow('Webcam', frame)

            # Check for 'q' key press to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def frame_callback(self, frame):
        # convert to gray
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect face
        faces = self.face_cascade.detectMultiScale(gray_frame, 1.1, 7)

        # compute box around face
        corners = []
        for (x, y, w, h) in faces:
            corner_1 = (x, y)
            corner_2 = (x + w, y + h)
            corners.append([corner_1, corner_2])
            cv2.rectangle(frame, corner_1, corner_2, (0,0,255), 3)
        
        # midpoint
        midpoints = []
        for rect in corners:
            x = (rect[0][0] + rect[1][0]) // 2
            y = (rect[0][1] + rect[1][1]) // 2
            midpoints.append((x, y))
            cv2.circle(frame, (x, y), 3, (0,255,0), -1)  # Red dot for face center

        # error
        errors = []
        for face_center in midpoints:
            ex = self.image_center[0] - face_center[0]
            ey = self.image_center[1] - face_center[1]
            errors.append((ex, ey))
        
            # print((ex, ey), image_center, face_center)
            cv2.line(frame, face_center, (ex + face_center[0], ey + face_center[1]), (255, 255, 255), 2)

        # Process the frame (you can add any processing logic here)
        self.get_logger().info(f"Processing frame of size: {frame.shape}")
        # Example: You can publish or process the frame here

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()

    try:
        node.spin()  # Continuously check for webcam frames
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()  # Clean up resources when shutting down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
