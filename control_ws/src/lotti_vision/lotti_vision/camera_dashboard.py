import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import time
import os
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - depends on optional local install
    YOLO = None

class CameraDashboard(Node):
    def __init__(self):
        super().__init__('camera_dashboard')
        self.bridge = CvBridge()

        # State Variables
        self.frames = {'camera_1': None, 'camera_2': None, 'camera_3': None}

        self.detections = {'camera_1': "", 'camera_2': "", 'camera_3': ""}
        self.ai_cam3_frame = None
        self.yolo_model = None

        self.lock = threading.Lock()

        self._configure_optional_ai()

        # Subscribe to the cameras
        self.create_subscription(CompressedImage, '/camera_1/image_raw/compressed',
                                 lambda msg: self.image_callback(msg, 'camera_1'), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera_2/image_raw/compressed',
                                 lambda msg: self.image_callback(msg, 'camera_2'), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera_3/image_raw/compressed',
                                 lambda msg: self.image_callback(msg, 'camera_3'), qos_profile_sensor_data)

        if self.yolo_model is None:
            self.get_logger().info("Dashboard started in video-only mode. Camera 3 AI overlay is disabled.")
        else:
            self.get_logger().info("Dashboard started. AI scanner active only on Camera 3.")
            self.ai_thread = threading.Thread(target=self.ai_processing_worker, daemon=True)
            self.ai_thread.start()

    def _configure_optional_ai(self):
        """Load the optional YOLO model if the dependency and model file are available."""
        if YOLO is None:
            self.detections['camera_3'] = "YOLO unavailable: ultralytics not installed"
            self.get_logger().warning(
                "Optional dependency 'ultralytics' is not installed. "
                "Camera 3 will run without hazard detection."
            )
            return

        package_share_directory = get_package_share_directory('lotti_vision')
        model_path = os.path.join(package_share_directory, 'models', 'best.onnx')

        if not os.path.exists(model_path):
            self.detections['camera_3'] = "YOLO unavailable: model file missing"
            self.get_logger().warning(
                f"Optional YOLO model was not found at {model_path}. "
                "Camera 3 will run without hazard detection."
            )
            return

        self.get_logger().info("Locating YOLO ONNX model for Camera 3...")

        try:
            self.yolo_model = YOLO(model_path, task='detect')
            self.detections['camera_3'] = "Scanning for hazards..."
        except Exception as exc:  # pragma: no cover - runtime model/backend dependent
            self.yolo_model = None
            self.detections['camera_3'] = "YOLO unavailable: failed to load model"
            self.get_logger().warning(
                f"Failed to initialize optional YOLO detector: {exc}. "
                "Camera 3 will run without hazard detection."
            )

    def image_callback(self, msg, cam_name):
        """ Instantly decodes the compressed Wi-Fi packet and saves it. """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.frames[cam_name] = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to decode {cam_name}: {e}")

    def ai_processing_worker(self):
        """
        BACKGROUND THREAD: Only processes Camera 3 to save massive CPU power.
        """
        while rclpy.ok():
            time.sleep(0.1) # Throttle to 10 FPS

            if self.yolo_model is None:
                continue
            
            cam3_frame = None
            with self.lock:
                if self.frames['camera_3'] is not None:
                    cam3_frame = self.frames['camera_3'].copy()

            if cam3_frame is not None:
                # Run YOLO detection on the current frame from Camera 3
                results = self.yolo_model.predict(source=cam3_frame, conf=0.6, verbose=False)
                annotated_frame = results[0].plot() 
                detected_count = len(results[0].boxes)

                if detected_count > 0:
                    simulated_result = f"WARNING: {detected_count} Hazard(s) Detected!"
                else:
                    simulated_result = "Scanning for hazards..."
                
                with self.lock:
                    self.detections['camera_3'] = simulated_result
                    self.ai_cam3_frame = annotated_frame


def main(args=None):
    rclpy.init(args=args)
    dashboard_node = CameraDashboard()

    ros_thread = threading.Thread(target=rclpy.spin, args=(dashboard_node,), daemon=True)
    ros_thread.start()

    try:
        while rclpy.ok():
            with dashboard_node.lock:
                display_frames = {k: v.copy() if v is not None else None for k, v in dashboard_node.frames.items()}
                display_text = dashboard_node.detections.copy()
            if dashboard_node.ai_cam3_frame is not None:
                    display_frames['camera_3'] = dashboard_node.ai_cam3_frame.copy()
            for cam, frame in display_frames.items():
                if frame is not None:
                    # Only draw text if there is actually text to draw (Camera 3)
                    text = display_text[cam]
                    if text: 
                        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.8, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    cv2.imshow(f"{cam.upper()} FEED", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        dashboard_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
