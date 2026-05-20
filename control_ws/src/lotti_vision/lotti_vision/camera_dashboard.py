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


        self.camera_config = [
            {'name': 'front_left',  'topic': '/camera_1/image_raw/compressed'},
            {'name': 'front_right', 'topic': '/camera_2/image_raw/compressed'},
            {'name': 'back_left',   'topic': '/camera_3/image_raw/compressed'},
            {'name': 'back_right',  'topic': '/camera_4/image_raw/compressed'},
            {'name': 'arm_camera',  'topic': '/camera_5/image_raw/compressed'}
        ]

        self.ai_camera_name = 'arm_camera'

        # build runtime tracking tracking states from matrix array
        self.frames = {config['name']: None for config in self.camera_config}
        self.detections = {config['name']: "" for config in self.camera_config}
        
        self.ai_processed_frame = None
        self.yolo_model = None
        self.lock = threading.Lock()

        self._configure_optional_ai()

        # generate native ROS 2 subscriptions from config array
        for config in self.camera_config:
            cam_name = config['name']
            topic_str = config['topic']
            
            # Using a default argument inside lambda handles scoping boundaries cleanly
            self.create_subscription(
                CompressedImage,
                topic_str,
                lambda msg, name=cam_name: self.image_callback(msg, name),
                qos_profile_sensor_data
            )
            self.get_logger().info(f"subscribed: '{cam_name}' -> Listening on {topic_str}")

        if self.yolo_model is None:
            self.get_logger().info("Dashboard running in standard mode. Object detection offline.")
        else:
            self.get_logger().info(f"Dashboard running. AI Scanner targeted on: [{self.ai_camera_name}]")
            self.ai_thread = threading.Thread(target=self.ai_processing_worker, daemon=True)
            self.ai_thread.start()

    def _configure_optional_ai(self):
        """Loads target network architectures from index share trees."""
        if YOLO is None:
            self.detections[self.ai_camera_name] = "YOLO unavailable: package missing"
            self.get_logger().warning("Optional dependency 'ultralytics' not found. AI features disabled.")
            return

        package_share_directory = get_package_share_directory('lotti_vision')
        model_path = os.path.join(package_share_directory, 'models', 'best.onnx')

        if not os.path.exists(model_path):
            self.detections[self.ai_camera_name] = "YOLO unavailable: model missing"
            self.get_logger().warning(f"No ONNX weight profile resolved at path target: {model_path}")
            return

        try:
            self.yolo_model = YOLO(model_path, task='detect')
            self.detections[self.ai_camera_name] = "Scanning target workspace..."
        except Exception as exc:
            self.yolo_model = None
            self.detections[self.ai_camera_name] = "YOLO offline: initialization error"
            self.get_logger().error(f"Failed to isolate and instantiate target weight model: {exc}")

    def image_callback(self, msg, cam_name):
        """Processes high-frequency payload packets while dropping old packets to preserve zero-latency bounds."""
        try:
            # Latency gate check
            now = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            latency = (now - msg_time).nanoseconds / 1e9
            
            if latency > 0.1:  # Drop anything delayed by over 100ms
                return

            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.frames[cam_name] = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed packet frame unpack for target [{cam_name}]: {e}")

    def ai_processing_worker(self):
        """Isolated background pipeline handler. Targets the arm camera explicitly."""
        while rclpy.ok():
            time.sleep(0.05)  # Constrain execution overhead to 20 FPS max

            if self.yolo_model is None:
                continue
            
            target_frame = None
            with self.lock:
                if self.frames[self.ai_camera_name] is not None:
                    target_frame = self.frames[self.ai_camera_name].copy()

            if target_frame is not None:
                # Execute edge analytics
                results = self.yolo_model.predict(source=target_frame, conf=0.6, verbose=False)
                annotated_frame = results[0].plot() 
                detected_count = len(results[0].boxes)

                # --- Placeholder area for QR code parsing loops later ---
                # codes = qr_scanner_function(target_frame)
                
                if detected_count > 0:
                    status_text = f"WARNING: {detected_count} Hazard(s) Detected!"
                else:
                    status_text = "Scanning workspace clear..."
                
                with self.lock:
                    self.detections[self.ai_camera_name] = status_text
                    self.ai_processed_frame = annotated_frame


def main(args=None):
    rclpy.init(args=args)
    dashboard_node = CameraDashboard()

    ros_thread = threading.Thread(target=rclpy.spin, args=(dashboard_node,), daemon=True)
    ros_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.016)  # Cap structural cycle draws at 60 Hz

            with dashboard_node.lock:
                # Create point copies of current operational buffers
                display_frames = {k: v.copy() if v is not None else None for k, v in dashboard_node.frames.items()}
                display_text = dashboard_node.detections.copy()
            
            # Splice in the processed frame overlay if available
            if dashboard_node.ai_processed_frame is not None:
                display_frames[dashboard_node.ai_camera_name] = dashboard_node.ai_processed_frame.copy()
            
            # Loop through active setups and paint the UI window frames
            for cam_name, frame in display_frames.items():
                if frame is not None:
                    text = display_text.get(cam_name, "")
                    if text: 
                        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.7, (0, 0, 255) if "WARNING" in text else (0, 255, 0), 2, cv2.LINE_AA)
                    
                    # Renders a cleanly capitalized window header name based on dictionary keys
                    window_title = cam_name.replace('_', ' ').upper() + " FEED"
                    cv2.imshow(window_title, frame)

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