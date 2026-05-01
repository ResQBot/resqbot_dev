import subprocess
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import struct

class FFmpegCameraNode(Node):
    def __init__(self):
        super().__init__('ffmpeg_camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        device  = self.get_parameter('device').value
        width   = self.get_parameter('width').value
        height  = self.get_parameter('height').value
        fps     = self.get_parameter('fps').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', qos)
        self.width = width
        self.height = height

        # Pipe MJPEG frames directly — much smaller than raw BGR
        ffmpeg_cmd = [
            'ffmpeg',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-probesize', '32',
            '-analyzeduration', '0',
            '-thread_queue_size', '512',
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-framerate', str(fps),
            '-video_size', f'{width}x{height}',
            '-i', device,
            '-err_detect', 'ignore_err',
            '-vcodec', 'copy',      # no decode — pass MJPEG straight through
            '-f', 'mjpeg',          # output as MJPEG stream
            'pipe:1'
        ]

        self.process = subprocess.Popen(
            ffmpeg_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0
        )

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Streaming {device} @ {width}x{height} {fps}fps (MJPEG pipe)')

    def _read_mjpeg_frame(self):
        """Read one complete MJPEG frame by scanning for SOI/EOI markers."""
        buf = bytearray()
        while True:
            byte = self.process.stdout.read(1)
            if not byte:
                return None
            buf.extend(byte)
            # MJPEG frame ends with EOI marker 0xFF 0xD9
            if len(buf) >= 2 and buf[-2] == 0xFF and buf[-1] == 0xD9:
                # Check it also starts with SOI marker 0xFF 0xD8
                soi = buf.find(b'\xff\xd8')
                if soi > 0:
                    buf = buf[soi:]  # trim garbage before SOI
                return bytes(buf)

    def _capture_loop(self):
        self.get_logger().info('Capture loop started')
        while rclpy.ok():
            if self.process.poll() is not None:
                self.get_logger().error('FFmpeg died')
                break

            jpeg_data = self._read_mjpeg_frame()
            if jpeg_data is None:
                continue

            # Decode MJPEG → BGR with OpenCV
            arr = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.process.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FFmpegCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()