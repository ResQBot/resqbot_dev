import subprocess
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

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
        
        # FIX: Match the CompressedImage topic type of your subscriber
        self.publisher_ = self.create_publisher(CompressedImage, '/camera_1/image_raw/compressed', qos)

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
            '-vcodec', 'copy',      
            '-f', 'mjpeg',          
            'pipe:1'
        ]

        self.process = subprocess.Popen(
            ffmpeg_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=65536 # Increase internal pipe buffer allocation
        )

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Streaming {device} natively as CompressedImage @ {fps}fps')

    def _capture_loop(self):
        # Read chunks rather than individual bytes to stop thread starvation
        buffer = bytearray()
        chunk_size = 4096 

        while rclpy.ok():
            if self.process.poll() is not None:
                self.get_logger().error('FFmpeg process terminated unexpectedly.')
                break

            chunk = self.process.stdout.read(chunk_size)
            if not chunk:
                continue
            
            buffer.extend(chunk)

            # Find JPEG Start and End of Image Markers within the buffer stream
            while True:
                soi = buffer.find(b'\xff\xd8')
                eoi = buffer.find(b'\xff\xd9')

                if soi != -1 and eoi != -1 and eoi > soi:
                    # Extract the complete JPEG image frame block
                    jpeg_data = buffer[soi:eoi+2]
                    # Retain remaining buffer contents for next validation sequence
                    buffer = buffer[eoi+2:]

                    # Directly package into ROS message without wasting cycles decoding to BGR
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera'
                    msg.format = 'jpeg'
                    msg.data = bytes(jpeg_data)
                    self.publisher_.publish(msg)
                else:
                    # Clean up orphaned buffer chunks if no markers are visible
                    if soi == -1 and len(buffer) > chunk_size * 2:
                        buffer.clear()
                    break

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