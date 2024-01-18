import rclpy
import serial

from rclpy.node import Node

from std_msgs.msg import Int16
from rclpy import qos
from threading import Lock

# Global Defines --------------------------------------------------------------
FLIPPER_INTERFACE_STS_INIT = 0
FLIPPER_INTERFACE_STS_RUN = 1

# Class -----------------------------------------------------------------------
class ResqFlipperInterface(Node):
    # This class represents the drive interface node.
    # It is a bridge between the ros network and the arduino drive controller

    def __init__(self):
        # Entrypoint of the class (first called)
        super().__init__('flipper_interface')

        # Create private variables
        #

        # Status see timer callback for description
        self.__state = FLIPPER_INTERFACE_STS_INIT
        self.__cmd_speed_frontLeft = 0.0
        self.__cmd_speed_frontLeft_lock = Lock()
        self.__cmd_speed_frontRight = 0.0
        self.__cmd_speed_frontRight_lock = Lock()
        self.__cmd_speed_rearLeft = 0.0
        self.__cmd_speed_rearLeft_lock = Lock()
        self.__cmd_speed_rearRight = 0.0
        self.__cmd_speed_rearRight_lock = Lock()
        self.__flushCountdown = 0


        # Init class -> read parameters, create subscribers, create timer (for update loop)
        self.__readParams()
        self.__createSubscribers()
        self.__createTimer()

    def __flipper_frontLeft_callback(self, msg):
        # Called when a message is received on the left wheel topic
        #self.get_logger().info('front left Flipper mode: %f' % msg.data)

        self.__cmd_speed_frontLeft_lock.acquire()
        self.__cmd_speed_frontLeft = msg.data
        self.__cmd_speed_frontLeft_lock.release()

    def __flipper_frontRight_callback(self, msg):
        # Called when a message is received on the left wheel topic
        #self.get_logger().info('front right Flipper mode: %f' % msg.data)

        self.__cmd_speed_frontRight_lock.acquire()
        self.__cmd_speed_frontRight = msg.data
        self.__cmd_speed_frontRight_lock.release()
    
    def __flipper_rearLeft_callback(self, msg):
        # Called when a message is received on the left wheel topic
        #self.get_logger().info('rear left Flipper mode: %f' % msg.data)

        self.__cmd_speed_rearLeft_lock.acquire()
        self.__cmd_speed_rearLeft = msg.data
        self.__cmd_speed_rearLeft_lock.release()

    def __flipper_rearRight_callback(self, msg):
        # Called when a message is received on the left wheel topic
        #self.get_logger().info('rear right Flipper mode: %f' % msg.data)

        self.__cmd_speed_rearRight_lock.acquire()
        self.__cmd_speed_rearRight = msg.data
        self.__cmd_speed_rearRight_lock.release()
    
    def __timerCallback(self):
        # Called when the timer is triggered with the update rate specified in the parameters
        #self.get_logger().info('Timer callback')

        # Statemachine to handle connect and reconnect to arduino uno
        # if it is unplugged while node is running

        # Init state
        # Try to connect to arduino uno, otherwise stay in init state
        if self.__state == FLIPPER_INTERFACE_STS_INIT:
            try:
                # Open serial connection
                self.__serial = serial.Serial(self._serial_name.value, self._serial_baudrate.value, timeout=self._serial_timeout.value)
                
                # Flush old data from buffers
                self.__serial.flush()

                # Transition to run state
                self.__state = FLIPPER_INTERFACE_STS_RUN

                # Log success
                self.get_logger().info('Connected to arduino uno at %s' % self._serial_name.value)
            except:
                # Stay in init state
                self.__state = FLIPPER_INTERFACE_STS_INIT

                # Log error
                self.get_logger().error('Could not connect to arduino uno at %s' % self._serial_name.value)
                

        # Run state
        # Transmit data to arduino uno
        elif self.__state == FLIPPER_INTERFACE_STS_RUN:

            # Get speeds thread safe
            self.__cmd_speed_frontLeft_lock.acquire()
            cmd_speed_frontLeft = self.__cmd_speed_frontLeft
            self.__cmd_speed_frontLeft_lock.release()

            self.__cmd_speed_frontRight_lock.acquire()
            cmd_speed_frontRight = self.__cmd_speed_frontRight
            self.__cmd_speed_frontRight_lock.release()
            
            self.__cmd_speed_rearLeft_lock.acquire()
            cmd_speed_rearLeft = self.__cmd_speed_rearLeft
            self.__cmd_speed_rearLeft_lock.release()

            self.__cmd_speed_rearRight_lock.acquire()
            cmd_speed_rearRight = self.__cmd_speed_rearRight
            self.__cmd_speed_rearRight_lock.release()

            # Create tx message
            tx_msg = format("FL{}FR{}RL{}RR{}\n".format(int(cmd_speed_frontLeft), int(cmd_speed_frontRight), int(cmd_speed_rearLeft), int(cmd_speed_rearRight)));

            # Log message / uncomment for debugging
            self.get_logger().info('TX: %s' % tx_msg)

            # Send message
            try:
                self.__serial.write(tx_msg.encode('utf-8'))

            except:
                # Transition to init state
                self.__state = FLIPPER_INTERFACE_STS_INIT
                self.__serial.close()

                # Log error
                self.get_logger().error('Could not send data to arduino uno -> Transition to init state')


            # Read message
#            try:
#               self.__serial.reset_input_buffer()
#               rx_msg = self.__serial.readline().decode('utf-8').rstrip()
#
#               # Log message / uncomment for debugging
#               #self.get_logger().info('RX: %s' % rx_msg)
#
#            except:
#              # Transition to init state
#                self.__state = FLIPPER_INTERFACE_STS_INIT
#                self.__serial.close()
#
#               # Log error
#                self.get_logger().error('Could not read data from arduino uno -> Transition to init state')


    def __readParams(self):
        # Declare parameters
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('serial_timeout_sec', 0.1)
        self.declare_parameter('serial_name', '/dev/ttyACM1')
        self.declare_parameter('serial_baudrate', 115200)

        # Read parameters

        self._update_rate_hz = rclpy.parameter.Parameter(
            'update_rate_hz',
            rclpy.Parameter.Type.DOUBLE,
            10.0
        )

        self._serial_timeout = rclpy.parameter.Parameter(
            'serial_timeout_sec',
            rclpy.Parameter.Type.DOUBLE,
            0.1
        )

        self._serial_name = rclpy.parameter.Parameter(
            'serial_name',
            rclpy.Parameter.Type.STRING,
            '/dev/ttyACM1'
        )

        self._serial_baudrate = rclpy.parameter.Parameter(
            'serial_baudrate',
            rclpy.Parameter.Type.INTEGER,
            115200
        )
        
        # Check for valid settings
        if (1.0 / self._update_rate_hz.value) < self._serial_timeout.value:
            self.get_logger().error('Serial timeout cannot be greater than the update rate of the task!')
            raise Exception('Serial timeout cannot be greater than the update rate of the task!')

    def __createSubscribers(self):
        # Create subscribers

        self._sub_flipper_frontLeft = self.create_subscription(
            Int16,
            'cmd/flipper/frontLeft',
            self.__flipper_frontLeft_callback,
            1,
        )

        self._sub_flipper_frontRight = self.create_subscription(
            Int16,
            'cmd/flipper/frontRight',
            self.__flipper_frontRight_callback,
            1,
        )

        self._sub_flipper_rearLeft = self.create_subscription(
            Int16,
            'cmd/flipper/rearLeft',
            self.__flipper_rearLeft_callback,
            1,
        )

        self._sub_flipper_rearRight = self.create_subscription(
            Int16,
            'cmd/flipper/rearRight',
            self.__flipper_rearRight_callback,
            1,
        )

    def __createTimer(self):
        # Create timer

        self._timer = self.create_timer(
            1.0 / self._update_rate_hz.value,
            self.__timerCallback
        )

# Main ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    flipper_interface = ResqFlipperInterface()

    rclpy.spin(flipper_interface)

    flipper_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
