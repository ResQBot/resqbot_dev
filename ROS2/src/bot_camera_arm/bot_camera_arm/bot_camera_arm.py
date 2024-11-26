import rclpy
import serial
import time

from rclpy.node import Node

from std_msgs.msg import Float32, String
from rclpy import qos
from threading import Lock

# Global Defines --------------------------------------------------------------
ARM_INTERFACE_STS_INIT = 0
ARM_INTERFACE_STS_RUN = 1

# Class -----------------------------------------------------------------------
class camera_arm(Node):
    # This class represents the drive interface node.
    # It is a bridge between the ros network and the arduino drive controller

    def __init__(self):
        # Entrypoint of the class (first called)
        super().__init__('arm_interface')

        # Create private variables

        # Status see timer callback for description
        self.__state = ARM_INTERFACE_STS_INIT
        self.__cmd_arm_movements = String()
        self.__cmd_arm_movements_lock = Lock()


        # Init class -> read parameters, create subscribers, create timer (for update loop)
        self.__readParams()
        self.__createSubscribers()
        self.__createTimer()

    def __armCallback(self, msg):

        self.__cmd_arm_movements_lock.acquire()
        self.__cmd_arm_movements = msg.data + "\n"
        self.__cmd_arm_movements_lock.release()


    def handshake(self):
        # Statemachine to handle connect and reconnect to arduino
        # if it is unplugged while node is running
        # Try to connect to arduino, otherwise stay in init state
        # Open serial connection with every port and look for correct Arduino
        handshake = False
        print("handshake")
        for portNo in range (0, 10): 
            comPort = format("/dev/ttyACM{}".format(int(portNo)))
            try:
                self.__armInterface = serial.Serial(
                    port = comPort, 
                    baudrate = self._serial_baudrate.value, 
                    timeout = self._serial_timeout.value)
                # Flush old data from buffers
                #self.__armInterface.flush()
                tx_msg = format("Who are you?\n")
                self.__armInterface.write(tx_msg.encode('utf-8'))
                print(comPort)
                time.sleep(0.5)
                rx_msg = self.__armInterface.readline().strip().decode("utf-8")
                print(rx_msg)

                if rx_msg == 'ArmController':
                    tx_msg = format("Hello ArmController\n")
                    self.__armInterface.write(tx_msg.encode('utf-8'))
                    time.sleep(0.5)

                    rx_msg = self.__armInterface.readline().strip().decode("utf-8")
                    print(rx_msg)
                    while handshake == False:
                        rx_msg = self.__armInterface.readline().strip().decode("utf-8")
                        print(rx_msg)
                        if rx_msg == 'confirmed':
                            # Log success
                            
                            self.get_logger().info('Connected to ArmController at ' + comPort)
                            #transition to run state
                            self.__state = ARM_INTERFACE_STS_RUN
                            handshake = True
                            return
                        else:
                            time.sleep(0.5)
                    break 
                else:
                    self.__armInterface.close()
                    continue
            except:
                # Log error
                self.get_logger().error('Could not connect to armInterface')

    
    def __timerCallback(self):

        if self.__state == ARM_INTERFACE_STS_INIT:
            self.handshake()

        # Run state
        # Transmit data to arduino uno
        elif self.__state == ARM_INTERFACE_STS_RUN:
            
            try:
                self.__armInterface.write(self.__cmd_arm_movements.encode('utf-8'))
                print(self.__cmd_arm_movements.encode('utf-8'))
                
            except:
                # Transition to init state
                self.__state = ARM_INTERFACE_STS_INIT
                self.__armInterface.close()

                # Log error
                self.get_logger().error('Could not send data to arduino uno -> Transition to init state')

    def __readParams(self):
        # Declare parameters
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('serial_timeout_sec', 0.1)
        self.declare_parameter('serial_baudrate', 9600)
        self.declare_parameter('Joy_timeout', 1)

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
            '/dev/ttyACM0'
        )

        self._serial_baudrate = rclpy.parameter.Parameter(
            'serial_baudrate',
            rclpy.Parameter.Type.INTEGER,
            115200
        )
        
        self.__Joy_timeout = 1.0
        
        # Check for valid settings
        if (1.0 / self._update_rate_hz.value) < self._serial_timeout.value:
            self.get_logger().error('Serial timeout cannot be greater than the update rate of the task!')
            raise Exception('Serial timeout cannot be greater than the update rate of the task!')

    def __createSubscribers(self):
        # Create subscribers

        self._sub_arm = self.create_subscription(
            String,
            'movement/arm',
            self.__armCallback,
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

    arm_interface = camera_arm()

    rclpy.spin(arm_interface)

    arm_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
