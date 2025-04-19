import rclpy
import time
from rclpy.node import Node
import math
import numpy

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from rclpy import qos
from threading import Lock

# Global Defines --------------------------------------------------------------
#DRIVE_INTERFACE_STS_INIT = 0
#DRIVE_INTERFACE_STS_RUN = 1

xAxes = 0
yAxes = 1

x_button = 2
y_button = 3
a_button = 0
b_button = 1
lb_button = 4
rb_button = 5

#max_motor_speed = 4000.0
#transmission = 111*45/24
#max_speed = float(max_motor_speed / transmission)
PI=3.1415926535897

# Class -----------------------------------------------------------------------
class chainController(Node):

    def __init__(self):
        #Entrypoint of the class
        super().__init__('chain_controller')

        #define variables
        self.__velocity = Float32()
        self.__alpha = Float32()

        self.__cmd_speed_x = float(0)
        self.__cmd_speed_x_lock = Lock()
        self.__cmd_speed_y = float(0)
        self.__cmd_speed_y_lock = Lock()
        self.flipper_mode = int(0)
        self._speed = float(0)

        self.__button_a = int(0)
        self.__button_a_lock = Lock()
        self.__button_b = int(0)
        self.__button_b_lock = Lock()
        self.__button_x = int(0)
        self.__button_x_lock = Lock()
        self.__button_y = int(0)
        self.__button_y_lock = Lock()
        self.__button_lb = int(0)
        self.__button_lb_lock = Lock()
        self.__button_rb = int(0)
        self.__button_rb_lock = Lock()

        self.__flipper_frontLeft_cmd    = Int16()
        self.__flipper_frontRight_cmd   = Int16()
        self.__flipper_rearLeft_cmd     = Int16()         
        self.__flipper_rearRight_cmd    = Int16()


        self.__time_last_joy_msg = 0.0
        self._first_joy_msg_received = Bool()

        self.__joy_enabled = False
        self.__joy_enabled_old = False

        # Status see timer callback for description
#        self.__state = DRIVE_INTERFACE_STS_INIT

        #Init class ->create subscriber, create timer
        self.__readParams()
        self.__createSubscribers()
        self.__createPublishers()
        self.__createTimer()

    def __readParams(self):
        #declare parameters
        self.declare_parameter('Publish_rate', 10)              #[Hz]
        self.declare_parameter('Joy_timeout', 1)                #[sec]  

        #read parameters
        self.__Publish_rate = rclpy.parameter.Parameter(
            'Publish_rate',
            rclpy.Parameter.Type.DOUBLE,
            10.0
        )
        self.__Joy_timeout = 1.0


    def __checkCMDOutputEnable(self):

        # ----- Timout error check -----
        # If the time difference between the last recived flipper msg and now is greater then the timeout, DISABLE the motor rpm output
        self.__time_since_last_joy_msg = 0.0

        if (self.__time_last_joy_msg != 0.0 and self.__Joy_timeout != 0.0):
            # Calculate the time difference
            self.__time_since_last_joy_msg = time.time() - self.__time_last_joy_msg

            # If there was a joy msg before, the flipper output is in ENABLED status but the time since the last joy msg was received
            # is greater then the timout time, DISABLE the flipper output and print a msg for the user that there was a timeout.
            if (self.__joy_enabled == True and self.__time_since_last_joy_msg > self.__Joy_timeout):
                self._joy_enabled = False
                self.get_logger().error("Joy msg timeout")
        else:
            self.__joy_enabled = True

        # Print a message to show the current flipper output status.
        if self.__joy_enabled != self.__joy_enabled_old:
            if (self.__joy_enabled == True and self.__joy_enabled_old == False):
                self.get_logger(), "Chain command: ENABLED"
            else:
                if (self.__joy_enabled == False and self.__joy_enabled_old == True):
                    self.get_logger(), "Chain command: DISABLED"

            # Set the old state to the new state
            self.__joy_enabled_old = self.__joy_enabled


    def __calcAndSendMovement(self):

        x_value = self.__cmd_speed_x
        y_value = self.__cmd_speed_y

        #calculating desired velocity
        self.__speed =  math.sqrt((x_value * x_value) + (y_value * y_value))

        self.__velocity.data = self.__speed
        
        #calculating desired angle
        #avoiding division by 0
        if(y_value == 0):
            self.__angle = (PI / 2)
        else:
            self.__angle = math.atan(x_value/y_value)
        #using degrees to be humanly understandable  
        self.__angle = math.degrees(self.__angle)
        #using absolute to solve direcional problems
        self.__angle_abs = float(abs(self.__angle))

        if(y_value < 0):
            self.__angle_abs = 180 - self.__angle_abs

        if(x_value < 0):
            self.__angle_abs = self.__angle_abs * (-1)

        self.__alpha.data = self.__angle_abs

        self.__velocityPub.publish(self.__velocity)
        self.__angularPub.publish(self.__alpha)

        #set Flipper commands
        if (self.__button_lb == 1):
            self.__flipper_mode = -1
        elif (self.__button_rb == 1):
            self.__flipper_mode = 1
        else:
            self.__flipper_mode = 0

        self.__flipper_frontLeft_cmd.data = int(self.__button_x * self.__flipper_mode)
        self.__flipper_frontRight_cmd.data = int(self.__button_y * self.__flipper_mode)
        self.__flipper_rearLeft_cmd.data = int(self.__button_a * self.__flipper_mode)         
        self.__flipper_rearRight_cmd.data = int(self.__button_b * self.__flipper_mode)

        #send the commands
        self.__pub_flipper_frontLeft.publish(self.__flipper_frontLeft_cmd)
        self.__pub_flipper_frontRight.publish(self.__flipper_frontRight_cmd)
        self.__pub_flipper_rearLeft.publish(self.__flipper_rearLeft_cmd)
        self.__pub_flipper_rearRight.publish(self.__flipper_rearRight_cmd)


        

    def __timerCallback(self):
        self.__checkCMDOutputEnable()
        self.__calcAndSendMovement()



    def __joyCallback(self, msg):
        self.__cmd_speed_x_lock.acquire()
        self.__cmd_speed_x = msg.axes[xAxes]
        self.__cmd_speed_x_lock.release()

        self.__cmd_speed_y_lock.acquire()
        self.__cmd_speed_y = msg.axes[yAxes]
        self.__cmd_speed_y_lock.release()

        self.__button_x_lock.acquire()
        self.__button_x = msg.buttons[x_button]
        self.__button_x_lock.release()

        self.__button_y_lock.acquire()
        self.__button_y = msg.buttons[y_button]
        self.__button_y_lock.release()

        self.__button_a_lock.acquire()
        self.__button_a = msg.buttons[a_button]
        self.__button_a_lock.release()

        self.__button_b_lock.acquire()
        self.__button_b = msg.buttons[b_button]
        self.__button_b_lock.release()

        self.__button_lb_lock.acquire()
        self.__button_lb = msg.buttons[lb_button]
        self.__button_lb_lock.release()
        
        self.__button_rb_lock.acquire()
        self.__button_rb = msg.buttons[rb_button]
        self.__button_rb_lock.release()

        # Check if there was a joy msgs since the node was started
        if(self._first_joy_msg_received == False):
            self._first_joy_msg_received = True
        

    def __createSubscribers(self):
        # Create subscribers

        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.__joyCallback,
            5,
        )

    def __createPublishers(self):
        # Create publishers

        self.__velocityPub = self.create_publisher(
            Float32,
            'movement/velocity',
            1
        )
        
        self.__angularPub = self.create_publisher(
            Float32,
            'movement/angle',
            1
        )

        self.__pub_flipper_frontLeft = self.create_publisher(
            Int16,
            'cmd/flipper/frontLeft',
            1
        )

        self.__pub_flipper_frontRight = self.create_publisher(
            Int16,
            'cmd/flipper/frontRight',
            1
        )

        self.__pub_flipper_rearLeft = self.create_publisher(
            Int16,
            'cmd/flipper/rearLeft',
            1
        )

        self.__pub_flipper_rearRight = self.create_publisher(
            Int16,
            'cmd/flipper/rearRight',
            1
        )        

    def __createTimer(self):
        # Create timer
        self._timer = self.create_timer(
            1.0 / self.__Publish_rate.value,
            self.__timerCallback
        )


def main(args=None):
    rclpy.init(args=args)

    chain_controller = chainController()

    rclpy.spin(chain_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    chain_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
