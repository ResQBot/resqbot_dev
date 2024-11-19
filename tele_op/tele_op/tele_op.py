import rclpy
from rclpy.node import Node
import math
import time

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import String
from rclpy import qos
from threading import Lock

# Global Defines --------------------------------------------------------------
left_stick_x = 0
left_stick_y = 1
right_stick_x = 2
right_stick_y = 3
left_trigger = 5
right_trigger = 4
d_pad_x = 6
d_pad_y = 7

a_button = 0
b_button = 1
x_button = 3
y_button = 4
lb_button = 6
rb_button = 7
menue_button = 10
start_button = 11
xBox_button = 12
left_stick_button = 13
right_stick_button = 14
enter_button = 15

PI=3.1415926535897

# Class -----------------------------------------------------------------------
class TeleOp(Node):

    def __init__(self):
        #Entrypoint of the class
        super().__init__('tele_op')

        #define variables
        self.__velocity_msg = Float32()
        self.__angle_msg = Float32()
        self.__speed = float(0)
        self.__angle = float(0)

        self.__axes_left_stick_x = float(0)
        self.__axes_left_stick_x_lock = Lock()
        self.__axes_left_stick_y = float(0)
        self.__axes_left_stick_y_lock = Lock()

        self.__axes_right_stick_x = float(0)
        self.__axes_right_stick_x_lock = Lock()
        self.__axes_right_stick_y = float(0)
        self.__axes_right_stick_y_lock = Lock()

        self.__button_a = int(0)
        self.__button_a_lock = Lock()
        self.__button_b = int(0)
        self.__button_b_lock = Lock()
        self.__button_x = int(0)
        self.__button_x_lock = Lock()
        self.__button_y = int(0)
        self.__button_y_lock = Lock()
        self.__d_pad_x = int(0)
        self.__d_pad_x_lock = Lock()
        self.__d_pad_y = int(0)
        self.__d_pad_y_lock = Lock()
#        self.__button_rb = int(0)
#        self.__button_rb_lock = Lock()
#        self.__button_lb = int(0)
#        self.__button_lb_lock = Lock()
        self.__button_start = int(0)
        self.__button_start_lock = Lock()

        self.__flipper_frontLeft_cmd    = Int16()
        self.__flipper_frontRight_cmd   = Int16()
        self.__flipper_rearLeft_cmd     = Int16()         
        self.__flipper_rearRight_cmd    = Int16()

        self.__first_joy_msg_received = bool(False)
        self.__joy_enabled = bool(False)
        self.__joy_enabled_old = bool(False)
        self.__button_start_pressed = bool(False)
        self.__button_start_last_pressed = 0.0

        #Init class ->create subscriber, create timer
        self.__readParams()
        self.__createSubscribers()
        self.__createPublishers()
        self.__createTimer()

        print("tele_op initiated")

    def __readParams(self):
        #declare parameters
        self.declare_parameter('Publish_rate', 10)              #[Hz]

        #read parameters
        self.__Publish_rate = rclpy.parameter.Parameter(
            'Publish_rate',
            rclpy.Parameter.Type.DOUBLE,
            10.0
        )


    def __checkCMDOutputEnable(self):

        # ----- Enable button check -----
        # If there was no joy msg received before and the twist output somehow is ENABLED, DISABLE the twist output.
        if (self.__first_joy_msg_received == False and self.__joy_enabled != False):
            self.__joy_enabled = False
        
        elif (self.__first_joy_msg_received == True):
            # If the output is disabled, the output-enable button was not pressed and is now pressed, set the button
            # pressed var to true and save the time point when it was pressed to get the time difference.
            if(self.__joy_enabled == False and self.__button_start_pressed == False and self.__button_start == 1):
                    self.__button_start_pressed = True
                    self.__button_start_last_pressed = time.time()
            
            # check if button is pressed long enough.
            elif (self.__joy_enabled == False and self.__button_start_pressed == True and self.__button_start == 1):
                # messure how long the start button is pressed
                time_difference = time.time() - self.__button_start_last_pressed

                # If the start button is pressed long enough, enable the controlles
                if(time_difference > 1):
                    self.__joy_enabled = True
                        
            # After the output was enabled, wait for the button to be released befor the button_pressed var is set to false.
            elif (self.__joy_enabled == True and self.__button_start_pressed == True and self.__button_start == 0):
                self.__button_start_pressed = False
            
            # if joy is disabled and the start_button was pressed but is released before the output could be 
            # enabled, the button_pressed var is set to false to receive a new button press.
            elif (self.__joy_enabled == False and self.__button_start_pressed == True and self.__button_start == 0):
                self.__button_start_pressed = False

            #if joy is enabled and the enable button is pressed, disable the output.
            elif (self.__joy_enabled == True and self.__button_start_pressed == False and self.__button_start == 1):
                self.__joy_enabled = False


        # Print a message to show the current output status.
        if self.__joy_enabled != self.__joy_enabled_old:
            if (self.__joy_enabled == True and self.__joy_enabled_old == False):
                print("Tele Op: ENABLED")
            else:
                if (self.__joy_enabled == False and self.__joy_enabled_old == True):
                    print("Tele Op: DISABLED")

            # Set the old state to the new state
            self.__joy_enabled_old = self.__joy_enabled


    def __calcAndSendFlipper(self):

        self.__flipper_frontLeft_cmd.data = int(self.__button_x * self.__d_pad_y)
        self.__flipper_frontRight_cmd.data = int(self.__button_y * self.__d_pad_y)
        self.__flipper_rearLeft_cmd.data = int(self.__button_a * self.__d_pad_y)         
        self.__flipper_rearRight_cmd.data = int(self.__button_b * self.__d_pad_y)

        #send the commands
        if (self.__joy_enabled == True):
            self.__pub_flipper_frontLeft.publish(self.__flipper_frontLeft_cmd)
            self.__pub_flipper_frontRight.publish(self.__flipper_frontRight_cmd)
            self.__pub_flipper_rearLeft.publish(self.__flipper_rearLeft_cmd)
            self.__pub_flipper_rearRight.publish(self.__flipper_rearRight_cmd)

    def __calcAndSendChains(self):

        y_value = self.__axes_left_stick_x
        x_value = self.__axes_left_stick_y
        #reverse_left = bool(0)
        #reverse_right = bool(0)
        #calculating desired velocity
        self.__speed =  math.sqrt((x_value * x_value) + (y_value * y_value))
        if(x_value == 0):
            self.__angle = PI/2
        else:
            self.__angle = math.atan(y_value/x_value)
        self.__angle = math.degrees(self.__angle)
        self.__angle_abs = float(abs(self.__angle))

        if(x_value < 0):
            self.__angle_abs = 180 - self.__angle_abs    
        
        if(y_value < 0):
            self.__angle_abs = self.__angle_abs * (-1)
        
        self.__velocity_msg.data = self.__speed
        self.__angle_msg.data = self.__angle_abs

        #send movement commands
        if (self.__joy_enabled == True):
            self.__velocityPub.publish(self.__velocity_msg)
            self.__anglePub.publish(self.__angle_msg)
        

    def calc_and_send_arm(self):
        turn_value = self.__axes_right_stick_x
        up_value = self.__d_pad_x
        look_up_value = self.__axes_right_stick_y

        messages = []

        if up_value < -0.5:
            messages.append("DOWN")
        elif up_value > 0.5:
            messages.append("UP")
        else:
            messages.append("STOP")
        
        if turn_value < -0.5:
            messages.append("RIGHT")
        elif turn_value > 0.5:
            messages.append("LEFT")
        else:
            messages.append("STOP")

        if look_up_value < -0.5:
            messages.append("LDOWN")
        elif look_up_value > 0.5:
            messages.append("LUP")
        else:
            messages.append("STOP")
        
        message = ",".join(messages)
        if message:
            msg = String()
            msg.data = message
            self.__arm_publisher.publish(msg)
    

    def __timerCallback(self):
        self.__checkCMDOutputEnable()
        self.__calcAndSendFlipper()
        self.__calcAndSendChains()
        self.calc_and_send_arm()



    def __joyCallback(self, msg):
        self.__axes_left_stick_x_lock.acquire()
        self.__axes_left_stick_x = msg.axes[left_stick_x]
        self.__axes_left_stick_x_lock.release()

        self.__axes_left_stick_y_lock.acquire()
        self.__axes_left_stick_y = msg.axes[left_stick_y]
        self.__axes_left_stick_y_lock.release()


        self.__axes_right_stick_x_lock.acquire()
        self.__axes_right_stick_x = msg.axes[right_stick_x]
        self.__axes_right_stick_x_lock.release()

        self.__axes_right_stick_y_lock.acquire()
        self.__axes_right_stick_y = msg.axes[right_stick_y]
        self.__axes_right_stick_y_lock.release()

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

        self.__d_pad_y_lock.acquire()
        self.__d_pad_y = msg.axes[d_pad_y]
        self.__d_pad_y_lock.release()

        self.__d_pad_x_lock.acquire()
        self.__d_pad_x = msg.axes[d_pad_x]
        self.__d_pad_x_lock.release()

        self.__button_start_lock.acquire()
        self.__button_start = msg.buttons[start_button]
        self.__button_start_lock.release()

        # Check if there was a joy msgs since the node was started
        if(self.__first_joy_msg_received == False):
            self.__first_joy_msg_received = True
        

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
        
        self.__anglePub= self.create_publisher(
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

        self.__arm_publisher = self.create_publisher(
            String,
            "movement/arm",
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

    tele_op = TeleOp()

    rclpy.spin(tele_op)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tele_op.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
