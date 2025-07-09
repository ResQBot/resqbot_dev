import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Int8
from threading import Lock
from std_srvs.srv import Trigger


# Global Defines --------------------------------------------------------------
# mapping controller buttons and sticks to joy topc axes and buttons

#axes
left_stick_x = 0
left_stick_y = 1
right_stick_x = 3
right_stick_y = 4
left_trigger = 2
right_trigger = 5
d_pad_x = 6
d_pad_y = 7
#buttons
a_button = 0
b_button = 1
x_button = 2
y_button = 3
lb_button = 4
rb_button = 5
opt_left_button = 6
opt_right_button = 7
xBox_button = 8
left_stick_button = 9
right_stick_button = 10
space_button = 11

PI=3.1415926535897

# Class -----------------------------------------------------------------------
class TeleOp(Node):
    
    def __init__(self):
        #Entrypoint of the class
        super().__init__('tele_op')

        #define variables

        #axes
        self.__left_stick_x = float(0)
        self.__left_stick_x_lock = Lock()
        self.__left_stick_y = float(0)
        self.__left_stick_y_lock = Lock()
        
        self.__left_trigger = float(0)
        self.__left_trigger_lock = Lock()

        self.__right_stick_x = float(0)
        self.__right_stick_x_lock = Lock()
        self.__right_stick_y = float(0)
        self.__right_stick_y_lock = Lock()
        
        self.__right_trigger = float(0)
        self.__right_trigger_lock = Lock()

        self.__d_pad_x = float(0)
        self.__d_pad_x_lock = Lock()
        
        self.__d_pad_y = float(0)
        self.__d_pad_y_lock = Lock()

        #buttons
        self.__button_a = int(0)
        self.__button_a_lock = Lock()
        self.__button_b = int(0)
        self.__button_b_lock = Lock()
        self.__button_x = int(0)
        self.__button_x_lock = Lock()
        self.__button_y = int(0)
        self.__button_y_lock = Lock()

        self.__button_rb = int(0)
        self.__button_rb_lock = Lock()
       
        self.__button_lb = int(0)
        self.__button_lb_lock = Lock()
       
        self.__button_opt_right = int(0)
        self.__button_opt_right_lock = Lock()

        self.__button_opt_left = int(0)
        self.__button_opt_left_lock = Lock()

        self.__button_space = int(0)
        self.__button_space_lock = Lock()


        #safety stuff
        self.__first_joy_msg_received = bool(False)
        self.__joy_enabled = bool(False)
        self.__joy_enabled_old = bool(False)
        self.__button_opt_right_pressed = bool(False)
        self.__button_opt_right_last_pressed = 0.0

        #switch to arm
        self.__arm_enabled = bool(False)
        self.__arm_enabled_old = bool(True)
        self.__button_opt_left_pressed = bool(False)
        self.__button_opt_left_last_pressed = 0.0

        #flipper controlls
        self.__flipper_direction = int
        self.__flipper_cmd_fr = Int8()
        self.__flipper_cmd_fl = Int8()
        self.__flipper_cmd_rr = Int8()
        self.__flipper_cmd_rl = Int8()

        #chain controlls
        self.__chain_msg = Twist()
        self.__chain_msg.linear.y = float(0)
        self.__chain_msg.linear.z = float(0)
        self.__chain_msg.angular.y = float(0)
        self.__chain_msg.angular.x = float(0)

        #arm controlls
        self.__arm_msg = TwistStamped()
        self.__arm_msg.header.frame_id = ""

        #gripper controlls
        self.__gripper_msg = JointJog()
        self.__gripper_msg.joint_names = ["arm_link4_joint", "arm_link5_joint", "arm_link6_joint"]
        self.__gripper_msg.duration = 0.05

        #Init class ->create subscriber, create timer
        self.__readParams()
        self.__callServo()
        self.__createSubscribers()
        self.__createPublishers()
        self.__createTimer()

        self.get_logger().info("Tele_OP initiated")



    def __readParams(self):
        #declare parameters
        self.declare_parameter('Publish_rate', 20)              #[Hz]

        #read parameters
        self.__Publish_rate = rclpy.parameter.Parameter(
            'Publish_rate',
            rclpy.Parameter.Type.DOUBLE,
            20.0
        ) 



    #the servo_node service needs to be called to start
    def __callServo(self):
        self.__cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.__cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.__future = self.__cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, self.__future)
        self.get_logger().info('arm server initiated')
        return self.__future.result()



    def __checkCMDOutputEnable(self):

        # ----- Enable button check -----
        # If there was no joy msg received before and the twist output somehow is ENABLED, DISABLE the twist output.
        if (self.__first_joy_msg_received == False and self.__joy_enabled != False):
            self.__joy_enabled = False
        
        elif (self.__first_joy_msg_received == True):
            # If the output is disabled, the output-enable button was not pressed and is now pressed, set the button
            # pressed var to true and save the time point when it was pressed to get the time difference.
            if(self.__joy_enabled == False and self.__button_opt_right_pressed == False and self.__button_opt_right == 1):
                    self.__button_opt_right_pressed = True
                    self.__button_opt_right_last_pressed = time.time()
            
            # check if button is pressed long enough.
            elif (self.__joy_enabled == False and self.__button_opt_right_pressed == True and self.__button_opt_right == 1):
                # messure how long the start button is pressed
                time_difference = time.time() - self.__button_opt_right_last_pressed

                # If the start button is pressed long enough, enable the controlles
                if(time_difference > 1):
                    self.__joy_enabled = True
                        
            # After the output was enabled, wait for the button to be released befor the button_pressed var is set to false.
            elif (self.__joy_enabled == True and self.__button_opt_right_pressed == True and self.__button_opt_right == 0):
                self.__button_opt_right_pressed = False
            
            # if joy is disabled and the start_button was pressed but is released before the output could be 
            # enabled, the button_pressed var is set to false to receive a new button press.
            elif (self.__joy_enabled == False and self.__button_opt_right_pressed == True and self.__button_opt_right == 0):
                self.__button_opt_right_pressed = False

            #if joy is enabled and the enable button is pressed, disable the output.
            elif (self.__joy_enabled == True and self.__button_opt_right_pressed == False and self.__button_opt_right == 1):
                self.__joy_enabled = False


        # Print a message to show the current output status.
        if self.__joy_enabled != self.__joy_enabled_old:
            if (self.__joy_enabled == True and self.__joy_enabled_old == False):
                self.get_logger().info("Tele Op: ENABLED")
            else:
                if (self.__joy_enabled == False and self.__joy_enabled_old == True):
                    self.get_logger().info("Tele Op: DISABLED")

            # Set the old state to the new state
            self.__joy_enabled_old = self.__joy_enabled



    def __checkArmMode(self):

        # ----- Enable button check -----
        # If there was no joy msg received before and the arm mode is ENABLED, DISABLE it.
        if (self.__first_joy_msg_received == False and self.__arm_enabled != False):
            self.__arm_enabled = False
        
        elif (self.__first_joy_msg_received == True):
            #if current mode equals old mode and button is newly pressed, start timer
            if (self.__arm_enabled != self.__arm_enabled_old and self.__button_opt_left == 1 and self.__button_opt_left_pressed == False):
                self.__button_opt_left_pressed = True
                self.__button_opt_left_last_pressed = time.time()

            #if timer has been started before
            elif (self.__arm_enabled != self.__arm_enabled_old and self.__button_opt_left == 1 and self.__button_opt_left_pressed == True):
                # messure how long the start button has been pressed
                time_difference = time.time() - self.__button_opt_left_last_pressed
                # If the start button has been pressed long enough, switch mode
                if (time_difference > 1):
                    self.__arm_enabled = not self.__arm_enabled
                    if (self.__arm_enabled == True):
                        
                        self.get_logger().info("Control Mode: ARM")
                    else:
                        self.get_logger().info("Control Mode: BODY")
            
            #if mode has been switched, wait until start button is released, then change old mode status to enable new switch cycle
            elif (self.__arm_enabled == self.__arm_enabled_old and self.__button_opt_left_pressed == True and self.__button_opt_left == 0):
                self.__button_opt_left_pressed = False
                self.__arm_enabled_old = not self.__arm_enabled_old 
            
            # if button is not pressed long enough, reset
            elif (self.__arm_enabled != self.__arm_enabled_old and self.__button_opt_left_pressed == True and self.__button_opt_left == 0):
                self.__button_opt_left_pressed = False



    def __calcAndSendFlippers(self):

        if (self.__button_rb == 1 and self.__button_lb == 0):
            self.__flipper_direction = 1
        elif(self.__button_rb == 0 and self.__button_lb == 1):
            self.__flipper_direction = -1
        else:
            self.__flipper_direction = 0
            
        if(self.__arm_enabled == False):
            self.__flipper_cmd_fr.data = self.__button_y * self.__flipper_direction
            self.__flipper_cmd_fl.data = self.__button_x * self.__flipper_direction
            self.__flipper_cmd_rr.data = self.__button_b * self.__flipper_direction         
            self.__flipper_cmd_rl.data = self.__button_a * self.__flipper_direction
        else:
            self.__flipper_cmd_fr.data = 0
            self.__flipper_cmd_fl.data = 0
            self.__flipper_cmd_rr.data = 0         
            self.__flipper_cmd_rl.data = 0

            
        #send the commands
        if (self.__joy_enabled == True):
            self.__flipper_publisher_fr.publish(self.__flipper_cmd_fr)
            self.__flipper_publisher_fl.publish(self.__flipper_cmd_fl)
            self.__flipper_publisher_rr.publish(self.__flipper_cmd_rr)
            self.__flipper_publisher_rl.publish(self.__flipper_cmd_rl)



    def __calcAndSendChains(self):

        #check for arm mode and construct messages to be sent
        if(self.__arm_enabled == False):
            self.__chain_msg.linear.x = self.__left_stick_y
            self.__chain_msg.angular.z = self.__left_stick_x
        else:       
            self.__chain_msg.linear.x = 0.0
            self.__chain_msg.angular.z = 0.0

        #send movement commands
        if (self.__joy_enabled == True):
            self.__chain_publisher.publish(self.__chain_msg)
        


    def __calc_and_send_arm(self):

        #calc arm tilt        
        tilt = abs((self.__right_trigger -1)/2) + (self.__left_trigger -1)/2
        
        #calc gripper movement
        #gripper_mode = float(self.__button_a - self.__button_b)
        gripper_spin = float(self.__button_rb - self.__button_lb)
        
        #check for arm mode 
        if(self.__arm_enabled == True):
            #construct arm message
            self.__arm_msg.twist.linear.x = self.__left_stick_y
            self.__arm_msg.twist.linear.y = self.__d_pad_x
            self.__arm_msg.twist.linear.z = self.__d_pad_y
            self.__arm_msg.twist.angular.z = self.__left_stick_x        
            self.__arm_msg.twist.angular.y = tilt
            self.__arm_msg.twist.angular.x = 0.0

            #construct gripper message
            #self.__gripper_msg.linear.x = gripper_mode
            self.__gripper_msg.velocities = [self.__right_stick_y, self.__right_stick_x, gripper_spin]

        else:
            self.__arm_msg.twist.linear.x = 0.0
            self.__arm_msg.twist.linear.y = 0.0
            self.__arm_msg.twist.linear.z = 0.0
            self.__arm_msg.twist.angular.z = 0.0        
            self.__arm_msg.twist.angular.y = 0.0
            self.__arm_msg.twist.angular.x = 0.0

            self.__gripper_msg.velocities = [0.0, 0.0, 0.0]

        #add time stamps and seq number
        self.__gripper_msg.header.stamp = self.get_clock().now().to_msg()
        self.__arm_msg.header.stamp = self.get_clock().now().to_msg()

        #send arm commands
        if (self.__joy_enabled == True):
            self.__arm_publisher.publish(self.__arm_msg)
            self.__gripper_publisher.publish(self.__gripper_msg)
        


    def __timerCallback(self):
        self.__checkCMDOutputEnable()
        self.__checkArmMode()
        self.__calcAndSendFlippers()
        self.__calcAndSendChains()
        self.__calc_and_send_arm()



    def __joyCallback(self, msg):
        #save controller input to local variables

        #axes
        self.__left_stick_x_lock.acquire()
        self.__left_stick_x = msg.axes[left_stick_x]
        self.__left_stick_x_lock.release()

        self.__left_stick_y_lock.acquire()
        self.__left_stick_y = msg.axes[left_stick_y]
        self.__left_stick_y_lock.release()

        self.__left_trigger_lock.acquire()
        self.__left_trigger = msg.axes[left_trigger]
        self.__left_trigger_lock.release()

        self.__right_stick_x_lock.acquire()
        self.__right_stick_x = msg.axes[right_stick_x]
        self.__right_stick_x_lock.release()
        
        self.__right_stick_y_lock.acquire()
        self.__right_stick_y = msg.axes[right_stick_y]
        self.__right_stick_y_lock.release()

        self.__right_trigger_lock.acquire()
        self.__right_trigger = msg.axes[right_trigger]
        self.__right_trigger_lock.release()

        self.__d_pad_y_lock.acquire()
        self.__d_pad_y = msg.axes[d_pad_y]
        self.__d_pad_y_lock.release()

        self.__d_pad_x_lock.acquire()
        self.__d_pad_x = msg.axes[d_pad_x]
        self.__d_pad_x_lock.release()

        #buttons
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

        self.__button_opt_right_lock.acquire()
        self.__button_opt_right = msg.buttons[opt_right_button]
        self.__button_opt_right_lock.release()

        self.__button_opt_left_lock.acquire()
        self.__button_opt_left = msg.buttons[opt_left_button]
        self.__button_opt_left_lock.release()

        self.__button_space_lock.acquire()
        self.__button_space = msg.buttons[space_button]
        self.__button_space_lock.release()


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

        self.__chain_publisher = self.create_publisher(
            Twist,
            'cmd/drive',
            #'chain_controller/cmd_vel_unstamped',
            1
        )

        self.__flipper_publisher_fr = self.create_publisher(
            Int8,
            'cmd/flipper_fr',
            1
        )

        self.__flipper_publisher_fl = self.create_publisher(
            Int8,
            'cmd/flipper_fl',
            1
        )

        self.__flipper_publisher_rr = self.create_publisher(
            Int8,
            'cmd/flipper_rr',
            1
        )   

        self.__flipper_publisher_rl = self.create_publisher(
            Int8,
            'cmd/flipper_rl',
            1
        )

        self.__arm_publisher = self.create_publisher(
            TwistStamped,
            "cmd/arm/joy_twiststamped",
            1
        )

        self.__gripper_publisher = self.create_publisher(
            JointJog,
            "cmd/arm/joy_joint",
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
