import rclpy
from rclpy.node import Node
import math
import time

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist




# Class -----------------------------------------------------------------------
class ArmJoyTrajectory(Node):

    def __init__(self):
        #Entrypoint of the class
        super().__init__('arm_joy_trajectory')

        #define variables
        self.__cmd_angular_x = float(0)
        self.__cmd_angular_y = float(0)
        self.__cmd_angular_z = float(0)
        self.__cmd_linear_x = float(0)
        self.__cmd_linear_y = float(0)
        self.__cmd_linear_z = float(0)

        #build kdtree


        #Init class ->create subscriber, create timer
        self.__readParams()
        self.__createSubscribers()
        self.__createPublishers()
        self.__createTimer()

        print("arm_joy_trajectory initiated")



    def __readParams(self):
        #declare parameters
        self.declare_parameter('Publish_rate', 20)              #[Hz]

        #read parameters
        self.__Publish_rate = rclpy.parameter.Parameter(
            'Publish_rate',
            rclpy.Parameter.Type.DOUBLE,
            20.0
        )


 
    def __calc_and_send_arm(self):
        #calc arm movement

    def __timerCallback(self):
        self.__calc_and_send_arm()



    def __teleopCallback(self, msg):
        self.__cmd_linear_x = msg.linear.x
        self.__cmd_linear_y = msg.linear.y
        self.__cmd_linear_x = msg.linear.x
        self.__cmd_angular_z = msg.angular.z
        self.__cmd_angular_y = msg.angular.y
        self.__cmd_angular_x = msg.angular.x



    def __createSubscribers(self):
        # Create subscribers

        self._teleop_sub = self.create_subscription(
            Twist,
            'cmd/arm/twist_arm',
            self.__teleopCallback,
            5,
        )

    def __createPublishers(self):
        # Create publishers
        self.__arm_publisher = self.create_publisher(
            JointTrajectory,
            "cmd/arm/joint_trajectory",
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

    arm_joy_trajectory = ArmJoyTrajectory()

    rclpy.spin(arm_joy_trajectory)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_joy_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()