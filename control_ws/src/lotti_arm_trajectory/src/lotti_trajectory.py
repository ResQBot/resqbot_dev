import rclpy
from rclpy.node import Node
from threading import Lock
import time

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState


# Global Defines --------------------------------------------------------------

# Class -----------------------------------------------------------------------
class ArmTrajectory(Node):

    def __init__(self):
        #Entrypoint of the class
        super().__init__('arm_trajectory')

        #twist message variables
        self.__twist_lin_x = float(0)
        self.__twist_lin_x_lock = Lock()
        self.__twist_lin_y = float(0)
        self.__twist_lin_y_lock = Lock()
        self.__twist_lin_z = float(0)
        self.__twist_lin_z_lock = Lock()
        self.__twist_ang_x = float(0)
        self.__twist_ang_x_lock = Lock()
        self.__twist_ang_y = float(0)        
        self.__twist_ang_y_lock = Lock()
        self.__twist_ang_z = float(0)
        self.__twist_ang_z_lock = Lock()
        #joint jog message variables
        self.__joint_spin = float(0)
        self.__joint_spin_lock = Lock()
        self.__joint_tilt = float(0)
        self.__joint_tilt_lock = Lock()
        self.__joint_swivel = float(0)
        self.__joint_swivel_lock = Lock()
        #joint state variables
        self.__joint1_state = float(0)
        self.__joint1_state_lock = Lock()
        self.__joint2_state = float(0)
        self.__joint2_state_lock = Lock()
        self.__joint3_state = float(0)
        self.__joint3_state_lock = Lock()
        self.__joint4_state = float(0)
        self.__joint4_state_lock = Lock()
        self.__joint5_state = float(0)
        self.__joint5_state_lock = Lock()
        self.__joint6_state = float(0)
        self.__joint6_state_lock = Lock()

        self.__max_vel = 1.5 #m/s
        self.__update_rate = 20 #Hz
        self.__speed_factor = self.__max_vel * (1/self.__update_rate)

        self.__arm_command = JointTrajectory()
        self.__arm_command.joint_names = ['arm_link1_joint', 'arm_link2_joint', 'arm_link3_joint', 'arm_link4_joint', 'arm_link5_joint', 'arm_link6_joint']

        #Init class ->create subscriber, create timer
        self.__readParams()
        self.__createSubscribers()
        self.__createPublishers()
        self.__createTimer()

        print("Trajectory converter initiated")



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

        #get forward kinematic from current joint positions to determine ee position + orientation


        #calc ee goal points
        self.__ee_new_x = self.__ee_cur_x + self.__twist_lin_x * self.__speed_factor
        self.__ee_new_x = self.__ee_cur_y + self.__twist_lin_y * self.__speed_factor
        self.__ee_new_x = self.__ee_cur_z + self.__twist_lin_z * self.__speed_factor

        #use inverse kinematics to calc joint velocities
        

        #add joint jog commands
        self.__arm_command.points.velocities.

        #set commands
        self.__arm_command.points.velocities.x = 
        

        #stamp header
        self.__arm_command.header.stamp = self.get_clock().now().to_msg()

        #send command to controller
        self.__arm_publisher(self.__arm_command)
        

        
    def __timerCallback(self):
        self.__calc_and_send_arm()



    def __jointCallback(self, msg):
        #save joint commands locally
        self.__joint_spin_lock.acquire()
        self.__joint_spin = msg.velocities[2]
        self.__joint_spin_lock.release()

        self.__joint_swivel_lock.acquire()
        self.__joint_swivel = msg.velocities[1]
        self.__joint_swivel_lock.release()
        
        self.__joint_tilt_lock.acquire()
        self.__joint_tilt = msg.velocities[0]
        self.__joint_tilt_lock.release()



    def __twistCallback(self, msg):
        #save twist commands locally
        self.__twist_lin_x_lock.acquire()
        self.__twist_lin_x = msg.twist.linear.x
        self.__twist_lin_x_lock.release()

        self.__twist_lin_y_lock.acquire()
        self.__twist_lin_y = msg.twist.linear.y
        self.__twist_lin_y_lock.release()

        self.__twist_lin_z_lock.acquire()
        self.__twist_lin_z = msg.twist.linear.z        
        self.__twist_lin_z_lock.release()   

        self.__twist_ang_x_lock.acquire()
        self.__twist_ang_x = msg.twist.linear.x
        self.__twist_ang_x_lock.release()

        self.__twist_ang_y_lock.acquire()
        self.__twist_ang_y = msg.twist.linear.y
        self.__twist_ang_y_lock.release()

        self.__twist_ang_z_lock.acquire()
        self.__twist_ang_z = msg.twist.linear.z
        self.__twist_ang_z_lock.release()

        

    def __stateCallback(self, msg):
        #save current joint states
        self.__joint1_state_lock.acquire()
        self.__joint1_state = msg.position[]
        self.__joint1_state_lock.release()

        self.__joint2_state_lock.acquire()
        self.__joint2_state = msg.position[]
        self.__joint2_state_lock.release()

        self.__joint3_state_lock.acquire()
        self.__joint3_state = msg.position[]
        self.__joint3_state_lock.release()

        self.__joint4_state_lock.acquire()
        self.__joint4_state = msg.position[]
        self.__joint4_state_lock.release()

        self.__joint5_state_lock.acquire()
        self.__joint5_state = msg.position[]
        self.__joint5_state_lock.release()

        self.__joint6_state_lock.acquire()
        self.__joint6_state = msg.position[]
        self.__joint6_state_lock.release()



    def __createSubscribers(self):
        # Create subscribers

        self._teleop_joint_sub = self.create_subscription(
            JointJog,
            'cmd/arm/joy_joint',
            self.__jointCallback,
            5,
        )

        self._teleop_twist_sub = self.create_subscription(
            TwistStamped,
            'cdm/arm/joy_twiststamped',
            self.__twistCallback,
            5,
        )

        self._joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.__stateCallback,
            5,
        )



    def __createPublishers(self):
        # Create publishers

        self.__arm_publisher = self.create_publisher(
            JointTrajectory,
            'arm_controller/joint_trajectory',
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

    arm_trajectory = ArmTrajectory()

    rclpy.spin(arm_trajectory)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()