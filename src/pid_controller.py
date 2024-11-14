import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # PID parameters
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0
        
        # Target position
        self.target_x = 5.0
        self.target_y = 5.0
        
        # Initialize errors
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0
        
        self.prev_error_angular = 0.0
        self.integral_angular = 0.0
        
        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Create a subscriber to the odometry topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        
        # Create a subscriber to the target position topic
        self.target_subscriber = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            QoSProfile(depth=10)
        )
        
        # Create a publisher for the velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10)
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract the yaw angle from the quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Debug prints
        self.get_logger().info(f'Current Position: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}')
        
    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        
        # Debug prints
        self.get_logger().info(f'Target Position: x={self.target_x}, y={self.target_y}')
        
    def control_loop(self):
        # Compute the distance to the target
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance_to_target = math.sqrt(error_x**2 + error_y**2)
        
        # Compute the angle to the target
        angle_to_target = math.atan2(error_y, error_x)
        angle_error = angle_to_target - self.current_yaw
        
        # Normalize the angle error to the range [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        
        # Compute the integral and derivative for linear velocity
        self.integral_linear += distance_to_target
        derivative_linear = distance_to_target - self.prev_error_linear
        
        # Compute the control signal for linear velocity
        control_linear = self.kp_linear * distance_to_target + self.ki_linear * self.integral_linear + self.kd_linear * derivative_linear
        
        # Compute the integral and derivative for angular velocity
        self.integral_angular += angle_error
        derivative_angular = angle_error - self.prev_error_angular
        
        # Compute the control signal for angular velocity
        control_angular = self.kp_angular * angle_error + self.ki_angular * self.integral_angular + self.kd_angular * derivative_angular
        
        # Create a Twist message
        twist = Twist()
        twist.linear.x = control_linear
        twist.angular.z = control_angular
        
        # Publish the control signal
        self.cmd_vel_publisher.publish(twist)
        
        # Debug prints
        self.get_logger().info(f'Control Signals: linear={control_linear}, angular={control_angular}')
        
        # Update previous errors
        self.prev_error_linear = distance_to_target
        self.prev_error_angular = angle_error

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()