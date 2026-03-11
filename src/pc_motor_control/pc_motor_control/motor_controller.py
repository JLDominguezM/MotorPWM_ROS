import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('pc_motor_controller')
        
        # Create separate publishers for the left and right motors
        self.publisher_left = self.create_publisher(Float32, '/cmd_pwm_left', 10)
        self.publisher_right = self.create_publisher(Float32, '/cmd_pwm_right', 10)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.wheel_base = 0.19 
        self.wheel_radius = 0.05
        
        self.max_linear_vel = 0.5   
        self.max_angular_vel = 3.0  
        self.max_wheel_rads = (2.0 * self.max_linear_vel + self.max_angular_vel * self.wheel_base) / (2.0 * self.wheel_radius)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        # Calculate required speeds for both wheels
        wl = (2.0 * v - w * self.wheel_base) / (2.0 * self.wheel_radius)
        wr = (2.0 * v + w * self.wheel_base) / (2.0 * self.wheel_radius)
        
        # Map the calculated rad/s to a PWM duty cycle 
        pwm_val_l = wl / self.max_wheel_rads
        pwm_val_r = wr / self.max_wheel_rads
        
        pwm_val_l = max(min(pwm_val_l, 1.0), -1.0)
        pwm_val_r = max(min(pwm_val_r, 1.0), -1.0)
        
        # publish to the left motor
        pwm_msg_l = Float32()
        pwm_msg_l.data = pwm_val_l
        self.publisher_left.publish(pwm_msg_l)

        # publish to the right motor
        pwm_msg_r = Float32()
        pwm_msg_r.data = pwm_val_r
        self.publisher_right.publish(pwm_msg_r)
        
        self.get_logger().info(f'Sim (v:{v:.2f}, w:{w:.2f}) -> L_PWM: {pwm_val_l:.2f} | R_PWM: {pwm_val_r:.2f}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        stop_msg = Float32()
        stop_msg.data = 0.0
        motor_controller.publisher_left.publish(stop_msg)
        motor_controller.publisher_right.publish(stop_msg)
        
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()