import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('pc_motor_controller')
        
        self.publisher_ = self.create_publisher(Float32, '/cmd_pwm', 10)

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
        
        wl = (2.0 * v - w * self.wheel_base) / (2.0 * self.wheel_radius)
        
        # 2. Map the calculated rad/s to a PWM duty cycle [-1.0, 1.0]
        pwm_val = wl / self.max_wheel_rads
        
        # 3. Clamp the values to strictly stay within [-1.0, 1.0] to protect the motor driver
        pwm_val = max(min(pwm_val, 1.0), -1.0)
        
        # 4. Publish to the physical ESP32/Arduino
        pwm_msg = Float32()
        pwm_msg.data = pwm_val
        self.publisher_.publish(pwm_msg)
        
        self.get_logger().info(f'Sim (v:{v:.2f}, w:{w:.2f}) -> Motor PWM: {pwm_val:.2f}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        # Failsafe: send 0 velocity on exit so the motor stops spinning
        stop_msg = Float32()
        stop_msg.data = 0.0
        motor_controller.publisher_.publish(stop_msg)
        
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()