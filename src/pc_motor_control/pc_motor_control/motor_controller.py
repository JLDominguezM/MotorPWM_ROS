import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('pc_motor_controller')
        self.publisher_ = self.create_publisher(Float32, '/cmd_pwm', 10)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.time_counter = 0.0

    def timer_callback(self):
        msg = Float32()
        
        msg.data = math.sin(self.time_counter)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Mandando velocidad al motor: {msg.data:.2f}')
        
        self.time_counter += 0.1

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:

        stop_msg = Float32()
        stop_msg.data = 0.0
        motor_controller.publisher_.publish(stop_msg)
        pass
        
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
