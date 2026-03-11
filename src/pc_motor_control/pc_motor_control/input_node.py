import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np


class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # Parameters
        self.declare_parameter('signal_type', 'step')       # step, square, sine
        self.declare_parameter('amplitude', 5.0)             # rad/s
        self.declare_parameter('frequency', 0.5)             # Hz (for square/sine)
        self.declare_parameter('offset', 0.0)                # rad/s offset
        self.declare_parameter('sample_time', 0.01)          # 10 ms

        self.publisher_ = self.create_publisher(Float32, '/set_point', 10)

        Ts = self.get_parameter('sample_time').value
        self.timer = self.create_timer(Ts, self.timer_callback)
        self.t = 0.0
        self.Ts = Ts

    def timer_callback(self):
        signal_type = self.get_parameter('signal_type').value
        amplitude = self.get_parameter('amplitude').value
        freq = self.get_parameter('frequency').value
        offset = self.get_parameter('offset').value

        if signal_type == 'step':
            value = amplitude
        elif signal_type == 'square':
            value = amplitude * np.sign(np.sin(2.0 * np.pi * freq * self.t))
        elif signal_type == 'sine':
            value = amplitude * np.sin(2.0 * np.pi * freq * self.t)
        else:
            value = 0.0

        msg = Float32()
        msg.data = float(value + offset)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Set point: {msg.data:.2f} rad/s')

        self.t += self.Ts


def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
