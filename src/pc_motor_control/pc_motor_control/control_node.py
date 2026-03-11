import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import numpy as np


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # PID gains (tuned from experiments)
        self.declare_parameter('Kp', 0.011)
        self.declare_parameter('Ki', 0.022)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('sample_time', 0.05)          # Ts in seconds
        self.declare_parameter('encoder_ppr', 422.0)         # 11 PPR * 4 (X4) * 9.6 gear ratio
        self.declare_parameter('encoder_sign', -1.0)         # -1 to flip encoder direction
        self.declare_parameter('filter_size', 5)             # moving average window for velocity
        self.declare_parameter('slew_rate', 100.0)           # max setpoint change in rad/s per second

        self.Ts = self.get_parameter('sample_time').value
        self.ppr = self.get_parameter('encoder_ppr').value

        # PID state
        self.set_point_raw = 0.0      # raw from topic
        self.set_point_ramped = 0.0   # after slew rate limiter
        self.error_sum = 0.0
        self.prev_error = 0.0

        # Encoder state
        self.prev_ticks = 0
        self.ticks = 0
        self.first_encoder_msg = True

        # Velocity filter
        self.omega_buffer = []

        # Publishers
        self.cmd_pub = self.create_publisher(Float32, '/cmd_pwm', 10)
        self.vel_pub = self.create_publisher(Float32, '/motor_output', 10)
        self.sp_pub = self.create_publisher(Float32, '/set_point_ramped', 10)

        # Subscribers
        self.create_subscription(Float32, '/set_point', self.set_point_callback, 10)
        self.create_subscription(Int32, '/encoder', self.encoder_callback, 10)

        # Control loop timer
        self.timer = self.create_timer(self.Ts, self.control_loop)

    def set_point_callback(self, msg):
        self.set_point_raw = msg.data

    def encoder_callback(self, msg):
        if self.first_encoder_msg:
            self.prev_ticks = msg.data
            self.first_encoder_msg = False
        self.ticks = msg.data

    def control_loop(self):
        Ts = self.Ts
        ppr = self.ppr

        # Slew rate limiter: ramp setpoint instead of jumping
        slew_rate = self.get_parameter('slew_rate').value
        max_change = slew_rate * Ts
        diff = self.set_point_raw - self.set_point_ramped
        if abs(diff) > max_change:
            self.set_point_ramped += max_change * np.sign(diff)
        else:
            self.set_point_ramped = self.set_point_raw

        # Publish ramped setpoint for rqt_plot
        sp_msg = Float32()
        sp_msg.data = float(self.set_point_ramped)
        self.sp_pub.publish(sp_msg)

        # Compute angular velocity from encoder delta (rad/s)
        enc_sign = self.get_parameter('encoder_sign').value
        filter_size = self.get_parameter('filter_size').value
        delta_ticks = self.ticks - self.prev_ticks
        self.prev_ticks = self.ticks
        raw_omega = enc_sign * (2.0 * np.pi * delta_ticks) / (ppr * Ts)

        # Moving average filter to reduce encoder noise
        self.omega_buffer.append(raw_omega)
        if len(self.omega_buffer) > filter_size:
            self.omega_buffer.pop(0)
        omega = float(np.mean(self.omega_buffer))

        # Publish measured velocity for rqt_plot
        vel_msg = Float32()
        vel_msg.data = float(omega)
        self.vel_pub.publish(vel_msg)

        # PID error (use ramped setpoint, not raw)
        error = self.set_point_ramped - omega

        # Read gains dynamically
        Kp = self.get_parameter('Kp').value
        Ki = self.get_parameter('Ki').value
        Kd = self.get_parameter('Kd').value

        # Compute PID output before integrator update
        p_term = Kp * error
        i_term = Ki * Ts * (self.error_sum + error)
        d_term = Kd * (error - self.prev_error) / Ts
        u = p_term + i_term + d_term

        # Clamp to [-1, 1]
        u_clamped = float(np.clip(u, -1.0, 1.0))

        # Anti-windup: only update integrator if output is not saturated
        # or if the error is helping reduce saturation
        if abs(u) < 1.0 or (error * u_clamped < 0):
            self.error_sum += error

        self.prev_error = error

        # Publish motor command
        cmd_msg = Float32()
        cmd_msg.data = u_clamped
        self.cmd_pub.publish(cmd_msg)

        self.get_logger().info(
            f'SP: {self.set_point_ramped:.2f} | ω: {omega:.2f} | e: {error:.2f} | u: {u_clamped:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop motor on exit
        stop_msg = Float32()
        stop_msg.data = 0.0
        node.cmd_pub.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
