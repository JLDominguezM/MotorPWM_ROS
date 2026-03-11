import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import numpy as np
import time


class TuningNode(Node):
    """
    Three experiments to find PID/PI constants:
      experiment 1: Open-loop step response  → fits FOPDT model → Cohen-Coon & Z-N open-loop
      experiment 2: Ziegler-Nichols closed-loop → P-only, you increase Kp until oscillations
      experiment 3: Relay auto-tune → relay feedback induces limit cycle → extracts Ku, Tu
    """

    def __init__(self):
        super().__init__('tuning_node')

        # Common parameters
        self.declare_parameter('experiment', 1)              # 1, 2, or 3
        self.declare_parameter('sample_time', 0.05)          # Ts seconds (50ms for smoother readings)
        self.declare_parameter('encoder_ppr', 422.0)         # 11 PPR * 4 (X4) * 9.6 gear ratio
        self.declare_parameter('encoder_sign', -1.0)         # -1 to flip encoder direction
        self.declare_parameter('duration', 5.0)              # experiment duration seconds

        # Experiment 1 parameters (open-loop step)
        self.declare_parameter('step_amplitude', 0.5)        # PWM value [0, 1]

        # Experiment 2 parameters (Z-N closed-loop)
        self.declare_parameter('zn_setpoint', 5.0)           # target rad/s
        self.declare_parameter('zn_Kp', 0.01)                # starting Kp (increase until oscillations)

        # Experiment 3 parameters (relay auto-tune)
        self.declare_parameter('relay_setpoint', 0.0)        # 0 = auto-detect from step response
        self.declare_parameter('relay_amplitude', 0.5)       # relay output ±d

        self.Ts = self.get_parameter('sample_time').value
        self.ppr = self.get_parameter('encoder_ppr').value
        self.duration = self.get_parameter('duration').value
        self.experiment = self.get_parameter('experiment').value

        # Publishers
        self.cmd_pub = self.create_publisher(Float32, '/cmd_pwm', 10)
        self.vel_pub = self.create_publisher(Float32, '/motor_output', 10)
        self.sp_pub = self.create_publisher(Float32, '/set_point', 10)

        # Subscriber
        self.create_subscription(Int32, '/encoder', self.encoder_callback, 10)

        # Encoder state
        self.ticks = 0
        self.prev_ticks = 0
        self.first_encoder_msg = True

        # Moving average filter for omega (reduces encoder noise)
        self.omega_buffer = []
        self.omega_filter_size = 5

        # Data recording
        self.time_data = []
        self.omega_data = []
        self.cmd_data = []
        self.start_time = None
        self.finished = False

        # Relay state (experiment 3)
        self.relay_crossings = []
        self.relay_last_sign = 0
        self.relay_omega_max = -1e9
        self.relay_omega_min = 1e9
        # Phase 1: brief open-loop step to find steady-state speed
        self.relay_phase = 'warmup'  # 'warmup' -> 'detect_ss' -> 'relay'
        self.relay_ss_omega = 0.0
        self.relay_warmup_data = []

        self.get_logger().info(f'=== TUNING EXPERIMENT {self.experiment} ===')
        if self.experiment == 1:
            self.get_logger().info('Open-loop step response. Motor will run at constant PWM.')
        elif self.experiment == 2:
            self.get_logger().info(
                f'Ziegler-Nichols P-only. Kp={self.get_parameter("zn_Kp").value:.4f}. '
                'If no oscillations, restart with higher zn_Kp.'
            )
        elif self.experiment == 3:
            d = self.get_parameter('relay_amplitude').value
            self.get_logger().info(
                f'Relay auto-tune. d={d:.2f}. '
                'Phase 1: detect steady-state speed. Phase 2: relay feedback.'
            )

        self.timer = self.create_timer(self.Ts, self.control_loop)

    def encoder_callback(self, msg):
        if self.first_encoder_msg:
            self.prev_ticks = msg.data
            self.first_encoder_msg = False
        self.ticks = msg.data

    def get_omega(self):
        enc_sign = self.get_parameter('encoder_sign').value
        delta = self.ticks - self.prev_ticks
        self.prev_ticks = self.ticks
        raw_omega = enc_sign * (2.0 * np.pi * delta) / (self.ppr * self.Ts)

        # Moving average filter
        self.omega_buffer.append(raw_omega)
        if len(self.omega_buffer) > self.omega_filter_size:
            self.omega_buffer.pop(0)
        return float(np.mean(self.omega_buffer))

    def send_cmd(self, u):
        u = float(np.clip(u, -1.0, 1.0))
        msg = Float32()
        msg.data = u
        self.cmd_pub.publish(msg)
        return u

    def publish_vel(self, omega):
        msg = Float32()
        msg.data = float(omega)
        self.vel_pub.publish(msg)

    def publish_sp(self, sp):
        msg = Float32()
        msg.data = float(sp)
        self.sp_pub.publish(msg)

    def control_loop(self):
        if self.finished:
            return

        if self.start_time is None:
            self.start_time = time.time()

        t = time.time() - self.start_time
        omega = self.get_omega()

        self.publish_vel(omega)

        if t > self.duration:
            self.send_cmd(0.0)
            self.finished = True
            self.analyze()
            return

        if self.experiment == 1:
            self.run_step_response(t, omega)
        elif self.experiment == 2:
            self.run_ziegler_nichols(t, omega)
        elif self.experiment == 3:
            self.run_relay(t, omega)

    # EXPERIMENT 1: Open-loop step response
    def run_step_response(self, t, omega):
        amp = self.get_parameter('step_amplitude').value
        # 1s of no input, then step
        if t < 1.0:
            u = self.send_cmd(0.0)
        else:
            u = self.send_cmd(amp)

        self.publish_sp(amp if t >= 1.0 else 0.0)
        self.time_data.append(t)
        self.omega_data.append(omega)
        self.cmd_data.append(u)

    # EXPERIMENT 2: Ziegler-Nichols closed-loop (P-only)
    def run_ziegler_nichols(self, t, omega):
        sp = self.get_parameter('zn_setpoint').value
        Kp = self.get_parameter('zn_Kp').value

        error = sp - omega
        u = Kp * error
        u = self.send_cmd(u)

        self.publish_sp(sp)
        self.time_data.append(t)
        self.omega_data.append(omega)
        self.cmd_data.append(u)

    # EXPERIMENT 3: Relay auto-tune
    def run_relay(self, t, omega):
        d = self.get_parameter('relay_amplitude').value
        sp_param = self.get_parameter('relay_setpoint').value

        # Phase 1: warmup — run open-loop step for 3s to find steady state
        if self.relay_phase == 'warmup':
            if t < 0.5:
                self.send_cmd(0.0)
                self.publish_sp(0.0)
            elif t < 3.0:
                self.send_cmd(d)
                self.relay_warmup_data.append(omega)
                self.publish_sp(0.0)
            else:
                # Compute steady-state speed from last 1s of warmup
                if len(self.relay_warmup_data) > 10:
                    n_last = max(1, len(self.relay_warmup_data) // 2)
                    self.relay_ss_omega = float(np.mean(self.relay_warmup_data[-n_last:]))
                else:
                    self.relay_ss_omega = 5.0

                # Set point = half of steady state (so motor can oscillate around it)
                if sp_param > 0.0:
                    self.relay_ss_omega = sp_param
                else:
                    self.relay_ss_omega = self.relay_ss_omega * 0.5

                self.relay_phase = 'relay'
                self.get_logger().info(
                    f'Warmup done. Steady-state ω ≈ {self.relay_ss_omega:.2f} rad/s. '
                    f'Starting relay around setpoint = {self.relay_ss_omega:.2f} rad/s'
                )

            self.time_data.append(t)
            self.omega_data.append(omega)
            self.cmd_data.append(d if t >= 0.5 else 0.0)
            return

        # Phase 2: relay feedback
        sp = self.relay_ss_omega
        error = sp - omega

        if error > 0:
            u = d
            current_sign = 1
        else:
            u = -d
            current_sign = -1

        # Detect zero crossings of error
        if self.relay_last_sign != 0 and current_sign != self.relay_last_sign:
            self.relay_crossings.append(t)
            if current_sign < 0:
                self.relay_omega_max = -1e9
            else:
                self.relay_omega_min = 1e9

        self.relay_last_sign = current_sign
        self.relay_omega_max = max(self.relay_omega_max, omega)
        self.relay_omega_min = min(self.relay_omega_min, omega)

        u = self.send_cmd(u)
        self.publish_sp(sp)
        self.time_data.append(t)
        self.omega_data.append(omega)
        self.cmd_data.append(u)

    # ANALYSIS
    def analyze(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('EXPERIMENT FINISHED — ANALYZING DATA')
        self.get_logger().info('=' * 60)

        if self.experiment == 1:
            self.analyze_step()
        elif self.experiment == 2:
            self.analyze_zn()
        elif self.experiment == 3:
            self.analyze_relay()

        # Save CSV
        self.save_csv()

    def analyze_step(self):
        """Fit FOPDT model from open-loop step response."""
        t = np.array(self.time_data)
        y = np.array(self.omega_data)
        amp = self.get_parameter('step_amplitude').value

        # Find step start (t >= 1.0)
        step_idx = np.searchsorted(t, 1.0)
        t_step = t[step_idx:] - t[step_idx]
        y_step = y[step_idx:]

        # Baseline and steady state
        y0 = np.mean(y[:step_idx]) if step_idx > 5 else 0.0
        y_step = y_step - y0

        # Steady state: average of last 20% of data
        n = len(y_step)
        if n < 10:
            self.get_logger().error('Not enough data. Increase duration.')
            return

        y_ss = np.mean(y_step[int(0.8 * n):])

        if abs(y_ss) < 0.01:
            self.get_logger().error('Motor did not respond. Check wiring or increase step_amplitude.')
            return

        # Plant gain
        K = y_ss / amp

        # Use cumulative approach for noisy data
        # Find time to reach 28.3% and 63.2% of steady state
        y_abs = np.abs(y_step)
        y_ss_abs = abs(y_ss)

        t_28 = None
        t_63 = None
        for i in range(n):
            if t_28 is None and y_abs[i] >= 0.283 * y_ss_abs:
                t_28 = t_step[i]
            if t_63 is None and y_abs[i] >= 0.632 * y_ss_abs:
                t_63 = t_step[i]

        if t_28 is None:
            t_28 = t_step[0]
        if t_63 is None:
            t_63 = t_step[-1]

        # Two-point method for FOPDT: tau and L
        tau = 1.5 * (t_63 - t_28)
        L = t_63 - tau

        if L < self.Ts:
            L = self.Ts
        if tau < self.Ts:
            tau = self.Ts

        self.get_logger().info(f'Plant Model (FOPDT):')
        self.get_logger().info(f'  K   (gain)          = {K:.4f}  (rad/s per unit PWM)')
        self.get_logger().info(f'  tau (time constant)  = {tau:.4f} s')
        self.get_logger().info(f'  L   (dead time)      = {L:.4f} s')
        self.get_logger().info(f'  y_ss (steady state)  = {y_ss:.4f} rad/s  (at PWM={amp})')
        self.get_logger().info(f'  Max speed at PWM=1.0 ≈ {y_ss/amp:.1f} rad/s')
        self.get_logger().info('')

        # ---- Ziegler-Nichols Open-Loop Rules ----
        r = L / tau
        self.get_logger().info('--- Ziegler-Nichols Open-Loop ---')

        zn_p_Kp = (1.0 / K) * (tau / L)
        zn_pi_Kp = (0.9 / K) * (tau / L)
        zn_pi_Ti = L / 0.3
        zn_pi_Ki = zn_pi_Kp / zn_pi_Ti
        zn_pid_Kp = (1.2 / K) * (tau / L)
        zn_pid_Ti = 2.0 * L
        zn_pid_Td = 0.5 * L
        zn_pid_Ki = zn_pid_Kp / zn_pid_Ti
        zn_pid_Kd = zn_pid_Kp * zn_pid_Td

        self.get_logger().info(f'  P:   Kp = {zn_p_Kp:.6f}')
        self.get_logger().info(f'  PI:  Kp = {zn_pi_Kp:.6f}, Ki = {zn_pi_Ki:.6f}')
        self.get_logger().info(f'  PID: Kp = {zn_pid_Kp:.6f}, Ki = {zn_pid_Ki:.6f}, Kd = {zn_pid_Kd:.6f}')
        self.get_logger().info('')

        # ---- Cohen-Coon Rules ----
        self.get_logger().info('--- Cohen-Coon ---')
        cc_pi_Kp = (1.0 / (K * r)) * (0.9 + r / 12.0)
        cc_pi_Ti = L * (30.0 + 3.0 * r) / (9.0 + 20.0 * r)
        cc_pi_Ki = cc_pi_Kp / cc_pi_Ti

        cc_pid_Kp = (1.0 / (K * r)) * (4.0 / 3.0 + r / 4.0)
        cc_pid_Ti = L * (32.0 + 6.0 * r) / (13.0 + 8.0 * r)
        cc_pid_Td = L * 4.0 / (11.0 + 2.0 * r)
        cc_pid_Ki = cc_pid_Kp / cc_pid_Ti
        cc_pid_Kd = cc_pid_Kp * cc_pid_Td

        self.get_logger().info(f'  PI:  Kp = {cc_pi_Kp:.6f}, Ki = {cc_pi_Ki:.6f}')
        self.get_logger().info(f'  PID: Kp = {cc_pid_Kp:.6f}, Ki = {cc_pid_Ki:.6f}, Kd = {cc_pid_Kd:.6f}')
        self.get_logger().info('')

        # ---- Lambda Tuning (conservative) ----
        self.get_logger().info('--- Lambda Tuning (lambda = 3*tau) ---')
        lam = 3.0 * tau
        lam_Kp = tau / (K * (lam + L))
        lam_Ti = tau
        lam_Ki = lam_Kp / lam_Ti

        self.get_logger().info(f'  PI:  Kp = {lam_Kp:.6f}, Ki = {lam_Ki:.6f}')

    def analyze_zn(self):
        """Detect sustained oscillations, extract Ku and Tu."""
        t = np.array(self.time_data)
        y = np.array(self.omega_data)
        Ku = self.get_parameter('zn_Kp').value

        # Use last 60% of data (skip transient)
        n = len(y)
        start = int(0.4 * n)
        y_tail = y[start:]
        t_tail = t[start:]

        if len(y_tail) < 20:
            self.get_logger().error('Not enough data. Increase duration.')
            return

        # Check if oscillations exist
        mean_y = np.mean(y_tail)
        y_centered = y_tail - mean_y

        # Find zero crossings
        crossings = []
        for i in range(1, len(y_centered)):
            if y_centered[i - 1] * y_centered[i] < 0:
                crossings.append(t_tail[i])

        # Amplitude check: real oscillations have significant amplitude
        amplitude = np.max(y_tail) - np.min(y_tail)
        mean_abs = np.mean(np.abs(y_tail)) if np.mean(np.abs(y_tail)) > 0 else 1.0
        relative_amplitude = amplitude / mean_abs

        self.get_logger().info(f'Current Kp (Ku candidate) = {Ku:.6f}')
        self.get_logger().info(f'Oscillation amplitude = {amplitude:.4f} rad/s')
        self.get_logger().info(f'Relative amplitude = {relative_amplitude:.2%} of mean')
        self.get_logger().info(f'Zero crossings = {len(crossings)}')

        if len(crossings) < 6:
            self.get_logger().warn(
                f'Only {len(crossings)} zero crossings. '
                'Oscillations not sustained. Increase zn_Kp and retry.'
            )
            return

        # Ultimate period: average full cycle (every 2 crossings)
        periods = []
        for i in range(0, len(crossings) - 2, 2):
            periods.append(crossings[i + 2] - crossings[i])

        Tu = float(np.mean(periods))
        Tu_std = float(np.std(periods))

        # Validate: real oscillations have consistent period
        # and period should be >> sample time
        min_valid_period = self.Ts * 8  # at least 8 samples per cycle

        if Tu < min_valid_period:
            self.get_logger().warn(
                f'Tu = {Tu:.4f}s is too short (< {min_valid_period:.3f}s). '
                f'This is likely noise, not real oscillations. '
                f'Increase zn_Kp significantly and retry.'
            )
            return

        if Tu_std / Tu > 0.5:
            self.get_logger().warn(
                f'Oscillation period is very inconsistent (std={Tu_std:.4f}, mean={Tu:.4f}). '
                'May not be true sustained oscillations. Try higher zn_Kp.'
            )

        if relative_amplitude < 0.1:
            self.get_logger().warn(
                f'Oscillation amplitude is small ({relative_amplitude:.1%}). '
                'Could be noise. Try higher zn_Kp for clearer oscillations.'
            )

        self.get_logger().info('')
        self.get_logger().info(f'Ku (ultimate gain)   = {Ku:.6f}')
        self.get_logger().info(f'Tu (ultimate period) = {Tu:.4f} s  (std={Tu_std:.4f})')
        self.get_logger().info('')

        self.print_zn_constants(Ku, Tu)

    def analyze_relay(self):
        """Extract Ku and Tu from relay feedback oscillations."""
        d = self.get_parameter('relay_amplitude').value
        crossings = self.relay_crossings

        self.get_logger().info(f'Relay crossings detected: {len(crossings)}')

        if len(crossings) < 4:
            self.get_logger().error(
                'Not enough relay crossings. Try:\n'
                '  - Increase duration (e.g. -p duration:=20.0)\n'
                '  - Increase relay_amplitude (e.g. -p relay_amplitude:=0.8)\n'
                '  - Set a specific relay_setpoint lower than motor max speed'
            )
            # Still print whatever data we have
            y = np.array(self.omega_data)
            self.get_logger().info(f'omega range: [{np.min(y):.2f}, {np.max(y):.2f}] rad/s')
            self.get_logger().info(f'omega mean: {np.mean(y):.2f} rad/s')
            return

        # Discard first 2 crossings (transient)
        crossings_clean = crossings[2:] if len(crossings) > 6 else crossings

        # Ultimate period from crossings (every 2 crossings = 1 full cycle)
        periods = []
        for i in range(0, len(crossings_clean) - 2, 2):
            periods.append(crossings_clean[i + 2] - crossings_clean[i])

        if len(periods) == 0:
            self.get_logger().error('Could not compute oscillation period.')
            return

        Tu = float(np.mean(periods))

        # Oscillation amplitude from data after relay phase starts
        y = np.array(self.omega_data)
        # Use last 60% of data
        n = len(y)
        y_tail = y[int(0.4 * n):]
        a = (np.max(y_tail) - np.min(y_tail)) / 2.0

        if a < 0.01:
            self.get_logger().error('No measurable oscillation amplitude.')
            return

        # Ku from describing function: Ku = 4d / (π * a)
        Ku = (4.0 * d) / (np.pi * a)

        self.get_logger().info(f'd (relay amplitude)   = {d:.4f}')
        self.get_logger().info(f'a (output amplitude)  = {a:.4f} rad/s')
        self.get_logger().info(f'Ku (ultimate gain)    = {Ku:.6f}')
        self.get_logger().info(f'Tu (ultimate period)  = {Tu:.4f} s')
        self.get_logger().info('')

        self.print_zn_constants(Ku, Tu)

    def print_zn_constants(self, Ku, Tu):
        self.get_logger().info('=== RECOMMENDED CONSTANTS ===')
        self.get_logger().info('')
        self.get_logger().info('--- Ziegler-Nichols Closed-Loop Rules ---')

        p_Kp = 0.5 * Ku
        pi_Kp = 0.45 * Ku
        pi_Ti = Tu / 1.2
        pi_Ki = pi_Kp / pi_Ti
        pid_Kp = 0.6 * Ku
        pid_Ti = Tu / 2.0
        pid_Td = Tu / 8.0
        pid_Ki = pid_Kp / pid_Ti
        pid_Kd = pid_Kp * pid_Td

        self.get_logger().info(f'  P:   Kp = {p_Kp:.6f}')
        self.get_logger().info(f'  PI:  Kp = {pi_Kp:.6f}, Ki = {pi_Ki:.6f}')
        self.get_logger().info(f'  PID: Kp = {pid_Kp:.6f}, Ki = {pid_Ki:.6f}, Kd = {pid_Kd:.6f}')
        self.get_logger().info('')

        # Tyreus-Luyben (less aggressive, good for motors)
        self.get_logger().info('--- Tyreus-Luyben (less aggressive) ---')
        tl_pi_Kp = 0.31 * Ku
        tl_pi_Ti = 2.2 * Tu
        tl_pi_Ki = tl_pi_Kp / tl_pi_Ti
        tl_pid_Kp = 0.45 * Ku
        tl_pid_Ti = 2.2 * Tu
        tl_pid_Td = Tu / 6.3
        tl_pid_Ki = tl_pid_Kp / tl_pid_Ti
        tl_pid_Kd = tl_pid_Kp * tl_pid_Td

        self.get_logger().info(f'  PI:  Kp = {tl_pi_Kp:.6f}, Ki = {tl_pi_Ki:.6f}')
        self.get_logger().info(f'  PID: Kp = {tl_pid_Kp:.6f}, Ki = {tl_pid_Ki:.6f}, Kd = {tl_pid_Kd:.6f}')

        self.get_logger().info('')
        self.get_logger().info('To test these constants:')
        self.get_logger().info(
            f'  ros2 run pc_motor_control control_node --ros-args '
            f'-p Kp:={pi_Kp:.6f} -p Ki:={pi_Ki:.6f} -p Kd:=0.0'
        )

    def save_csv(self):
        """Save experiment data to CSV for offline analysis."""
        filename = f'/home/dominguez/microros_ws/tuning_experiment_{self.experiment}.csv'
        try:
            data = np.column_stack([self.time_data, self.omega_data, self.cmd_data])
            np.savetxt(filename, data, delimiter=',',
                       header='time,omega,cmd', comments='')
            self.get_logger().info(f'Data saved to {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TuningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Stop motor
    stop_msg = Float32()
    stop_msg.data = 0.0
    node.cmd_pub.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
