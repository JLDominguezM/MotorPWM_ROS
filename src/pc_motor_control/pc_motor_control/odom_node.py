import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Robot physical parameters (Matching your motor_controller.py)
        self.r = 0.05       # wheel radius
        self.L = 0.19       # wheel base
        self.ppr = 422.0    # pulses per revolution (Update this if your gear ratio is different)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_ticks_l = 0
        self.prev_ticks_r = 0
        self.first_l = True
        self.first_r = True
        
        self.last_time = self.get_clock().now()
        
        # Subscribers to hardware encoders
        self.sub_enc_l = self.create_subscription(Int32, '/encoder_left', self.enc_l_callback, 10)
        self.sub_enc_r = self.create_subscription(Int32, '/encoder_right', self.enc_r_callback, 10)
        
        # Publisher for Odometry (Feedback for pb-j_control)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal storage
        self.ticks_l = 0
        self.ticks_r = 0
        
        # Update timer (50Hz)
        self.timer = self.create_timer(0.02, self.update_odometry)

    def enc_l_callback(self, msg):
        self.ticks_l = msg.data

    def enc_r_callback(self, msg):
        self.ticks_r = msg.data

    def update_odometry(self):
        # Initialize previous ticks on the first pass
        if self.first_l or self.first_r:
            self.prev_ticks_l = self.ticks_l
            self.prev_ticks_r = self.ticks_r
            self.first_l = False
            self.first_r = False
            self.last_time = self.get_clock().now()
            return
            
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0: return

        # Delta ticks
        delta_l = self.ticks_l - self.prev_ticks_l
        delta_r = self.ticks_r - self.prev_ticks_r
        
        self.prev_ticks_l = self.ticks_l
        self.prev_ticks_r = self.ticks_r
        
        # Distance traveled by each wheel
        dist_l = (delta_l / self.ppr) * (2.0 * math.pi * self.r)
        dist_r = (delta_r / self.ppr) * (2.0 * math.pi * self.r)
        
        # Robot center displacement and rotation
        d_center = (dist_r + dist_l) / 2.0
        d_theta = (dist_r - dist_l) / self.L
        
        # Update global pose
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        
        # Calculate instantaneous velocities
        v = d_center / dt
        w = d_theta / dt
        
        self.last_time = current_time
        
        # --- Publish Odometry for the SMC/CTC Controllers ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        self.odom_pub.publish(odom)
        
        # --- Publish TF ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()