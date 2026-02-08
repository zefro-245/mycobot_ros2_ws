import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class IKInternalNode(Node):
    def __init__(self):
        super().__init__('ik_internal_node')

        # Internal joint estimate (used for IK)
        self.q_est = None

        # Desired end-effector position (meters)
        self.x_des = np.array([0.18, 0.00, 0.25])

        # IK gain
        self.K = 0.5

        # Subscribe ONLY to initialize q_est
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Run IK at fixed rate (1 Hz)
        self.timer = self.create_timer(1.0, self.ik_loop)

        self.get_logger().info("Internal IK node started")

    # -----------------------------
    # Joint state callback (ONCE)
    # -----------------------------
    def joint_callback(self, msg):
        if self.q_est is None and len(msg.position) >= 6:
            self.q_est = np.array(msg.position[:6])
            self.get_logger().info(
                f"Initialized q_est (deg): {np.degrees(self.q_est).round(2)}"
            )

    # -----------------------------
    # Denavit–Hartenberg transform
    # -----------------------------
    def dh(self, a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    # -----------------------------
    # Forward kinematics
    # -----------------------------
    def forward_kinematics(self, q):
        a = [0, 0, 0.1104, 0.096, 0, 0]
        alpha = [np.pi/2, np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2]
        d = [0.13156, 0, 0, 0.06062, 0.07318, 0.0456]
        theta_offsets = [np.pi/2, -np.pi/2, 0, 0, 0, 0]

        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh(a[i], alpha[i], d[i], q[i] + theta_offsets[i])

        return T

    # -----------------------------
    # Jacobian (position only)
    # -----------------------------
    def compute_jacobian(self, q):
        T = np.eye(4)

        z_axes = []
        origins = []

        a = [0, 0, 0.1104, 0.096, 0, 0]
        alpha = [np.pi/2, np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2]
        d = [0.13156, 0, 0, 0.06062, 0.07318, 0.0456]
        theta_offsets = [np.pi/2, -np.pi/2, 0, 0, 0, 0]

        origins.append(T[0:3, 3])
        z_axes.append(T[0:3, 2])

        for i in range(6):
            T = T @ self.dh(a[i], alpha[i], d[i], q[i] + theta_offsets[i])
            origins.append(T[0:3, 3])
            z_axes.append(T[0:3, 2])

        o_n = origins[-1]

        J = np.zeros((3, 6))
        for i in range(6):
            J[:, i] = np.cross(z_axes[i], o_n - origins[i])

        return J

    # -----------------------------
    # One IK update step
    # -----------------------------
    def ik_step(self, q):
        T = self.forward_kinematics(q)
        x = T[0:3, 3]

        error = self.x_des - x
        J = self.compute_jacobian(q)

        dq = self.K * np.linalg.pinv(J) @ error
        return q + dq, error

    # -----------------------------
    # Main IK loop
    # -----------------------------
    def ik_loop(self):
        if self.q_est is None:
            return

        self.q_est, error = self.ik_step(self.q_est)

        T = self.forward_kinematics(self.q_est)
        x = T[0:3, 3]

        self.get_logger().info(
            f"q_est (deg): {np.degrees(self.q_est).round(2)} | "
            f"EE pos: {x.round(3)} | "
            f"error: {error.round(4)}"
        )


def main():
    rclpy.init()
    node = IKInternalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
