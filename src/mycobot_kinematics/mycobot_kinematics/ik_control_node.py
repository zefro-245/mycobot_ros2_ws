import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class IKControlNode(Node):
    def __init__(self):
        super().__init__('ik_control_node')

        # Current actuated joint state (5 DOF)
        self.q = None

        # Desired EE position (meters)
        self.x_des = np.array([0.051, -0.052,  0.094])

        # IK gain
        self.K = 0.2

        # Controlled joints (MUST match controller)
        self.joint_names = [
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_link6'
        ]

        # Joint state subscriber
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Trajectory command publisher
        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("IK control node started")

    # -------------------------
    # Joint states callback
    # -------------------------
    def joint_callback(self, msg):
        if len(msg.position) >= 5:
            self.q = np.array(msg.position[:5])

    # -------------------------
    # DH transform
    # -------------------------
    def dh(self, a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    # -------------------------
    # Forward kinematics (6 DOF model)
    # -------------------------
    def forward_kinematics(self, q_full):
        a = [0, 0, 0.1104, 0.096, 0, 0]
        alpha = [np.pi/2, np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2]
        d = [0.13156, 0, 0, 0.06062, 0.07318, 0.0456]
        theta_offsets = [np.pi/2, -np.pi/2, 0, 0, 0, 0]

        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh(a[i], alpha[i], d[i], q_full[i] + theta_offsets[i])

        return T

    # -------------------------
    # Jacobian (position, 3×5)
    # -------------------------
    def compute_jacobian(self, q_full):
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
            T = T @ self.dh(a[i], alpha[i], d[i], q_full[i] + theta_offsets[i])
            origins.append(T[0:3, 3])
            z_axes.append(T[0:3, 2])

        o_n = origins[-1]

        J_full = np.zeros((3, 6))
        for i in range(6):
            J_full[:, i] = np.cross(z_axes[i], o_n - origins[i])

        # Use only first 5 joints
        return J_full[:, :5]

    # -------------------------
    # IK control loop
    # -------------------------
    def control_loop(self):
        if self.q is None:
            return

        # Build full joint vector (6th joint fixed)
        q_full = np.append(self.q, 0.0)

        # Current EE position
        T = self.forward_kinematics(q_full)
        x = T[0:3, 3]

        error = self.x_des - x

        if np.linalg.norm(error) < 1e-3:
            return

        J = self.compute_jacobian(q_full)
        dq = self.K * np.linalg.pinv(J) @ error

        q_cmd = self.q + dq

        self.send_joint_command(q_cmd)

        self.get_logger().info(
            f"EE pos: {x.round(3)} | error: {error.round(4)}"
        )

    # -------------------------
    # Send joint command
    # -------------------------
    def send_joint_command(self, q_cmd):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_cmd.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200_000_000  # 0.2 s

        traj.points.append(point)
        self.cmd_pub.publish(traj)


def main():
    rclpy.init()
    node = IKControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
