import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')

        self.q = None

        self.print_timer = self.create_timer(1.0, self.print_joint_angles)


        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info("FK node started")


    def print_joint_angles(self):
        if self.q is None or len(self.q) < 6:
            return

        q = self.q[:6]
        q_deg = np.degrees(q).round(2)

        # Forward kinematics
        T = self.forward_kinematics(q)
        pos = T[0:3, 3].round(3)

        self.get_logger().info(
            f"q (deg): {q_deg} | EE pos (m): x={pos[0]}, y={pos[1]}, z={pos[2]}"
        )


    def joint_callback(self, msg):
        self.q = np.array(msg.position)

        # if len(self.q) < 6:
        #     return

        # T = self.forward_kinematics(self.q[:6])
        # pos = T[0:3, 3]

        # self.get_logger().info(
        #     f"End Effector Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}"
        # )

    def dh(self, a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    def forward_kinematics(self, q):
        a = [0, 0, 0.1104, 0.096, 0, 0]
        alpha = [np.pi/2, np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2]
        d = [0.13156, 0, 0, 0.06062, 0.07318, 0.0456]

        theta_offsets = [np.pi/2, -np.pi/2, 0, 0, 0, 0]

        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh(a[i], alpha[i], d[i], q[i] + theta_offsets[i])

        return T


def main():
    rclpy.init()
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
