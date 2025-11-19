import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

from swarm_logic import update_swarm

class SwarmController(Node):
    def __init__(self):
        super().__init__("swarm_controller")
        default_uavs = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5"]
        self.declare_parameter("uav_names", default_uavs)
        self.uav_names = self.get_parameter("uav_names").value

        self.odom = {name: None for name in self.uav_names}
        self.cmd_pubs = {}
        self.odom_subs = []

        for name in self.uav_names:
            self.cmd_pubs[name] = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            sub = self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda msg, n=name: self.odom_callback(msg, n), 10)
            self.odom_subs.append(sub)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Agent states
        N = len(self.uav_names)
        self.positions = np.zeros((N, 2))
        self.velocities = np.zeros((N, 2))
        self.leader_pos = np.array([0.0, 0.0])
        self.leader_vel = np.array([0.5, 0.0])

    def odom_callback(self, msg: Odometry, uav_name: str):
        idx = self.uav_names.index(uav_name)
        self.positions[idx, 0] = msg.pose.pose.position.x
        self.positions[idx, 1] = msg.pose.pose.position.y
        # You could store z if needed for altitude control
        # velocities could also be extracted from msg.twist

    def control_loop(self):
        # Update swarm positions and velocities
        self.positions, self.velocities = update_swarm(
            self.positions, self.velocities,
            self.leader_pos, self.leader_vel
        )

        # Publish velocities as Twist messages
        for i, name in enumerate(self.uav_names):
            cmd = Twist()
            cmd.linear.x = self.velocities[i, 0]
            cmd.linear.y = self.velocities[i, 1]
            cmd.linear.z = 0.0  # optionally add altitude control
            cmd.angular.z = 0.0
            self.cmd_pubs[name].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down swarm_controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()
