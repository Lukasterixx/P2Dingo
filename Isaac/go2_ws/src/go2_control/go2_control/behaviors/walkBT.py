# go2_control/go2_control/behaviors/walk_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import py_trees
from py_trees.common import ParallelPolicy
from py_trees_ros.trees import BehaviourTree

# Import your two leaf behaviours (each defined in its own file)
from go2_control.behaviors.permission_behavior import PermissionBehavior
from go2_control.behaviors.walk_forward_behavior import WalkForwardBehavior


class WalkNode(Node):
    def __init__(self):
        super().__init__('walk_node')

        # --- 1) Create your publisher & Twist message:
        self.cmd_pub = self.create_publisher(Twist, 'robot0/cmd_vel', 10)
        twist_msg = Twist()
        twist_msg.linear.x = 1.0

        # --- 2) Instantiate your two leaf behaviours:
        permission   = PermissionBehavior(name="Permission")
        walk_forward = WalkForwardBehavior(
            name="WalkForward",
            publisher=self.cmd_pub,
            twist_msg=twist_msg
        )

        # --- 3) Build the Parallel root with SuccessOnAll (no synchronisation) ---
        root = py_trees.composites.Parallel(
            name="MainParallel",
            policy=ParallelPolicy.SuccessOnAll(synchronise=False)
        )
        root.add_children([permission, walk_forward])

        # --- 4) Wrap in a py_trees_ros BehaviourTree and start it ticking at 10 Hz ---
        self.tree = BehaviourTree(root=root)
        self.tree.setup(node=self, node_name='walk_node_tree')
        self.tree.tick_tock(period_ms=100)

        self.get_logger().info("WalkNode: Parallel tree started @ 10 Hz")


def main(args=None):
    rclpy.init(args=args)
    node = WalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
