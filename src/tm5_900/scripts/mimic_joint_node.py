#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class MimicJointNode(Node):
    def __init__(self):
        super().__init__('mimic_joint_node')
        
        # Subscribe to joint states to read lefthand_joint position
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish directly to righthand_joint position controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/righthand_joint_controller/commands',
            10
        )

    def joint_state_callback(self, msg: JointState):
        if 'lefthand_joint' in msg.name:
            idx = msg.name.index('lefthand_joint')
            position = msg.position[idx]
            
            cmd = Float64MultiArray()
            cmd.data = [position]  # multiplier=1, offset=0
            self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MimicJointNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()