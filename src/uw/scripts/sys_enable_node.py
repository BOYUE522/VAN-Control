#!/usr/bin/python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Empty
from ds_dbw_msgs.msg import MiscReport


class SysEnableNode(Node):
    def __init__(self) -> None:
        super().__init__('sys_enable_node')

        latch_qos = QoSProfile(depth=1)
        latch_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        default_sys_enabled = True
        self.pub_sys_enabled = self.create_publisher(Bool, '/sys_enabled', latch_qos)
        self.pub_dbw_enable = self.create_publisher(Empty, '/vehicle/enable', 1)
        self.pub_sys_enabled.publish(Bool(data=default_sys_enabled))
        self.sub_misc_report = self.create_subscription(MiscReport, '/vehicle/misc/report', self.recv_misc_report, 1)

        # Parameters
        self.declare_parameter('enable_sys', default_sys_enabled)
        self.add_on_set_parameters_callback(self.param_change_cb)

    def recv_misc_report(self, msg: MiscReport):
        if msg.btn_ld_ok:
            self.pub_dbw_enable.publish(Empty())

    def param_change_cb(self, params):
        for p in params:
            if p.name == 'enable_sys':
                self.pub_sys_enabled.publish(Bool(data=p.value))

        return SetParametersResult(successful=True)


if __name__ == '__main__':
    rclpy.init()
    node_instance = SysEnableNode()
    rclpy.spin(node_instance)
