from mecanumbot_msgs.msg import 
import rclpy
from rclpy.node import Node
import numpy as np
import py_trees
import py_trees_ros

from mecanumbot_msgs.srv  import GetLedStatus, SetLedStatus

class LEDIndicateTarget(py_trees.behaviour.Behaviour):

    def __init__(self, name="LEDIndicateTarget"):
        super().__init__(name)

        # Create the ROS publisher
        self.srv_set = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="led_indicatetarget_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="led_indicatetarget_dt", 
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        self.srv_set = self.node.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback)

    def initialise(self):
        self.LED_seq = self.blackboard.led_indicatetarget_seq
        self.delay = self.blackboard.led_indicatetarget_dt  # in milliseconds
        self.index = 0
        self.next_send_time = None

    
    def update(self):
        # Safety checks
        if not self.behaviour_seq or self.delay is None:
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        # First command
        if self.next_send_time is None:
            self._send_command(now)
            return py_trees.common.Status.RUNNING

        # Wait until delay has passed
        if now < self.next_send_time:
            return py_trees.common.Status.RUNNING

    def _send_command(self, now):
        cmd = self.LED_seq[self.index]
        request = SetLedStatus.Request()
        (request.fl_mode, request.fl_color,
         request.fr_mode, request.fr_color,
         request.br_mode, request.br_color,
         request.bl_mode, request.bl_color) = cmd

        # Call the service
        future = self.srv_set.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"LED command sent: {cmd}")
        else:
            self.node.get_logger().error(f"Failed to call service set_led_status")

        # Schedule next send time
        self.next_send_time = now + rclpy.duration.Duration(milliseconds=self.delay)

        # Move to next command in sequence
        self.index = (self.index + 1) % len(self.LED_seq)
    
class LEDCatchAttention(py_trees.behaviour.Behaviour):

    def __init__(self, name="LEDCatchAttention"):
        super().__init__(name)

        # Create the ROS publisher
        self.srv_set = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="led_attention_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="led_attention_dt", 
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        self.srv_set = self.node.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback)

    def initialise(self):
        self.LED_seq = self.blackboard.led_attention_seq
        self.delay = self.blackboard.led_attention_dt  # in milliseconds
        self.index = 0
        self.next_send_time = None

    
    def update(self):
        # Safety checks
        if not self.behaviour_seq or self.delay is None:
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        # First command
        if self.next_send_time is None:
            self._send_command(now)
            return py_trees.common.Status.RUNNING

        # Wait until delay has passed
        if now < self.next_send_time:
            return py_trees.common.Status.RUNNING

    def _send_command(self, now):
        cmd = self.LED_seq[self.index]
        request = SetLedStatus.Request()
        (request.fl_mode, request.fl_color,
         request.fr_mode, request.fr_color,
         request.br_mode, request.br_color,
         request.bl_mode, request.bl_color) = cmd

        # Call the service
        future = self.srv_set.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"LED command sent: {cmd}")
        else:
            self.node.get_logger().error(f"Failed to call service set_led_status")

        # Schedule next send time
        self.next_send_time = now + rclpy.duration.Duration(milliseconds=self.delay)

        # Move to next command in sequence
        self.index = (self.index + 1) % len(self.led_attention_seq)

class LEDStop(py_trees.behaviour.Behaviour):

    def __init__(self, name="LEDStop"):
        super().__init__(name)

        # Create the ROS publisher
        self.srv_set = None

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        self.srv_set = self.node.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback)

    def initialise(self):
        pass

    
    def update(self):
        request = SetLedStatus.Request()
        (request.fl_mode, request.fl_color,
         request.fr_mode, request.fr_color,
         request.br_mode, request.br_color,
         request.bl_mode, request.bl_color) = (0, 0, 0, 0, 0, 0, 0, 0)  # All OFF

        # Call the service
        future = self.srv_set.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"LED STOP command sent")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(f"Failed to call service set_led_status")
            return py_trees.common.Status.FAILURE