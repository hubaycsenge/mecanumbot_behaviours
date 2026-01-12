import rclpy
from rclpy.node import Node
import numpy as np
import py_trees
import py_trees_ros

from mecanumbot_msgs.srv  import GetLedStatus, SetLedStatus

class LEDBehaviourSequence(py_trees.behaviour.Behaviour):  # Checks done - works
    def __init__(self, name="LEDBehaviourSeq",mode = "catch_attention"):
        super().__init__(name)

        # Create the ROS publisher
        self.srv_set = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="LED_indicate_target_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="LED_indicate_target_times", 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="LED_catch_attention_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="LED_catch_attention_times", 
            access=py_trees.common.Access.READ
        )
        self.mode = mode
    
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        self.index = 0
        self.srv_client = self.node.create_client(SetLedStatus,'/set_led_status')
        self.next_send_time = None

    def initialise(self):
        if self.mode == "indicate_target":
            self.LED_seq = self.blackboard.LED_indicate_target_seq
            self.delay = self.blackboard.LED_indicate_target_times  # [s]
        elif self.mode == "catch_attention":
            self.LED_seq = self.blackboard.LED_catch_attention_seq
            self.delay = self.blackboard.LED_catch_attention_times  # [s]
        elif self.mode == "indicate_close_target":
            self.LED_seq = self.blackboard.LED_indicate_close_target_seq
            self.delay = self.blackboard.LED_indicate_close_target_times  # [s]
        else:
            self.node.get_logger().error(f"Unknown LED behaviour mode: {self.mode}")
            return
    
    def update(self):
        # Safety checks
        if not self.LED_seq or self.delay is None:
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        # First command
        if self.next_send_time is None:
            self._send_command(now)
            return py_trees.common.Status.RUNNING
        
        # Wait until delay has passed
        if now < self.next_send_time:
            return py_trees.common.Status.RUNNING
        
        elif self.index < len(self.LED_seq): # the commands are being sent out
            self._send_command(now)
            return py_trees.common.Status.RUNNING
        
        elif self.index >= len(self.LED_seq):
            return py_trees.common.Status.SUCCESS


    def _send_command(self, now):

        cmd = self.LED_seq[self.index]
        # Call the service asynchronously
        future = self.srv_client.call_async(cmd)

        # Optional: store future if you want to check result later
        self.last_future = future
        current_delay = rclpy.duration.Duration()
        current_delay.seconds = self.delay[self.index]
        # Compute next time
        self.node.get_logger().info(f"index: {self.index}")
        self.node.get_logger().info(f"LED Command sent: {cmd}")
        #self.node.get_logger().info(f"Waiting for {self.delay[self.index]} seconds until next command.")
        
        self.next_send_time = now + current_delay
        self.node.get_logger().info(f"Next command scheduled at: {self.next_send_time.nanoseconds // 1_000_000_000}.{self.next_send_time.nanoseconds % 1_000_000_000:09d} s, which is in {current_delay.seconds} seconds.")

        self.index += 1

    