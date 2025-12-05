import rclpy
from rclpy.node import Node
import numpy as np
import py_trees
import py_trees_ros

from mecanumbot_msgs.srv  import GetLedStatus, SetLedStatus

class LED_behaviour_seq(py_trees.behaviour.Behaviour):  
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

        self.srv_set = self.node.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback)

    def initialise(self):
        if self.mode == "indicate_target":
            self.LED_seq = self.blackboard.LED_indicate_target_seq
            self.delay = self.blackboard.LED_indicate_target_times  # [s]
        elif self.mode == "catch_attention":
            self.LED_seq = self.blackboard.LED_catch_attention_seq
            self.delay = self.blackboard.LED_catch_attention_times  # [s]
        else:
            self.node.get_logger().error(f"Unknown LED behaviour mode: {self.mode}")
            return
        
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
        self.publisher.publish(cmd)

        # Compute next time
        self.next_send_time = now + rclpy.time.Time([self.delay[self.index], 0])

        self.index += 1

    