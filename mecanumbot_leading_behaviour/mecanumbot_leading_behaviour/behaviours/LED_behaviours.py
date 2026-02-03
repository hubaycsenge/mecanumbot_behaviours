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
            key="LED_indicate_close_target_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="LED_indicate_close_target_times", 
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
        
    def initialise(self):
        self.next_send_time = None
        self.pending_future = None
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
            self.node.get_logger().error(f"{self.name}:Unknown LED behaviour mode: {self.mode}")
            return
    
    def update(self):

        if not self.LED_seq or self.delay is None:
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        # Waiting for service response
        if self.pending_future is not None:
            if not self.pending_future.done():
                return py_trees.common.Status.RUNNING
            else:
                # service finished, now wait for delay
                self.pending_future = None
                self.next_send_time = now + rclpy.duration.Duration(seconds=float(self.delay[self.index-1]))
                return py_trees.common.Status.RUNNING

        # Waiting for delay after service
        if self.next_send_time is not None and now < self.next_send_time:
            return py_trees.common.Status.RUNNING

        # All commands done
        if self.index >= len(self.LED_seq):
            return py_trees.common.Status.SUCCESS

        # Send next command
        self.node.get_logger().info(f"{self.name}:CMD: {self.LED_seq[self.index]}")
        self._send_command()
        return py_trees.common.Status.RUNNING


    def _send_command(self):

        cmd = self.LED_seq[self.index]
        

        self.pending_future = self.srv_client.call_async(cmd)

        self.index += 1
        self.next_send_time = None

    