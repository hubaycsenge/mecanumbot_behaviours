"""LED signalling behaviours."""

import py_trees

from mecanumbot_msgs.srv import SetLedStatus

from mecanumbot_leading_behaviour.behaviours.ros_interfaces import duration

LED_SERVICE = "/mecanumbot/set_led_status"

# Signalling mode -> the pair of blackboard keys holding its patterns and timings.
LED_KEYS = {
    "indicate_target": ("LED_indicate_target_seq", "LED_indicate_target_times"),
    "indicate_close_target": (
        "LED_indicate_close_target_seq",
        "LED_indicate_close_target_times",
    ),
    "catch_attention": ("LED_catch_attention_seq", "LED_catch_attention_times"),
    "thank": ("LED_thank_seq", "LED_thank_times"),
}


class LEDBehaviourSequence(py_trees.behaviour.Behaviour):
    """Play one timed LED pattern sequence from the blackboard.

    Each pattern is a `SetLedStatus` service call; the next one is only sent
    after the previous call returned and its hold time elapsed.
    """

    def __init__(self, name="LEDBehaviourSeq", mode="catch_attention"):
        super().__init__(name)
        if mode not in LED_KEYS:
            raise ValueError(
                f"unknown LED behaviour mode '{mode}', expected one of {sorted(LED_KEYS)}"
            )
        self.mode = mode
        self.seq_key, self.times_key = LED_KEYS[mode]

        self.blackboard = self.attach_blackboard_client(name=name)
        for seq_key, times_key in LED_KEYS.values():
            self.blackboard.register_key(
                key=seq_key, access=py_trees.common.Access.READ
            )
            self.blackboard.register_key(
                key=times_key, access=py_trees.common.Access.READ
            )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.index = 0
        self.next_send_time = None
        self.pending_call = None
        self.patterns = getattr(self.blackboard, self.seq_key)
        self.delays = getattr(self.blackboard, self.times_key)

    def update(self):
        if not self.patterns or not self.delays:
            self.feedback_message = f"no patterns configured for '{self.mode}'"
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        if self.pending_call is not None:
            if not self.pending_call.done():
                self.feedback_message = "waiting for the LED service"
                return py_trees.common.Status.RUNNING
            self.pending_call = None
            self.next_send_time = now + duration(self.delays[self.index - 1])
            return py_trees.common.Status.RUNNING

        if self.next_send_time is not None and now < self.next_send_time:
            self.feedback_message = f"step {self.index}/{len(self.patterns)}"
            return py_trees.common.Status.RUNNING

        if self.index >= len(self.patterns):
            self.node.get_logger().info(
                f"{self.name}: LED pattern '{self.mode}' completed"
            )
            return py_trees.common.Status.SUCCESS

        self.pending_call = self.led_client.call_async(self.patterns[self.index])
        self.index += 1
        self.next_send_time = None
        return py_trees.common.Status.RUNNING
