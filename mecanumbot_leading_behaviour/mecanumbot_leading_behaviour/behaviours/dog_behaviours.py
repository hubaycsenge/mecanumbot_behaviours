from mecanumbot_msgs.msg import AccessMotorCmd
import rclpy
from rclpy.node import Node
import numpy as np
import py_trees
import py_trees_ros



class DogIndicateTarget(py_trees.behaviour.Behaviour):

    def __init__(self, name="DogIndicateTarget"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="dog_indicatetarget_behaviour_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="dog_indicatetarget_behaviour_dt", 
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        self.publisher = node.create_publisher(
            AccessMotorCmd,
            "cmd_accessory_pos",
            10
        )

    def initialise(self):
        self.behaviour_seq = self.blackboard.dog_indicatetarget_behaviour_seq
        self.delay = self.blackboard.dog_indicatetarget_behaviour_dt  # in milliseconds
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

        # Send next command
        self._send_command(now)

        # Finished sequence
        if self.index >= len(self.behaviour_seq):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def _send_command(self, now):
        cmd = self.behaviour_seq[self.index]
        self.publisher.publish(cmd)

        # Compute next time
        self.next_send_time = now + Duration(milliseconds=int(self.delay))

        self.index += 1

class DogCatchAttention(py_trees.behaviour.Behaviour):

    def __init__(self, name="DogCatchAttention"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="dog_attention_behaviour_seq", # list of 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="dog_attention_behaviour_dt", 
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        self.publisher = node.create_publisher(
            AccessMotorCmd,
            "cmd_accessory_pos",
            10
        )

    def initialise(self):
        self.behaviour_seq = self.blackboard.dog_attention_behaviour_seq
        self.delay = self.blackboard.dog_attention_behaviour_dt  # in milliseconds
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

        # Send next command
        self._send_command(now)

        # Finished sequence
        if self.index >= len(self.behaviour_seq):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def _send_command(self, now):
        cmd = self.behaviour_seq[self.index]
        self.publisher.publish(cmd)

        # Compute next time
        self.next_send_time = now + Duration(milliseconds=int(self.delay))

        self.index += 1

class DogLookForFeedback(py_trees.behaviour.Behaviour):

    def __init__(self, name="DogLookForFeedback"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="dog_last_target_subject_distance", # [m between target and subject] 
            access=py_trees.common.Access.READWRITE
        )
        self.blackboard.register_key(
            key="target_subject_distance", # [m] 
            access=py_trees.common.Access.READ
        )

        self.blackboard.register_key(
            key="dog_leading_status" # reached/leading/lost
            access=py_trees.common.Access.WRITE
        )

        self.blackboard.register_key(
            key = "target_reached_within_threshold", # True/False
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]
        self.dog_last_target_subject_distance = None
        self.target_subject_distance = self.blackboard.target_subject_distance
        self.logger.info("Turn command publisher ready")
    
    def update(self):
        """
        Read target-subject distance from blackboard and decide if feedback is needed.
        """

        target_subject_distance = self.blackboard.target_subject_distance

        self.blackboard.dog_last_target_subject_distance = target_subject_distance
        self.target_reached_within_threshold = self.blackboard.target_reached_within_threshold

        if target_subject_distance is None:
            self.logger.warn("No target-subject distance on blackboard yet")
            return py_trees.common.Status.RUNNING
        else:
            if self.target_reached_within_threshold:
                self.blackboard.dog_leading_status = "reached"
            else:
                if self.dog_last_target_subject_distance is None:
                    self.dog_last_target_subject_distance = target_subject_distance
                    self.blackboard.dog_last_target_subject_distance = target_subject_distance
                    self.blackboard.dog_leading_status = "leading"
                    return py_trees.common.Status.SUCCESS
                elif target_subject_distance - self.dog_last_target_subject_distance  < 0.10: #ha legfeljebb 10 cm-rel kerult messzebb
                    self.blackboard.dog_last_target_subject_distance = target_subject_distance
                    self.blackboard.dog_leading_status = "leading"
                else:
                    self.blackboard.dog_leading_status = "lost"
                
            return py_trees.common.Status.SUCCESS 

        return py_trees.common.Status.RUNNING

class DogCheckSubjectTargetSuccess(py_trees.behaviour.Behaviour):

    def __init__(self, name="CheckSubjectTargetSuccess"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="dog_leading_status", # True/False 
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]

    
    def update(self):
        """
        Read target_reached_within_threshold from blackboard and decide if target is reached.
        """

        leading_status = self.blackboard.dog_leading_status

        if leading_status is None:
            self.logger.warn("No  leading status on blackboard yet")
            return py_trees.common.Status.RUNNING
        else:
            if leading_status == "reached":
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

class DogCheckIfSubjLed(py_trees.behaviour.Behaviour):

    def __init__(self, name="CheckSubjectTargetSuccess"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="dog_leading_status", # True/False 
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]

    
    def update(self):
        """
        Read target_reached_within_threshold from blackboard and decide if target is reached.
        """

        leading_status = self.blackboard.dog_leading_status

        if leading_status is None:
            self.logger.warn("No  leading status on blackboard yet")
            return py_trees.common.Status.RUNNING
        else:
            if leading_status == "leading":
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE