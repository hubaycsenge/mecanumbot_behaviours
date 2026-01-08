from mecanumbot_msgs.msg import AccessMotorCmd
import rclpy
from rclpy.node import Node
import numpy as np
import py_trees
import py_trees_ros



class DogBehaviourSequence(py_trees.behaviour.Behaviour): # Checks done - works

    def __init__(self, name="DogBehaviourSequence",mode = "indicate_target"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="Dog_indicate_target_seq", # list of motor commands
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="Dog_indicate_target_times", 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="Dog_catch_attention_seq", # list of motor commands
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="Dog_catch_attention_times", 
            access=py_trees.common.Access.READ
        )

        self.mode = mode
    
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        self.publisher = node.create_publisher(
            AccessMotorCmd,
            "cmd_accessory_pos",
            10
        )
        self.index = 0
        self.next_send_time = None
        

    def initialise(self):
        self.index = 0
        self.next_send_time = None
        if self.mode not in ["indicate_target", "catch_attention"]:
            self.node.get_logger().error(f"Unknown Dog behaviour mode: {self.mode}")
        elif self.mode == "indicate_target":
            self.behaviour_seq = self.blackboard.Dog_indicate_target_seq
            self.delay = self.blackboard.Dog_indicate_target_times  # in senconds
            #self.node.get_logger().info("Dog Indicate Target behaviour publisher ready")
        elif self.mode == "catch_attention":
            self.behaviour_seq = self.blackboard.Dog_catch_attention_seq
            self.delay = self.blackboard.Dog_catch_attention_times  # in senconds
            #self.node.get_logger().info("Dog Catch Attention behaviour publisher ready")
        
   
    def update(self):
        # Safety checks
        if not self.behaviour_seq or self.delay is None:
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        # First command
        if self.next_send_time is None:
            self._send_command(now)
            self.node.get_logger().info(f"Dog behaviour {self.mode} started")
            return py_trees.common.Status.RUNNING
        
        # Wait until delay has passed
        if now < self.next_send_time:
            #self.node.get_logger().info(f"Dog behaviour {self.mode} waiting...")
            return py_trees.common.Status.RUNNING
        
        elif self.index < len(self.behaviour_seq): # the commands are being sent out
            self._send_command(now)
            self.node.get_logger().info(f"Dog behaviour {self.mode} command {self.index} sent")
            return py_trees.common.Status.RUNNING
        
        elif self.index >= len(self.behaviour_seq):
            self.node.get_logger().info(f"Dog behaviour {self.mode} completed")
            return py_trees.common.Status.SUCCESS
        
    def _send_command(self, now):
        cmd = self.behaviour_seq[self.index]
        self.publisher.publish(cmd)

        # Compute next time
        delay_sec =  float(self.delay[self.index])
        self.next_send_time = now + rclpy.duration.Duration(seconds=delay_sec)
        self.index += 1

class DogLookForFeedback(py_trees.behaviour.Behaviour): #TODO

    def __init__(self, name="DogLookForFeedback"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(
            key="target_within_subject_threshold", # bool
            access=py_trees.common.Access.READ
        )        
        self.blackboard.register_key(
            key = "target_distance_from_subject", # True/False
            access=py_trees.common.Access.READ
        )

        self.blackboard.register_key(
            key="dog_leading_status", # reached/leading/lost
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="dog_last_target_distance_from_subject", # float
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="Dog_following_min_threshold", # [m]
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]
        self.target_within_subject_threshold = self.blackboard.target_reached_within_threshold
        self.target_distance_from_subject = self.blackboard.target_distance_from_subject
        self.logger.info("Turn command publisher ready")
    
    def update(self):
        """
        Read target-subject distance from blackboard and decide if feedback is needed.
        """
        self.target_within_subject_threshold = self.blackboard.target_reached_within_threshold
        self.target_distance_from_subject = self.blackboard.target_distance_from_subject

        if self.target_within_subject_threshold is None or self.target_distance_from_subject is None:
            self.logger.warn("No target-subject distance on blackboard yet")
            return py_trees.common.Status.FAILURE
        else:
            if self.target_within_subject_threshold: # If the suject reached the target
                self.blackboard.dog_last_target_distance_from_subject
                self.blackboard.dog_leading_status = "reached"
            else:
                if self.blackboard.dog_last_target_distance_from_subject is None: #first time checking
                    self.blackboard.dog_last_target_distance_from_subject = self.target_distance_from_subject
                    self.blackboard.dog_leading_status = "leading"
                elif self.target_distance_from_subject - self.blackboard.dog_last_target_distance_from_subject  > self.blackboard.Dog_following_min_threshold: # subject going away
                    self.blackboard.dog_last_target_distance_from_subject = self.target_distance_from_subject
                    self.blackboard.dog_leading_status = "lost"
                else: # subject following
                    self.blackboard.dog_last_target_distance_from_subject = self.target_distance_from_subject
                    self.blackboard.dog_leading_status = "leading"

        return py_trees.common.Status.SUCCESS
    
class DogCheckSubjectTargetSuccess(py_trees.behaviour.Behaviour): #TODO

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
            return py_trees.common.Status.FAILURE
        else:
            if leading_status == "reached":
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

class DogCheckIfSubjLed(py_trees.behaviour.Behaviour): #TODO

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
            self.logger.warn("No leading status on blackboard yet")
            return py_trees.common.Status.RUNNING
        else:
            if leading_status == "leading":
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE