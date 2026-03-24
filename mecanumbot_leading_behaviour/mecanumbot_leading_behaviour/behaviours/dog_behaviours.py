from mecanumbot_msgs.msg import AccessMotorCmd
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import numpy as np
import py_trees
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Point
import math


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
            #self.node.get_logger().info(f"Dog behaviour {self.mode} command {self.index} sent")
            return py_trees.common.Status.RUNNING
        
        elif self.index >= len(self.behaviour_seq):
            self.index = 0
            self.node.get_logger().info(f"Dog behaviour {self.mode} completed")
            return py_trees.common.Status.SUCCESS
        
    def _send_command(self, now):
        cmd = self.behaviour_seq[self.index]
        self.publisher.publish(cmd)

        # Compute next time
        delay_sec =  float(self.delay[self.index])
        self.next_send_time = now + rclpy.duration.Duration(seconds=delay_sec)
        self.index += 1

class DogCheckFollowing(py_trees.behaviour.Behaviour): # TODO

    def __init__(self, name="DogCheckFollowing"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key="Dog_following_max_threshold", 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="Dog_max_wander_allowed", 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="last_distance",
            access = py_trees.common.Access.WRITE
        )
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        self.subject_subscriber = node.create_subscription(
            PoseStamped,
            "/mecanumbot/subject_pose",
            self.subject_callback,
            10
        )
        self.wanders = 0

    def initialise(self):
        self.current_distance = None
    
    def update(self):
        if self.current_distance is None:
            return py_trees.common.Status.RUNNING
        distance = self.current_distance
        last_distance = self.blackboard.last_distance

        distance_diff = distance - last_distance

        if distance is None:
            #self.node.get_logger().info("DogCheckFollowing: No distance data yet")
            return py_trees.common.Status.RUNNING
        if distance_diff >= 0:
            self.wanders += 1
        if self.wanders > self.blackboard.Dog_max_wander_allowed:
            self.node.get_logger().info(f"{self.name}: Subject wandered too much, lost")
            self.wanders = 0
            return py_trees.common.Status.FAILURE
        elif distance_diff > self.blackboard.Dog_following_max_threshold:
            self.node.get_logger().info(f"{self.name}: Subject too far, distance increased by {distance_diff:.2f} m")
            self.blackboard.last_distance = distance
            return py_trees.common.Status.FAILURE
        elif distance_diff <= self.blackboard.Dog_following_max_threshold:
            self.node.get_logger().info(f"{self.name}: Following OK, distance change {distance_diff:.2f} m")
            self.blackboard.last_distance = distance
            return py_trees.common.Status.SUCCESS
    
    def subject_callback(self, msg):
        subject_pos = msg.pose.position
        target_pos = self.blackboard.target_position

        dist = np.sqrt(
            (subject_pos.x - target_pos.x) ** 2 +
            (subject_pos.y - target_pos.y) ** 2
        )

        # Update last distance
        self.current_distance = dist

class DogSelectTarget(py_trees.behaviour.Behaviour): #TODO
    def __init__(self, name="DogCheckFollowing"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_checkpoints",access=py_trees.common.Access.READ)
        self.blackboard.register_key("Dog_current_checkpoint",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_max_checkpoint",access=py_trees.common.Access.READ)
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )
    
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback to store the latest robot pose.
        We only need the Pose component (which contains the position Point).
        """
        self.robot_position = msg.pose.pose.position # geometry_msgs/Point

    def initialise(self):
        self.current_distance = None
    
    def update(self):
        robot_position = self.robot_position
        if robot_position is None:
            #self.feedback_message = "waiting for initial robot pose"
            #self.node.get_logger().info(self.feedback_message)
            return py_trees.common.Status.RUNNING
        self.blackboard.Dog_current_checkpoint = self.select_closest_ckpt(robot_position, self.blackboard.Dog_checkpoints)
        self.node.get_logger().info(f'Current checkpoint set to {self.blackboard.Dog_current_checkpoint}')
        return py_trees.common.Status.SUCCESS
    
    def select_closest_ckpt(self,robot_position,checkpoints):
        min_distance = float("inf")
        closest_index = -1

        rx = robot_position.x
        ry = robot_position.y
        rz = robot_position.z

        for i, point in enumerate(checkpoints):
            dx = point.x - rx
            dy = point.y - ry
            dz = point.z - rz

            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

            if distance < min_distance:
                min_distance = distance
                closest_index = i

        if closest_index != self.blackboard.Dog_max_checkpoint:
                closest_index += 1
        return closest_index
        
        
    