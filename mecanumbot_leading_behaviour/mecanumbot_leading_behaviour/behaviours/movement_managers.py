import math
import rclpy
import py_trees
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, PoseStamped,PoseWithCovarianceStamped,Pose
import numpy as np
from mecanumbot_msgs.msg import AccessMotorCmd
#from tf_transformations import quaternion_from_euler, euler_from_quaternion
from transforms3d.euler import quat2euler, euler2quat
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray, GoalStatus


#nav2goal statuses
STATUS_UNKNOWN=0
STATUS_ACCEPTED=1
STATUS_EXECUTING=2
STATUS_CANCELING=3
STATUS_SUCCEEDED=4
STATUS_CANCELED=5
STATUS_ABORTED=6


class TurnToward(py_trees.behaviour.Behaviour): # Tested, works

    def __init__(self, name="TurnToward", target_type="subject"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None
        self.subject_pose = None
        self.robot_pose = None
        
        # Blackboard
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="start_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key("Dog_current_checkpoint",access=py_trees.common.Access.READ)
        self.blackboard.register_key("Dog_checkpoints",access=py_trees.common.Access.WRITE)

        self.turning = False
        self.target_type = target_type
    
        self.cmd_send_time = None 

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)

        if self.target_type == "subject":
            self.subject_subscriber = node.create_subscription(
                PoseArray,
                "/mecanumbot/people_fusion",
                self.subject_callback,
                10
            )

        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.goal_sent = False
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goals_in_sys = None
        self.compare_position = None
        return super().initialise()

    def update(self):
        # Safety Checks
        #self.node.get_logger().info(f"{self.name}: Update called")
        if self.robot_pose is None:
            return py_trees.common.Status.RUNNING
        if self.compare_position is None:
            if self.target_type == "subject":
                if self.subject_pose is None:
                    return py_trees.common.Status.RUNNING
                self.compare_position = self.subject_pose.position
            elif self.target_type == "start":
                self.compare_position = self.blackboard.start_position
            elif self.target_type == "checkpoint":
                self.compare_position = self.blackboard.Dog_checkpoints[self.blackboard.Dog_current_checkpoint]
            else:
                self.compare_position = self.blackboard.target_position
        if self.compare_position is None:
            return py_trees.common.Status.FAILURE

        if not self.goal_sent:
            if self.goals_in_sys is not None and len(self.goals_in_sys) >= 0: # robot goal status array has history
                robot_has_goal_running = self.check_if_running()
                if not robot_has_goal_running:
                    self.send_turn_command()
                    return py_trees.common.Status.RUNNING
                else:
                    self.node.get_logger().info(f"{self.name}: Waiting for previous goal to finish")
                    return py_trees.common.Status.RUNNING
            else:
                self.send_turn_command()
                return py_trees.common.Status.RUNNING
        else:
            if self.goal_uuid is None:
                self.assign_goal_uuid()
            if self.goal_uuid is None:
                self.node.get_logger().info(f"{self.name}: No goal UUID assigned yet")
                if self.node.get_clock().now() - self.cmd_send_time > rclpy.duration.Duration(seconds=3.0):
                    self.node.get_logger().info(f"{self.name}: Timeout waiting for goal UUID")
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING
            for goal in self.goals_in_sys:
                #self.node.get_logger().info(f"{self.name}: Checking goal UUID {goal.goal_info.goal_id.uuid} against {self.goal_uuid}")
                if np.array_equal(goal.goal_info.goal_id.uuid, self.goal_uuid):
                    self.goal_status = goal.status
            if self.goal_status == STATUS_SUCCEEDED:
                self.node.get_logger().info(f"{self.name}: Turn completed successfully")
                return py_trees.common.Status.SUCCESS
            elif self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
                #self.node.get_logger().info(f"{self.name}: Turn in progress")
                return py_trees.common.Status.RUNNING
            elif self.goal_status in [STATUS_ABORTED, STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN]:
                self.node.get_logger().info(f"{self.name}: Turn failed, retrying")
                self.goal_sent = False
                self.goal_uuid = None
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING

    def assign_goal_uuid(self):
        """
        Assign the UUID of the last sent goal.
        FIX: Iterate BACKWARDS to find the NEWEST goal that matches the time.
        """
        if self.goals_in_sys is None:
            return

        # Iterate in reverse (newest first)
        for goal_status in reversed(self.goals_in_sys):
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            cmd_time = Time.from_msg(self.cmd_send_time.to_msg())
            
            # Check if this goal was created AFTER or AT the same time we sent the command
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                self.node.get_logger().info(f"{self.name}: Locked onto Goal UUID {self.goal_uuid}")
                return

    def check_if_running(self):
        """Check if there is a goal currently being executed by nav2."""
        if self.goals_in_sys is None:
            return False
            
        for goal_status in self.goals_in_sys:
            if goal_status.status == STATUS_EXECUTING:
                return True
        return False
    
    def send_turn_command(self):
        """Helper to create and publish the command with fresh timestamp"""
        desired_orientation = calculate_facing_orientation(self.robot_pose, self.compare_position)
        
        self.turn_cmd = PoseStamped()
        self.turn_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        self.turn_cmd.header.stamp = self.cmd_send_time.to_msg() 
        self.turn_cmd.pose.position = self.robot_pose.position
        self.turn_cmd.pose.orientation = desired_orientation
        
        self.publisher.publish(self.turn_cmd)
        
        self.goal_sent = True
        self.node.get_logger().info(f"{self.name}: Published turn command \n Directions: Z: {desired_orientation.z} W: {desired_orientation.w}")

    def goal_status_callback(self, msg):
        if len(msg.status_list) > 5:
            self.goals_in_sys = msg.status_list[-5:]
        else:
            self.goals_in_sys = msg.status_list
        
        # Optional: reduce logging verbosity to prevent console lag
        # self.logger.debug(f"{self.name}: Updated goal status list")
        
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def subject_callback(self, msg):
        self.subject_pose = msg.poses[0]

class RelativeTurnPattern(py_trees.behaviour.Behaviour):
    """Perform a small turn pattern: left, right, right, back to start heading."""

    def __init__(self, name="RelativeTurnPattern", step_angle_deg=20.0):
        super().__init__(name)
        self.publisher = None
        self.robot_pose = None
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goals_in_sys = []
        self.goal_sent = False
        self.turn_offsets = [
            math.radians(step_angle_deg),
            -math.radians(2*step_angle_deg),
            math.radians(2*step_angle_deg),
            -math.radians(step_angle_deg),
            
        ]

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)

        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10
        )

    def initialise(self):
        self.phase_index = 0
        self.goal_sent = False
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goal_status = STATUS_UNKNOWN

    def update(self):
        if self.robot_pose is None:
            return py_trees.common.Status.RUNNING

        if self.phase_index >= len(self.turn_offsets):
            self.node.get_logger().info(f"{self.name}: Turn pattern completed")
            return py_trees.common.Status.SUCCESS

        if not self.goal_sent:
            if self.check_if_running():
                return py_trees.common.Status.RUNNING
            self.send_relative_turn(self.turn_offsets[self.phase_index])
            return py_trees.common.Status.RUNNING

        if self.goal_uuid is None:
            self.assign_goal_uuid()
        if self.goal_uuid is None:
            return py_trees.common.Status.RUNNING

        self.goal_status = STATUS_UNKNOWN
        for goal in self.goals_in_sys:
            if np.array_equal(goal.goal_info.goal_id.uuid, self.goal_uuid):
                self.goal_status = goal.status
                break

        if self.goal_status == STATUS_SUCCEEDED:
            self.phase_index += 1
            self.goal_sent = False
            self.goal_uuid = None
            self.cmd_send_time = None
            return py_trees.common.Status.RUNNING
        if self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
            return py_trees.common.Status.RUNNING
        if self.goal_status in [STATUS_ABORTED, STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN]:
            self.node.get_logger().info(f"{self.name}: Turn phase failed, retrying phase {self.phase_index}")
            self.goal_sent = False
            self.goal_uuid = None
            self.cmd_send_time = None
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def send_relative_turn(self, yaw_offset):
        current_yaw = yaw_from_quaternion(self.robot_pose.orientation)
        desired_yaw = normalize_angle(current_yaw + yaw_offset)
        q = euler2quat(0, 0, desired_yaw)

        turn_cmd = PoseStamped()
        turn_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        turn_cmd.header.stamp = self.cmd_send_time.to_msg()
        turn_cmd.pose.position = self.robot_pose.position
        turn_cmd.pose.orientation.w = q[0]
        turn_cmd.pose.orientation.x = q[1]
        turn_cmd.pose.orientation.y = q[2]
        turn_cmd.pose.orientation.z = q[3]

        self.publisher.publish(turn_cmd)
        self.goal_sent = True
        self.node.get_logger().info(f"{self.name}: Sent turn phase {self.phase_index + 1}/{len(self.turn_offsets)}")

    def assign_goal_uuid(self):
        if self.goals_in_sys is None or self.cmd_send_time is None:
            return

        cmd_time = Time.from_msg(self.cmd_send_time.to_msg())
        for goal_status in reversed(self.goals_in_sys):
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                return

    def check_if_running(self):
        if self.goals_in_sys is None:
            return False

        for goal_status in self.goals_in_sys:
            if goal_status.status == STATUS_EXECUTING:
                return True
        return False

    def goal_status_callback(self, msg):
        if len(msg.status_list) > 5:
            self.goals_in_sys = msg.status_list[-5:]
        else:
            self.goals_in_sys = msg.status_list

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

class FindPeople(py_trees.behaviour.Behaviour):
    """Spin in place until a fresh people_fusion message is received."""

    def __init__(self, name="FindPeople", spin_speed=0.5, sight_timeout=1.0):
        super().__init__(name)
        self.spin_speed = float(spin_speed)
        self.sight_timeout = float(sight_timeout)
        self.publisher = None
        self.people_poses = []
        self.last_people_seen_time = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.people_subscriber = self.node.create_subscription(
            PoseArray,
            "/mecanumbot/people_fusion",
            self.people_callback,
            10,
        )
        self.logger.info(f"{self.name}: Setup complete")
        self.first_update = True
        self.first_update_time = None
        self.spin_time_threshold = 10.0

    def has_fresh_people(self):
        if self.last_people_seen_time is None or len(self.people_poses) == 0:
            return False

        now = self.node.get_clock().now()
        age = (now - self.last_people_seen_time).nanoseconds / 1e9
        return age <= self.sight_timeout

    def publish_spin(self):
        cmd = Twist()
        cmd.angular.z = self.spin_speed
        self.publisher.publish(cmd)

    def stop_robot(self):
        self.publisher.publish(Twist())

    def update(self):
        
        now = self.node.get_clock().now()
        #self.node.get_logger().info(f"{self.name}: updating, dt = {(now - self.first_update_time).nanoseconds*10**-9 if self.first_update_time else 'N/A'} seconds")
        if self.has_fresh_people():
            self.stop_robot()
            self.first_update = True
            self.node.get_logger().info(f"{self.name}: People detected, stopping spin")
            return py_trees.common.Status.SUCCESS

        self.publish_spin()
        
        if self.first_update == True:
            self.node.get_logger().info(f"{self.name}: first update")
            self.first_update_time = now
            self.first_update = False
        else:
            if (now - self.first_update_time).nanoseconds*10**-9 > self.spin_time_threshold:
                self.node.get_logger().info(f"{self.name}: time threshold over, no person found")
                self.first_update = True
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def people_callback(self, msg):
        self.people_poses = list(msg.poses)
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            self.last_people_seen_time = Time.from_msg(msg.header.stamp)
        else:
            self.last_people_seen_time = self.node.get_clock().now()
###### NEW CLASSES TO TEST ########################################x
class WaitForPerson(py_trees.behaviour.Behaviour):
    """Passively returns SUCCESS if a person is recently visible, otherwise RUNNING.
       Dynamically updates search_direction based on the person's position relative to path checkpoints."""
    def __init__(self, name="WaitForPerson", sight_timeout=1.0):
        super().__init__(name)
        self.sight_timeout = sight_timeout
        self.last_seen_time = None
        self.last_seen_person_pose = None
        self.robot_pose = None
        self.accessory_publisher = None
        self.begin_wait_sent = False
        
        self.blackboard = self.attach_blackboard_client(name=name)
        # CHANGED: search_direction must be WRITE access to update it dynamically
        self.blackboard.register_key("search_direction", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_current_checkpoint", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("patrol_current_checkpoint", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_max_checkpoint", access=py_trees.common.Access.READ)
        # NEW: Read checkpoints to compare distances
        self.blackboard.register_key("Dog_checkpoints", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.accessory_publisher = self.node.create_publisher(AccessMotorCmd, "/cmd_accessory_pos", 10)
        self.vel_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        
        self.sub_people = self.node.create_subscription(
            PoseArray, 
            "/mecanumbot/people_fusion", 
            self.people_callback, 
            10
        )
        # NEW: Subscribe to AMCL to know where the robot is when looking at the person
        self.sub_amcl = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10
        )

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def _send_accessory_command(self, n_pos, gl_pos, gr_pos):
        cmd = AccessMotorCmd()
        cmd.n_pos = float(n_pos)
        cmd.gl_pos = float(gl_pos)
        cmd.gr_pos = float(gr_pos)
        self.node.get_logger().info(f"{self.name}: Sending accessory command: n_pos={n_pos}, gl_pos={gl_pos}, gr_pos={gr_pos}")
        self.accessory_publisher.publish(cmd)

    def people_callback(self, msg):
        if len(msg.poses) > 0:
            self.last_seen_time = self.node.get_clock().now()
            # Store the closest/first detected person's pose
            self.last_seen_person_pose = msg.poses[0]

    def get_closest_checkpoint_index(self, x, y):
        """Helper to find the closest checkpoint index to any given (x, y) coordinate."""
        checkpoints = self.blackboard.Dog_checkpoints
        min_dist = float('inf')
        closest_idx = 0
        for i, cp in enumerate(checkpoints):
            dx = cp.x - x
            dy = cp.y - y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    def update_search_direction(self):
        """Calculates whether the person is ahead (+) or behind (-) along the checkpoint path."""
        if self.robot_pose is None or self.last_seen_person_pose is None or not hasattr(self.blackboard, "Dog_checkpoints"):
            self.node.get_logger().warn(f"{self.name}: Missing pose or checkpoint data, defaulting search_direction to -1")
            self.blackboard.search_direction = -1
            return

        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        px = self.last_seen_person_pose.position.x
        py = self.last_seen_person_pose.position.y

        robot_idx = self.get_closest_checkpoint_index(rx, ry)
        person_idx = self.get_closest_checkpoint_index(px, py)

        if person_idx > robot_idx:
            self.blackboard.search_direction = 1
        elif person_idx < robot_idx:
            self.blackboard.search_direction = -1
        else:
            # EDGE CASE: Both are closest to the exact same checkpoint.
            # Use vector dot product to see if person is further along the path direction than the robot.
            checkpoints = self.blackboard.Dog_checkpoints
            if robot_idx < len(checkpoints) - 1:
                next_cp = checkpoints[robot_idx + 1]
                path_vx = next_cp.x - rx
                path_vy = next_cp.y - ry
            elif robot_idx > 0:
                prev_cp = checkpoints[robot_idx - 1]
                path_vx = rx - prev_cp.x
                path_vy = ry - prev_cp.y
            else:
                self.blackboard.search_direction = -1
                return

            # Vector from robot to person
            person_vx = px - rx
            person_vy = py - ry

            # Dot product: positive means pointing in the same general direction as the path
            dot_product = (path_vx * person_vx) + (path_vy * person_vy)
            self.blackboard.search_direction = 1 if dot_product >= 0 else -1

        self.node.get_logger().info(
            f"{self.name}: Robot at CP {robot_idx}, Person at CP {person_idx}. Set search_direction = {self.blackboard.search_direction}"
        )

    def update(self):
        if self.begin_wait_sent is False:
            self.node.get_logger().info(f"{self.name}: Starting WaitForPerson behaviour")
            self._send_accessory_command(7.0, 6.83, 3.36)  # Look ahead
            self.begin_wait_sent = True
            
        if self.last_seen_time is None:
            return py_trees.common.Status.RUNNING
            
        age = (self.node.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        if age <= self.sight_timeout:
            self.node.get_logger().info(f"{self.name}: Person detected! Interrupting search.")
            self._send_accessory_command(6.0, 6.83, 3.36)
            cmd = Twist()
            self.vel_publisher.publish(cmd)  # Stop the robot
            self.begin_wait_sent = False
            self.blackboard.patrol_current_checkpoint = None  # Reset patrol checkpoint to None
            
            # Dynamically set direction based on relative position before returning SUCCESS
            self.update_search_direction()
            
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING
    
class Spin360(py_trees.behaviour.Behaviour):
    """Spins the robot in a full circle using AMCL yaw tracking."""
    def __init__(self, name="Spin360", spin_speed=0.3):
        super().__init__(name)
        self.spin_speed = float(spin_speed)


    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.node.create_subscription(
            PoseWithCovarianceStamped, 
            "/amcl_pose", 
            self.amcl_callback, 
            10
        )
        self.current_yaw = None

    def initialise(self):
        self.accumulated_yaw = 0.0
        self.last_yaw = self.current_yaw

    def amcl_callback(self, msg):
        # Uses your existing yaw_from_quaternion function
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q)

    def update(self):
        if self.current_yaw is None or self.last_yaw is None:
            self.node.get_logger().info(f"{self.name}: Waiting for AMCL pose data...")
            self.last_yaw = self.current_yaw
            # FIX: Command the spin to force AMCL to update
            cmd = Twist()
            cmd.angular.z = self.spin_speed
            self.publisher.publish(cmd)
            return py_trees.common.Status.RUNNING
        
        #self.node.get_logger().info(f"{self.name}: Current Yaw: {self.current_yaw:.4f}, Last Yaw: {self.last_yaw:.4f}, Accumulated Yaw: {self.accumulated_yaw:.4f}")
        # Calculate shortest angular distance
        delta_yaw = self.current_yaw - self.last_yaw
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
        
        self.accumulated_yaw += abs(delta_yaw)
        self.last_yaw = self.current_yaw

        # Check if we hit a full circle (allow 5% margin for drift)
        if self.accumulated_yaw >= 2 * math.pi * 0.95:
            self.publisher.publish(Twist()) # Stop the robot
            self.node.get_logger().info(f"{self.name}: Full 360 scan completed.")
            return py_trees.common.Status.SUCCESS

        cmd = Twist()
        cmd.angular.z = self.spin_speed
        self.publisher.publish(cmd)
        return py_trees.common.Status.RUNNING
    
class ManageSearchCheckpoint(py_trees.behaviour.Behaviour):
    """Decrements checkpoints, switching to increments if it hits the start.
       On the first run, snaps to the closest checkpoint based on robot pose."""
       
    def __init__(self, name="ManageSearchCheckpoint"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("patrol_current_checkpoint", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("search_direction", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_max_checkpoint", access=py_trees.common.Access.READ)
        self.blackboard.register_key("Dog_checkpoints", access=py_trees.common.Access.READ)
        
        # New key to track if we have done our initial "snap to closest"
        self.blackboard.register_key("patrol_initialized", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        # We need the robot's pose to calculate the closest checkpoint
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
        self.robot_pose = None
        self.publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)
        # Defaults
        self.blackboard.search_direction = -1
        self.blackboard.patrol_current_checkpoint = 0
        self.blackboard.patrol_initialized = False

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def initialise(self):
        # Initialize search direction to negative (backwards) if not set
        if not hasattr(self.blackboard, "search_direction"):
            self.blackboard.search_direction = -1 

    def update(self):
        # Safety check: We can't find the closest point if we don't know where we are
        if self.robot_pose is None:
            self.node.get_logger().info(f"{self.name}: Waiting for robot pose to find closest checkpoint...")
            cmd = Twist()
            cmd.angular.z = self.spin_speed
            #self.publisher.publish(cmd)
            return py_trees.common.Status.RUNNING

        # --- ONE-TIME INITIALIZATION: SNAP TO CLOSEST ---
        if not getattr(self.blackboard, "patrol_initialized", False):
            self.node.get_logger().info(f"{self.name}: First run, snapping to closest checkpoint...")
            closest_idx = self.get_closest_checkpoint_index()
            self.blackboard.patrol_current_checkpoint = closest_idx
            self.blackboard.patrol_initialized = True
            
            self.node.get_logger().info(f"{self.name}: Initialized! Snapped to closest checkpoint: {closest_idx}")
            return py_trees.common.Status.SUCCESS

        # --- NORMAL BEHAVIOR: INCREMENT/DECREMENT ---
        if self.blackboard.patrol_current_checkpoint is None:
            self.blackboard.patrol_current_checkpoint = self.get_closest_checkpoint_index(self.robot_pose.position.x, self.robot_pose.position.y)
        self.node.get_logger().info(f"{self.name}: Current checkpoint: {self.blackboard.patrol_current_checkpoint}, Direction: {self.blackboard.search_direction}")    
        self.blackboard.patrol_current_checkpoint += self.blackboard.search_direction

        # Bound checks and direction flips
        if self.blackboard.patrol_current_checkpoint <= 0:
            self.blackboard.patrol_current_checkpoint = 0
            self.blackboard.search_direction = 1 # Reached start, go forward now
            self.node.get_logger().info(f"{self.name}: Hit start of maze. Searching forward.")
            
        elif self.blackboard.patrol_current_checkpoint >= self.blackboard.Dog_max_checkpoint-1:
            self.blackboard.patrol_current_checkpoint = self.blackboard.Dog_max_checkpoint
            self.blackboard.search_direction = -1 # Reached end, go backward now
            self.node.get_logger().info(f"{self.name}: Hit end of maze. Searching backward.")
            
        self.node.get_logger().info(f"{self.name}: Next search target is checkpoint {self.blackboard.patrol_current_checkpoint}")
        return py_trees.common.Status.SUCCESS

    def get_closest_checkpoint_index(self):
        """Helper function to calculate Euclidean distance to all checkpoints."""
        checkpoints = self.blackboard.Dog_checkpoints
        min_dist = float('inf')
        closest_idx = 0
        
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y

        for i, cp in enumerate(checkpoints):
            # Using cp.x and cp.y based on your is_robot_near_checkpoint implementation
            dx = cp.x - rx
            dy = cp.y - ry
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx
    
############################################# end of new stuff ################################################################    
class Approach(py_trees.behaviour.Behaviour):

    def __init__(self, name="Approach", target_type="subject", mode ="exact"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None
        self.subject_pose = None
        self.robot_pose = None
        self.mode = mode
        # Blackboard
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="visibility_time_threshold", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="start_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_closeness_threshold", access=py_trees.common.Access.READ) #stop_threshold
        self.blackboard.register_key(key="robot_approach_distance", access=py_trees.common.Access.READ) #go_threshold

        self.blackboard.register_key("Dog_checkpoints",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("patrol_checkpoints",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_current_checkpoint",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("patrol_current_checkpoint",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_max_checkpoint",access=py_trees.common.Access.WRITE)

        self.turning = False
        self.target_type = target_type
        self.subject_recovery_plan = None
        self.subject_recovery_index = 0
    
        self.cmd_send_time = None 

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)

        if self.target_type == "subject":
            self.subject_subscriber = node.create_subscription(
                                        PoseArray,
                                        "/mecanumbot/people_fusion",
                                        self.subject_callback,
                                        10,
        )
        
        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.goal_sent = False
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goals_in_sys = None
        self.compare_position = None
        self.approach_started = False
        self.timeout_threshold = 3.0
        return super().initialise()

    def update(self):
        #self.node.get_logger().info(f'{self.name}: called')
        now = self.node.get_clock().now()
        # Safety Checks
        if self.robot_pose is None:
            return py_trees.common.Status.RUNNING
        
        if not self.approach_started:
            self.approach_started = True
            self.approach_started_time = now
            self.node.get_logger().info(f"{self.name}: Starting approach to {self.target_type}")
            
        if self.compare_position is None:
            if self.target_type == "subject":
                if self.subject_pose is None:
                    if now - self.approach_started_time > rclpy.duration.Duration(seconds=self.timeout_threshold):
                        self.node.get_logger().info(f"{self.name}: No subject detected for {self.timeout_threshold} seconds, aborting approach")
                        self.approach_started = False
                        return py_trees.common.Status.FAILURE
                    return py_trees.common.Status.RUNNING
                self.compare_position = self.subject_pose.position

            elif self.target_type == "checkpoint":
                    self.node.get_logger().info(f"current checkpoint: {self.blackboard.Dog_current_checkpoint}")
                    self.compare_position = self.blackboard.Dog_checkpoints[self.blackboard.Dog_current_checkpoint]
            elif self.target_type == "patrol":
                    self.compare_position = self.blackboard.patrol_checkpoints[self.blackboard.patrol_current_checkpoint]       
            elif self.target_type == "start":
                self.compare_position = self.blackboard.start_position
            else:
                self.compare_position = self.blackboard.Dog_checkpoints[-1]
        if self.compare_position is None:
            self.node.get_logger().info(f"{self.name}: No compare position available, aborting approach")
            return py_trees.common.Status.FAILURE

        if not self.goal_sent:
            if self.goals_in_sys is not None and len(self.goals_in_sys) >= 0: # robot goal status array has history
                robot_has_goal_running = self.check_if_running()
                if not robot_has_goal_running:
                    self.node.get_logger().info(f'{self.name}: goal not sent, has no prev running, goals in sys NONEMPTY')
                    self.send_goal_command()
                    return py_trees.common.Status.RUNNING

                else:
                    self.node.get_logger().info(f"{self.name}: Waiting for previous goal to finish")
                    return py_trees.common.Status.RUNNING
            else:
                self.node.get_logger().info(f'{self.name}: goal not sent, goals in sys EMPTY')
                self.send_goal_command()
                return py_trees.common.Status.RUNNING
        else:
            if self.goal_uuid is None:
                self.assign_goal_uuid()
            if self.goal_uuid is None:
                self.node.get_logger().info(f"{self.name}: No goal UUID assigned yet")
                dt = self.node.get_clock().now() - self.cmd_send_time
                if  dt >  rclpy.duration.Duration(seconds=10.0):
                    self.approach_started = False
                    self.node.get_logger().info(f'{self.name}:Timeout, node failed, dt = {dt}, thresh: {rclpy.duration.Duration(seconds=10.0)}')
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING
            for goal in self.goals_in_sys:
                #self.node.get_logger().info(f"{self.name}: Checking goal UUID {goal.goal_info.goal_id.uuid} against {self.goal_uuid}")
                if np.array_equal(goal.goal_info.goal_id.uuid, self.goal_uuid):
                    self.goal_status = goal.status
            if self.goal_status == STATUS_SUCCEEDED:
                self.node.get_logger().info(f"{self.name}: Approach completed successfully") 
                if self.target_type == "checkpoint":
                    if self.blackboard.Dog_current_checkpoint < self.blackboard.Dog_max_checkpoint:
                            self.blackboard.Dog_current_checkpoint += 1
                self.approach_started = False
                self.node.get_logger().info(f"{self.name}: Approach to {self.target_type} completed successfully")
                return py_trees.common.Status.SUCCESS
            elif self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
                #self.node.get_logger().info(f"{self.name}: Approach in progress")
                return py_trees.common.Status.RUNNING
            elif self.goal_status in [STATUS_ABORTED, STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN]:
                self.node.get_logger().info(f"{self.name}: Approach failed, retrying")
                self.goal_sent = False
                self.goal_uuid = None
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING
        
    def assign_goal_uuid(self):
        if self.goals_in_sys is None:
            self.node.get_logger().info('No goals in sys')
            return

        # Iterate in reverse (newest first)
        for goal_status in reversed(self.goals_in_sys):
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            cmd_time = Time.from_msg(self.cmd_send_time.to_msg())
            
            # Check if this goal was created AFTER or AT the same time we sent the command
            self.node.get_logger().info(f'Goal_time: {goal_time}, cmd_time: {cmd_time}')
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                self.node.get_logger().info(f"{self.name}: Locked onto Goal UUID {self.goal_uuid}")
                return

    def check_if_running(self):
        """Check if there is a goal currently being executed by nav2."""
        if self.goals_in_sys is None:
            return False
            
        for goal_status in self.goals_in_sys:
            # Only consider a goal "running" if it is EXECUTING
            # Be careful with ACCEPTED; if Nav2 is stuck in ACCEPTED, we might hang.
            if goal_status.status == STATUS_EXECUTING:
                return True
        return False
    
    def send_goal_command(self):
        stop_threshold = self.blackboard.robot_closeness_threshold
        if self.target_type == 'subject':
            go_threshold = self.blackboard.robot_approach_distance
        else:
            go_threshold = 1.0
        """Helper to create and publish the command with fresh timestamp"""
        desired_pose = pose_to_goal(self.compare_position, self.robot_pose, stop_threshold=stop_threshold, mode=self.mode, go_threshold = go_threshold)
        
        self.goal_cmd = PoseStamped()
        self.goal_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        self.goal_cmd.header.stamp = self.cmd_send_time.to_msg() 
        self.goal_cmd.pose = desired_pose
        
        self.publisher.publish(self.goal_cmd)
        
        self.goal_sent = True
        #self.node.get_logger().info(f"Robot pose is: {self.robot_pose}")
        self.node.get_logger().info(f"{self.name}: Published goal command type: {self.target_type} \n Directions: X: {desired_pose.position.x} Y: {desired_pose.position.y} Z: {desired_pose.orientation.z} W: {desired_pose.orientation.w}")
        if self.target_type == "checkpoint":
            self.node.get_logger().info(f"{self.name}: Current checkpoint: {self.blackboard.Dog_current_checkpoint + 1} / {self.blackboard.Dog_max_checkpoint + 1}")
    
    def goal_status_callback(self, msg):
        """
        Callback to monitor nav2goal status updates.
        OPTIMIZATION: Only keep the last 5 statuses to prevent memory/CPU flood.
        """
        # Nav2 appends new goals to the end of the list. 
        # We only care about the most recent ones.
        if len(msg.status_list) > 5:
            self.goals_in_sys = msg.status_list[-5:]
        else:
            self.goals_in_sys = msg.status_list
        
        # self.logger.debug(f"{self.name}: Updated goal status list")
        
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def subject_callback(self, msg):
        self.subj_last_seen = msg.header.stamp
        self.subject_pose = msg.poses[0]

    def is_robot_near_checkpoint(self, checkpoint_idx):
        if self.robot_pose is None:
            return False

        checkpoint = self.blackboard.Dog_checkpoints[checkpoint_idx]
        dx = checkpoint.x - self.robot_pose.position.x
        dy = checkpoint.y - self.robot_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        return dist <= float(self.blackboard.robot_closeness_threshold)
    
class CheckSubjectTargetSuccess(py_trees.behaviour.Behaviour): #TODO
    """
    Checks if the subject has successfully reached the target
    based on distance threshold from blackboard entries.
    """

    def __init__(self, name="CheckSubjectTargetSuccess"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_reached_threshold",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_position",access=py_trees.common.Access.READ)


    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        self.subject_subscriber = node.create_subscription(
            PoseArray,
            "/mecanumbot/people_fusion",
            self.subject_callback,
            10
        )
        self.subject_pose = None
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.subject_pose is None:
            self.node.get_logger().info(f"{self.name}: No subject pose received yet")
            return py_trees.common.Status.RUNNING
        if self.blackboard.target_reached_threshold>=self.dist:
            self.node.get_logger().info(f"{self.name}: Subject reached target: within threshold, distance: distance: {self.dist}")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"{self.name}: Subject has not reached target yet, distance: {self.dist}")
            return py_trees.common.Status.FAILURE

    def subject_callback(self, msg):
        self.subject_pose = msg.poses[0]
        #self.node.get_logger(f'subject pose set to {self.subject_pose}')
        target_position = self.blackboard.target_position
        dx = target_position.x - self.subject_pose.position.x
        dy = target_position.y - self.subject_pose.position.y
        self.dist = math.sqrt(dx*dx + dy*dy)

class CheckRobotHasBall(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckRobotHasBall"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="ball_seen_time_threshold",access=py_trees.common.Access.READ)
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        self.ball_time_subscriber = node.create_subscription(
            Bool,
            "/mecanumbot/has_object",
            self.has_ball_callback,
            10
        )
        self.has_ball = None
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.has_ball is None:
            self.node.get_logger().info(f"{self.name}: No ball time received yet")
            return py_trees.common.Status.RUNNING

        if self.has_ball == True:
            self.node.get_logger().info(f"{self.name}: Robot has ball")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"{self.name}: Robot does NOT have ball")
            return py_trees.common.Status.FAILURE
        
    def has_ball_callback(self, msg):
        self.has_ball = msg.data

class CheckRobotAtLastCheckpoint(py_trees.behaviour.Behaviour):
    """Return SUCCESS when the robot reached the last checkpoint index."""

    def __init__(self, name="CheckRobotAtLastCheckpoint"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("Dog_current_checkpoint", access=py_trees.common.Access.READ)
        self.blackboard.register_key("Dog_max_checkpoint", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        current_checkpoint = self.blackboard.Dog_current_checkpoint
        max_checkpoint = self.blackboard.Dog_max_checkpoint

        if current_checkpoint >= max_checkpoint:
            self.node.get_logger().info(
                f"{self.name}: Robot is at last checkpoint ({current_checkpoint}/{max_checkpoint})"
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f"{self.name}: Robot not at last checkpoint yet ({current_checkpoint}/{max_checkpoint})"
        )
        return py_trees.common.Status.FAILURE

def calculate_facing_orientation(robot_pose, target_position):
    # 1. Vector FROM Robot TO Target (Target - Robot)
    dx = target_position.x - robot_pose.position.x
    dy = target_position.y - robot_pose.position.y

    # 2. Absolute angle in the Map frame
    desired_yaw = np.arctan2(dy, dx)

    # 3. Convert to Quaternion
    # We do NOT subtract robot_yaw. We want the absolute compass direction.
    q = euler2quat(0, 0, desired_yaw)

    orientation = PoseStamped().pose.orientation
    orientation.w = q[0]
    orientation.x = q[1]
    orientation.y = q[2]
    orientation.z = q[3]
    
    return orientation

def pose_to_goal(object_position, robot_pose, stop_threshold=0.3, mode="exact", go_threshold=1.0):
    Ox = object_position.x
    Oy = object_position.y
    Rx = robot_pose.position.x
    Ry = robot_pose.position.y
    
    dx = Ox - Rx
    dy = Oy - Ry
    dist = math.sqrt(dx*dx + dy*dy)

    # 1. Check if we are already there
    if dist < stop_threshold:
        return robot_pose

    X, Y = Rx, Ry # Default to current pos

    # 2. Calculate Target Point
    if mode == "exact": 
        # Go straight to the final point (minus the buffer)
        ratio = (dist - stop_threshold) / dist
        X = Rx + dx * ratio
        Y = Ry + dy * ratio

    elif mode == "fixed_distance": 
        # Check if the step size is LARGER than the distance left
        effective_dist = dist - stop_threshold
        
        if go_threshold >= effective_dist:
            # If one step is enough to finish, just go to the end
            ratio = (dist - stop_threshold) / dist
            X = Rx + dx * ratio
            Y = Ry + dy * ratio
        else:
            # Otherwise, take one step (go_threshold) forward
            ratio = go_threshold / dist
            X = Rx + dx * ratio
            Y = Ry + dy * ratio

    # Orientation: face the person
    yaw = math.atan2(Oy - Y, Ox - X)
    q = euler2quat(0, 0, yaw)

    goal = Pose()
    goal.position.x = X
    goal.position.y = Y
    goal.orientation.x = q[0]
    goal.orientation.y = q[1]
    goal.orientation.z = q[2]
    goal.orientation.w = q[3]
    return goal

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

