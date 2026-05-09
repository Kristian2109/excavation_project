"""
base_motion_node.py – Executes base trajectory and publishes TF + visualization.

This node:
  1. Plans a trajectory from (0,0,0) to the working position.
  2. Publishes world → base_link TF as the robot drives.
  3. Publishes the planned path as a nav_msgs/Path for Foxglove.
  4. Publishes a Bool on /base_motion/done when complete.
  5. Subscribes to /goal_pose (PoseStamped) for new runtime goals.
     In Foxglove 3D panel: click the "Pose" tool and click in the scene.

Parameters:
    start_x, start_y, start_yaw       – initial base pose
    goal_x, goal_y, goal_yaw          – target working position
    linear_speed                       – m/s   (default 0.5)
    angular_speed                      – rad/s (default 0.3)
    publish_rate                       – Hz    (default 20.0)
    auto_start                         – start immediately (default true)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import (
    TransformStamped,
    PoseStamped,
    Quaternion,
)
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from tf2_ros import TransformBroadcaster

from excavation_core.base_planner import (
    BasePose,
    plan_base_trajectory,
)
from excavation_core.parameters import (
    declare_base_motion_node_parameters,
    retrieve_base_motion_node_parameters,
)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class BaseMotionNode(Node):
    def __init__(self) -> None:
        super().__init__('base_motion')

        self._params = self._load_parameters()
        start, goal = self._load_start_and_goal()
        self._load_speed_settings()
        self._plan_trajectory(start, goal)
        self._initialize_state(start)
        self._setup_ros_interfaces()
        self._publish_path()
        self._setup_timer()
        self._maybe_auto_start()

    def _load_parameters(self):
        """Declare and retrieve all node parameters from central helpers."""
        declare_base_motion_node_parameters(self)
        return retrieve_base_motion_node_parameters(self)

    def _load_start_and_goal(self) -> tuple[BasePose, BasePose]:
        """Load initial and goal poses from parameters."""
        start = BasePose(
            x=self._params.start_x,
            y=self._params.start_y,
            yaw=self._params.start_yaw,
        )
        goal = BasePose(
            x=self._params.goal_x,
            y=self._params.goal_y,
            yaw=self._params.goal_yaw,
        )
        return start, goal

    def _load_speed_settings(self) -> None:
        """Cache speed settings used for planning and re-planning."""
        speed_mult = max(0.1, self._params.speed_multiplier)
        self.linear_speed = self._params.linear_speed * speed_mult
        self.angular_speed = self._params.angular_speed * speed_mult

    def _plan_trajectory(self, start: BasePose, goal: BasePose) -> None:
        """Plan the current active trajectory."""
        self.trajectory = plan_base_trajectory(
            start,
            goal,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
        )
        self.get_logger().info(
            f'Base trajectory planned: {len(self.trajectory.points)} points, '
            f'duration={self.trajectory.duration:.1f}s')

    def _initialize_state(self, start: BasePose) -> None:
        """Initialize runtime state variables."""
        self.running = False
        self.done = False
        self.t_start = None
        self.current_pose = start

    def _setup_ros_interfaces(self) -> None:
        """Create publishers, subscribers, and TF broadcaster."""
        self.tf_broadcaster = TransformBroadcaster(self)

        latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/base_motion/path', latching)
        self.done_pub = self.create_publisher(
            Bool,
            '/base_motion/done',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_cb, 10)

    def _setup_timer(self) -> None:
        """Create periodic timer that advances trajectory and publishes TF."""
        self.timer = self.create_timer(1.0 / self._params.publish_rate, self._tick)

    def _maybe_auto_start(self) -> None:
        """Start motion immediately when configured."""
        if self._params.auto_start:
            self.start_motion()

    # ------------------------------------------------------------------ #
    #  Public API
    # ------------------------------------------------------------------ #
    def start_motion(self) -> None:
        """Begin executing the planned trajectory."""
        if self.running:
            return
        self.running = True
        self.done = False
        self.t_start = self.get_clock().now()
        # Immediately notify subscribers that motion is in progress
        msg = Bool()
        msg.data = False
        self.done_pub.publish(msg)
        self.get_logger().info('Base motion started')

    # ------------------------------------------------------------------ #
    #  Goal pose callback (from Foxglove "Pose" tool or /goal_pose topic)
    # ------------------------------------------------------------------ #
    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """Re-plan trajectory from current position to clicked goal."""
        # Immediately clear done so stale done=True stops being latched
        self.done = False
        self.running = False
        done_msg = Bool()
        done_msg.data = False
        self.done_pub.publish(done_msg)

        # Extract yaw from quaternion
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        goal = BasePose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=yaw,
        )

        self.get_logger().info(
            f'New goal received: ({goal.x:.2f}, {goal.y:.2f}, '
            f'yaw={math.degrees(goal.yaw):.1f}°)')

        # Re-plan from wherever we are now
        self.trajectory = plan_base_trajectory(
            self.current_pose, goal,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
        )
        self.get_logger().info(
            f'Re-planned: {len(self.trajectory.points)} points, '
            f'duration={self.trajectory.duration:.1f}s')

        self._publish_path()
        self.start_motion()

    # ------------------------------------------------------------------ #
    #  Timer callback
    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        if self.running and self.t_start is not None:
            elapsed = (self.get_clock().now() - self.t_start).nanoseconds / 1e9
            self.current_pose = self.trajectory.sample(elapsed)

            if elapsed >= self.trajectory.duration and not self.done:
                self.done = True
                self.running = False
                self.current_pose = self.trajectory.end_pose
                # Publish done=True once (latched via TRANSIENT_LOCAL)
                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(done_msg)
                self.get_logger().info(
                    f'Base motion complete. Final pose: '
                    f'({self.current_pose.x:.2f}, {self.current_pose.y:.2f}, '
                    f'yaw={math.degrees(self.current_pose.yaw):.1f}°)')

        # Always publish TF so the robot has a valid world→base_link
        self._publish_tf()

    # ------------------------------------------------------------------ #
    #  TF publisher
    # ------------------------------------------------------------------ #
    def _publish_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.current_pose.x
        t.transform.translation.y = self.current_pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation = _yaw_to_quaternion(self.current_pose.yaw)
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    #  Path visualization
    # ------------------------------------------------------------------ #
    def _publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()

        for tp in self.trajectory.points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = tp.pose.x
            ps.pose.position.y = tp.pose.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = _yaw_to_quaternion(tp.pose.yaw)
            msg.poses.append(ps)

        self.path_pub.publish(msg)
        self.get_logger().info(
            f'Published base path ({len(msg.poses)} poses)')


def main(args=None):
    rclpy.init(args=args)
    node = BaseMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
