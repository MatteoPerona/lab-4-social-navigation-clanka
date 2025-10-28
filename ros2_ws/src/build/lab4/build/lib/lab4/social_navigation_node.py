#!/usr/bin/env python3
"""
social_navigation_node.py
Send three Nav2 waypoints (start -> midpoint -> final) to a TurtleBot4.

Design:
- Optionally publishes the "start" as /initialpose (AMCL) to set localization seed.
- Then sends NavigateToPose action goals in sequence: start -> mid -> final.
- If the robot is already at the start pose, Nav2 will return success quickly.
- All poses are in the "map" frame. Yaw is in degrees for ergonomics.

Requires:
- Nav2 running with a loaded map and AMCL (TurtleBot4 bringup).
- Action server name: 'navigate_to_pose' (nav2_msgs/action/NavigateToPose).

Author/Maintainer: Giuliano Pioldi <gp433@cornell.edu>
"""
from math import cos, sin, radians
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


def yaw_deg_to_quat(z_deg: float):
    """Return (x,y,z,w) quaternion for yaw about Z (roll=pitch=0)."""
    half = radians(z_deg) * 0.5
    q = (0.0, 0.0, sin(half), cos(half))
    return q


def make_pose_stamped(x: float, y: float, yaw_deg: float, frame_id: str = 'map') -> PoseStamped:
    """Helper to create PoseStamped in 'frame_id' at time=now. PoseStamped is a standard message 
    type that combines a Pose (containing (x,y,z) position and quaternion orientation) with a 
    std_msgs/Header. This header provides a timestamp and the reference coordinate frame ID, 
    making the pose's context and timing clear for use in navigation and other applications. 
    frame_id: The name of the coordinate frame to which the pose is relative 
    (e.g., map, odom, base_link). """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    # stamp will be set by node.get_clock().now() when sending
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_deg_to_quat(yaw_deg)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class SocialNavigationNode(Node):
    def __init__(self):
        super().__init__('social_navigation_node')

        # ---- Parameters (can be overridden via ROS params / launch) ----
        # Each pose: [x, y, yaw_deg]
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_cov_xy', 0.25)     # variance (m^2) for x,y
        self.declare_parameter('initial_cov_yaw', 0.0685)  # variance (rad^2) ~ (15 deg)^2 in rad^2
        self.declare_parameter('timeout_sec', 180.0)       # per-goal timeout

        # Checkpoint 3: Set defaults waypoints — replace with your map coordinates:
        self.declare_parameter('start', [0.5979909896850586, 0.8495365381240845, -0.001434326171875])
        self.declare_parameter('mid',   [0.03517220914363861, 1.7675262689590454, -0.001434326171875])
        self.declare_parameter('final', [-0.892248272895813, -0.5965396165847778, 0.004547119140625])

        # Read ROS2 Parameters
        # frame_id: The name of the coordinate frame to which the pose is relative (e.g., map, odom, base_link).
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        # Initial robot pose (initial location in the map)
        self.publish_initial_pose_flag = self.get_parameter('publish_initial_pose').get_parameter_value().bool_value
        # Final time to accept robot requests
        self.timeout_sec = float(self.get_parameter('timeout_sec').get_parameter_value().double_value)
        # Starting xYZ location in a 3D map
        self.start_xyz = [float(v) for v in self.get_parameter('start').get_parameter_value().double_array_value]
        self.mid_xyz   = [float(v) for v in self.get_parameter('mid').get_parameter_value().double_array_value]
        self.final_xyz = [float(v) for v in self.get_parameter('final').get_parameter_value().double_array_value]

        self.initial_cov_xy   = float(self.get_parameter('initial_cov_xy').get_parameter_value().double_value)
        self.initial_cov_yaw  = float(self.get_parameter('initial_cov_yaw').get_parameter_value().double_value)

        # Set initialpose for AMCL which implements the server for taking a static map and localizing the robot 
        # within it using an Adaptive Monte-Carlo Localizer.
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Nav2 action client: https://docs.nav2.org/concepts/index.html
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Kick off once everything is up and running
        self.timer = self.create_timer(1.0, self._on_timer_once)
        self._started = False

        self.get_logger().info('SocialNavigationNode ready. Waiting to start...')

    def _on_timer_once(self):
        """Starts the navigation sequence once."""
        if self._started:
            return
        self._started = True
        self.timer.cancel()
        self.get_logger().info('Starting navigation sequence (start -> mid -> final).')

        # 1) Optionally publish the "start" as the initial pose for AMCL
        if self.publish_initial_pose_flag:
            self._publish_initial_pose(self.start_xyz)
            self.get_logger().info('Published /initialpose using "start" pose. (Rviz initial pose is also acceptable).')

        # 2) Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('navigate_to_pose action server not available. Is Nav2 running?')
            return

        # 3) Build the three waypoints
        waypoints = [
            make_pose_stamped(*self.start_xyz, frame_id=self.frame_id),
            make_pose_stamped(*self.mid_xyz, frame_id=self.frame_id),
            make_pose_stamped(*self.final_xyz, frame_id=self.frame_id),
        ]
        # Stamp them with "now" to avoid stale times
        now = self.get_clock().now().to_msg()
        for p in waypoints:
            p.header.stamp = now

        # 4) Send goals in sequence
        for i, pose in enumerate(waypoints, start=1):
            if not self._navigate_to_pose(pose, index=i):
                self.get_logger().error(f'Aborting sequence at waypoint {i}.')
                return

        self.get_logger().info('All three waypoints reached successfully.')

    def _publish_initial_pose(self, xyz: List[float]):
        """Publish PoseWithCovarianceStamped to /initialpose to seed AMCL."""
        x, y, yaw_deg = xyz
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_deg_to_quat(yaw_deg)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Simple diagonal covariance (x, y, yaw) with some uncertainty
        cov = [0.0] * 36
        cov[0] = self.initial_cov_xy     # x
        cov[7] = self.initial_cov_xy     # y
        cov[35] = self.initial_cov_yaw   # yaw (theta)
        msg.pose.covariance = cov

        self.initial_pose_pub.publish(msg)

    def _navigate_to_pose(self, pose: PoseStamped, index: int) -> bool:
        """Send a single NavigateToPose goal and block until result (with timeout).
        NavigateToPose is implemented as a ROS 2 Action. This means it involves a 
        client (e.g., a user or another ROS node) sending a goal to an action server 
        (the bt_navigator in Nav2) and receiving feedback and a final result."""
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'[{index}/3] Sending goal: '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, '
            f'yaw(deg)≈{self._quat_to_yaw_deg(pose.pose.orientation):.1f}'
        )

        """To send a goal asynchronously to the NavigateToPose action server in ROS 2, 
        you must use the send_goal_async() method of an ActionClient. The action client 
        is typically created in a Python node using rclpy and the 
        nav2_msgs/action/NavigateToPose action interface. 
        """
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_feedback(index, fb)
        )
        rclpy.task.Future.add_done_callback(send_future, lambda f: None)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.timeout_sec)

        if not send_future.done():
            self.get_logger().error(f'[{index}/3] Timed out waiting for goal acceptance.')
            return False

        """The immediate Future returned by send_goal_async() completes when the Nav2 
        action server either accepts or rejects the goal. 
        """
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{index}/3] Goal was rejected by the server.')
            return False

        """nav_client.send_goal_async() and goal_handle.get_result_async() 
        are the standard asynchronous methods used by a client to send a navigation 
        goal and retrieve the final result. This process is part of the Actions 
        communication pattern used by the Nav2 stack, which handles long-running 
        tasks like navigating to a specific pose. 
        """
        self.get_logger().info(f'[{index}/3] Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)

        """After calling nav_client.send_goal_async(), you must also wait for the goal 
        to be accepted by the Nav2 action server before you can get the final result. 
        The goal acceptance is an asynchronous step that returns a future. Once the goal 
        is accepted, you can then retrieve another future for the final result. 
        """
        if not result_future.done():
            self.get_logger().error(f'[{index}/3] Timed out waiting for result.')
            goal_handle.cancel_goal_async()
            return False

        result = result_future.result().result
        status = result_future.result().status
        # status codes come from action_msgs/GoalStatus; 4=SUCCEEDED
        if status == 4:
            self.get_logger().info(f'[{index}/3] Reached waypoint {index}.')
            return True
        else:
            self.get_logger().error(f'[{index}/3] Navigation failed with status={status}.')
            return False

    def _on_feedback(self, index: int, feedback_msg):
        """Callback function for NavigateToPose"""
        fb = feedback_msg.feedback
        # fb contains: current_pose, navigation_time, estimated_time_remaining, etc. (see docs)
        # Keep logs light; print distance remaining occasionally could be added.
        # Here we log every callback sparsely by throttling via logger.
        self.get_logger().debug(
            f'[{index}/3] Feedback: est. time remaining ~ {fb.estimated_time_remaining.sec}s, '
            f'recoveries={fb.number_of_recoveries}'
        )

    @staticmethod
    def _quat_to_yaw_deg(q) -> float:
        """Extract yaw (deg) for logging; assumes planar orientation."""
        # yaw = atan2(2*w*z, 1 - 2*z^2) for qx=qy=0
        import math
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * (q.z ** 2))
        return math.degrees(yaw)


def main():
    rclpy.init()
    node = SocialNavigationNode()
    try:
        # Spin the node; internal logic runs on a one-shot timer
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
