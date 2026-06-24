#!/usr/bin/env python3
#################################################################################
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################
#
# Authors: Ryan Shim, Gilbert, ChanHyeong Lee

import json
import math
import os
import random

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from std_srvs.srv import Trigger

from turtlebot3_msgs.srv import Dqn
from turtlebot3_msgs.srv import Goal


ROS_DISTRO = os.environ.get('ROS_DISTRO')


class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')
        self.declare_parameter('lidar_samples', 25)
        self.declare_parameter('max_step', 800)
        self.tables_loaded = False

        # Ubicacion de las mesas (fallback, se sobreescriben con la escena)
        self.tables = [
            [3, 3],     # mesa1
            [0, -3],    # mesa2
            [-3, 0],    # mesa3
        ]
        print('Mesas: Mesa1 (%.1f, %.1f), Mesa2 (%.1f, %.1f), Mesa3 (%.1f, %.1f)' % (
            self.tables[0][0], self.tables[0][1],
            self.tables[1][0], self.tables[1][1],
            self.tables[2][0], self.tables[2][1]))
        print('Tarea: servir 2 mesas cualesquiera (sin orden fijo)')
        self.tables_needed = 2
        self.visited_tables = [False, False, False]
        self.tables_visited_count = 0
        self.best_dist = [999.0, 999.0, 999.0]
        self.best_target_idx = -1
        self.best_target_dist = 0.0
        self.best_target_angle = 0.0

        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0

        self.action_size = 5
        self.max_step = self.get_parameter('max_step').get_parameter_value().integer_value

        self.done = False
        self.fail = False
        self.succeed = False

        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0

        self.local_step = 0
        self.table_reached_this_step = False
        self.stop_cmd_vel_timer = None
        self.angular_vel = [1.5, 0.75, 0.0, -0.75, -1.5]
        
        qos = QoSProfile(depth=10)

        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_sub_callback,
            qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_sub_callback,
            qos_profile_sensor_data
        )

        self.clients_callback_group = MutuallyExclusiveCallbackGroup()
        self.task_succeed_client = self.create_client(
            Goal,
            'task_succeed',
            callback_group=self.clients_callback_group
        )
        self.task_failed_client = self.create_client(
            Goal,
            'task_failed',
            callback_group=self.clients_callback_group
        )
        self.initialize_environment_client = self.create_client(
            Goal,
            'initialize_env',
            callback_group=self.clients_callback_group
        )
        self.table_goals_client = self.create_client(
            Trigger,
            'table_goals',
            callback_group=self.clients_callback_group
        )

        self.rl_agent_interface_service = self.create_service(
            Dqn,
            'rl_agent_interface',
            self.rl_agent_interface_callback
        )
        self.make_environment_service = self.create_service(
            Empty,
            'make_environment',
            self.make_environment_callback
        )
        self.reset_environment_service = self.create_service(
            Dqn,
            'reset_environment',
            self.reset_environment_callback
        )

    def try_load_tables(self):
        try:
            if not self.table_goals_client.wait_for_service(timeout_sec=2.0):
                self.tables_loaded = True
                self.get_logger().warn('table_goals service not available, using default coordinates')
                return
            req = Trigger.Request()
            future = self.table_goals_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.tables_loaded = True
            if future.done() and future.result() is not None and future.result().success:
                data = json.loads(future.result().message)
                tables = data.get('tables', [])
                if tables:
                    self.tables = []
                    for t in tables:
                        self.tables.append([t['x'], t['y']])
                    self.get_logger().info('Tables loaded from scene')
                    for i, t in enumerate(self.tables):
                        self.get_logger().info('  Mesa %d: [%.2f, %.2f]' % (i + 1, t[0], t[1]))
                else:
                    self.get_logger().warn('table_goals returned empty, using defaults')
            else:
                self.get_logger().warn('table_goals failed, using default coordinates')
        except Exception as e:
            self.tables_loaded = True
            self.get_logger().error('table_goals exception: %s' % str(e))

    def get_table_positions(self):
        while not self.table_goals_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('table_goals service not available, waiting ...')
        future = self.table_goals_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                data = json.loads(response.message)
                tables = data.get('tables', [])
                self.tables = []
                for t in tables:
                    self.tables.append([t['x'], t['y']])
                self.get_logger().info('Table positions loaded from scene')
                for i, t in enumerate(self.tables):
                    self.get_logger().info('  Mesa %d: [%.2f, %.2f]' % (i + 1, t[0], t[1]))
            else:
                self.get_logger().error('table_goals failed: ' + response.message)
        else:
            self.get_logger().error('table_goals service call failed')

    def reset_episode_state(self):
        self.visited_tables = [False, False, False]
        self.tables_visited_count = 0
        self.best_dist = [999.0, 999.0, 999.0]
        self.succeed = False
        self.fail = False
        self.done = False
        self.local_step = 0
        self.survival_steps = 0
        self.table_reached_this_step = False

    def make_environment_callback(self, request, response):
        self.get_logger().info('Make environment called')
        self.get_table_positions()
        self.reset_episode_state()
        self.get_logger().info('Entorno listo - sin goal fijo, recompensa guia a las mesas')
        while not self.initialize_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'service for initialize the environment is not available, waiting ...'
            )
        future = self.initialize_environment_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and not future.result().success:
            self.get_logger().error('initialize environment request failed')

        return response

    def reset_environment_callback(self, request, response):
        self.reset_episode_state()
        state = self.calculate_state()
        response.state = state

        return response

    def call_task_succeed(self):
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task succeed is not available, waiting ...')
        future = self.task_succeed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('service for task succeed finished')
        else:
            self.get_logger().error('task succeed service call failed')

    def call_task_failed(self):
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task failed is not available, waiting ...')
        future = self.task_failed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('service for task failed finished')
        else:
            self.get_logger().error('task failed service call failed')

    def scan_sub_callback(self, scan):
        self.scan_ranges = []
        self.front_ranges = []
        self.front_angles = []

        num_of_lidar_rays = len(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        self.front_distance = scan.ranges[0]

        for i in range(num_of_lidar_rays):
            angle = angle_min + i * angle_increment
            distance = scan.ranges[i]

            if distance == float('Inf'):
                distance = 3.5
            elif numpy.isnan(distance):
                distance = 0.0

            self.scan_ranges.append(distance)

            if (0 <= angle <= math.pi/2) or (3*math.pi/2 <= angle <= 2*math.pi):
                self.front_ranges.append(distance)
                self.front_angles.append(angle)

        self.min_obstacle_distance = min(self.scan_ranges)
        self.front_min_obstacle_distance = min(self.front_ranges) if self.front_ranges else 10.0

    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def calculate_state(self):
        state = []

        for tx, ty in self.tables:
            d = math.sqrt((tx - self.robot_pose_x) ** 2 + (ty - self.robot_pose_y) ** 2)
            state.append(float(min(d / 5.0, 1.0)))
        for tx, ty in self.tables:
            path_theta = math.atan2(ty - self.robot_pose_y, tx - self.robot_pose_x)
            goal_angle = path_theta - self.robot_pose_theta
            if goal_angle > math.pi:
                goal_angle -= 2 * math.pi
            elif goal_angle < -math.pi:
                goal_angle += 2 * math.pi
            state.append(float(goal_angle / math.pi))
        for v in self.visited_tables:
            state.append(1.0 if v else 0.0)
        state.append(float((self.robot_pose_x + 3.5) / 7.0))
        state.append(float((self.robot_pose_y + 3.5) / 7.0))

        lidar_samples = self.get_parameter(
            'lidar_samples'
        ).get_parameter_value().integer_value

        if len(self.front_ranges) == 0:
            sampled = [1.0] * lidar_samples
        else:
            orig_idx = numpy.linspace(0, len(self.front_ranges) - 1,
                                      len(self.front_ranges))
            target_idx = numpy.linspace(0, len(self.front_ranges) - 1,
                                        lidar_samples)
            sampled = numpy.interp(target_idx, orig_idx, self.front_ranges)
            sampled = numpy.clip(numpy.array(sampled) / 3.5, 0.0, 1.0)
        for var in sampled:
            state.append(float(var))

        self.local_step += 1
        self.table_reached_this_step = False

        self.best_target_idx = -1
        best_score = -1.0
        for i, visited in enumerate(self.visited_tables):
            if not visited:
                tx = self.tables[i][0]
                ty = self.tables[i][1]
                d = math.sqrt((tx - self.robot_pose_x) ** 2 + (ty - self.robot_pose_y) ** 2)
                path_theta = math.atan2(ty - self.robot_pose_y, tx - self.robot_pose_x)
                angle = path_theta - self.robot_pose_theta
                if angle > math.pi:
                    angle -= 2 * math.pi
                elif angle < -math.pi:
                    angle += 2 * math.pi
                score = (1.0 - abs(angle) / math.pi) / (1.0 + d)
                if score > best_score:
                    best_score = score
                    self.best_target_idx = i
                    self.best_target_dist = d
                    self.best_target_angle = angle

        if self.local_step == 1:
            for idx, t in enumerate(self.tables):
                self.get_logger().info('Mesa %d en (%.2f, %.2f)' % (idx + 1, t[0], t[1]))

        for i, visited in enumerate(self.visited_tables):
            if not visited:
                tx = self.tables[i][0]
                ty = self.tables[i][1]
                d = math.sqrt((tx - self.robot_pose_x) ** 2 + (ty - self.robot_pose_y) ** 2)
                if d < 0.70:
                    self.visited_tables[i] = True
                    self.tables_visited_count += 1
                    self.table_reached_this_step = True
                    self.survival_steps = 0
                    self.get_logger().info(
                        'Mesa %d alcanzada (%d/%d) | Robot: (%.2f, %.2f) | Mesa: (%.2f, %.2f) | Dist: %.3f' % (
                            i + 1, self.tables_visited_count, self.tables_needed,
                            self.robot_pose_x, self.robot_pose_y,
                            tx, ty, d)
                    )
                    if self.tables_visited_count >= self.tables_needed:
                        self.get_logger().info('Tarea completada: 2 mesas servidas')
                        self.succeed = True
                        self.done = True
                        self.local_step = 0
                    break


        if self.min_obstacle_distance < 0.15:
            self.get_logger().info('Collision happened')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            else:
                self.cmd_vel_pub.publish(TwistStamped())
            self.local_step = 0
            self.call_task_failed()

        if self.local_step == self.max_step:
            self.get_logger().info('Time out!')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            else:
                self.cmd_vel_pub.publish(TwistStamped())
            self.local_step = 0
            self.call_task_failed()

        return state

    def compute_directional_weights(self, relative_angles, max_weight=10.0):
        power = 6
        raw_weights = (numpy.cos(relative_angles))**power + 0.1
        scaled_weights = raw_weights * (max_weight / numpy.max(raw_weights))
        normalized_weights = scaled_weights / numpy.sum(scaled_weights)
        return normalized_weights

    def compute_weighted_obstacle_reward(self):
        if not self.front_ranges or not self.front_angles:
            return 0.0

        front_ranges = numpy.array(self.front_ranges)
        front_angles = numpy.array(self.front_angles)

        valid_mask = front_ranges <= 0.8
        if not numpy.any(valid_mask):
            return 0.0

        front_ranges = front_ranges[valid_mask]
        front_angles = front_angles[valid_mask]

        relative_angles = numpy.unwrap(front_angles)
        relative_angles[relative_angles > numpy.pi] -= 2 * numpy.pi

        weights = self.compute_directional_weights(relative_angles, max_weight=10.0)

        safe_dists = numpy.clip(front_ranges - 0.25, 1e-2, 3.5)
        decay = numpy.exp(-2.0 * safe_dists)

        weighted_decay = numpy.dot(weights, decay)

        reward = -(0.3 + 1.5 * weighted_decay)

        return reward

    def calculate_reward(self):
        obstacle_reward = self.compute_weighted_obstacle_reward()
        step_penalty = -0.02
        side_penalty = -2.0 if self.min_obstacle_distance < 0.35 else 0.0

        self.survival_steps += 1
        survival_bonus = 0.1 if self.survival_steps > 0 and self.survival_steps % 25 == 0 else 0.0

        table_reward = 0.0
        for i, visited in enumerate(self.visited_tables):
            if not visited:
                tx = self.tables[i][0]
                ty = self.tables[i][1]
                d = math.sqrt((tx - self.robot_pose_x) ** 2 + (ty - self.robot_pose_y) ** 2)
                path_theta = math.atan2(ty - self.robot_pose_y, tx - self.robot_pose_x)
                angle = path_theta - self.robot_pose_theta
                if angle > math.pi:
                    angle -= 2 * math.pi
                elif angle < -math.pi:
                    angle += 2 * math.pi

                orient_factor = 0.2 + 0.8 * (1.0 - abs(angle) / math.pi)
                dist_factor = 1.0 / math.sqrt(1.0 + d)
                table_reward += orient_factor * dist_factor * 3.0

                if d < self.best_dist[i]:
                    table_reward += 2.0
                    self.best_dist[i] = d

        reward = table_reward + obstacle_reward + step_penalty + side_penalty + survival_bonus

        if self.table_reached_this_step:
            if self.tables_visited_count >= self.tables_needed:
                reward += 300.0
                self.get_logger().info('BONUS +300 por completar las 2 mesas')
            else:
                reward += 100.0
                self.get_logger().info('BONUS +100 por mesa alcanzada (%d/%d)' % (
                    self.tables_visited_count, self.tables_needed))

        if self.succeed:
            reward += 200.0
        elif self.fail:
            reward -= 50.0

        if self.local_step % 50 == 0:
            print('Step: %d, Goal: M%d, Dist: %.2f, Angle: %.2f, Mesas: %d/%d, Total: %.2f, Obst: %.2f, Tables: %.2f, Surv: %d' % (
                self.local_step,
                self.best_target_idx + 1 if self.best_target_idx >= 0 else 0,
                self.best_target_dist,
                self.best_target_angle,
                self.tables_visited_count, self.tables_needed,
                reward,
                obstacle_reward,
                table_reward,
                self.survival_steps))

        return reward

    def rl_agent_interface_callback(self, request, response):
        action = request.action

        if hasattr(request, 'init') and request.init:
            if not self.tables_loaded:
                self.try_load_tables()
            self.reset_episode_state()

        if ROS_DISTRO == 'humble':
            msg = Twist()
            msg.linear.x = 0.2
            msg.angular.z = self.angular_vel[action]
        else:
            msg = TwistStamped()
            msg.twist.linear.x = 0.2
            msg.twist.angular.z = self.angular_vel[action]

        self.cmd_vel_pub.publish(msg)
        if self.stop_cmd_vel_timer is None:
            self.stop_cmd_vel_timer = self.create_timer(0.8, self.timer_callback)
        else:
            self.destroy_timer(self.stop_cmd_vel_timer)
            self.stop_cmd_vel_timer = self.create_timer(0.8, self.timer_callback)

        response.state = self.calculate_state()
        response.reward = self.calculate_reward()
        response.done = self.done

        if self.done is True:
            self.done = False
            self.succeed = False
            self.fail = False

        return response

    def timer_callback(self):
        self.get_logger().info('Stop called')
        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub.publish(Twist())
        else:
            self.cmd_vel_pub.publish(TwistStamped())
        self.destroy_timer(self.stop_cmd_vel_timer)

    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    rl_environment = RLEnvironment()
    try:
        while rclpy.ok():
            rclpy.spin_once(rl_environment, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rl_environment.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
