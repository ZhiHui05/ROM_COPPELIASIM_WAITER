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
        self.two_tables_bonus = False

        #Ubicacion de las mesas
        self.tables = [
            [3, 3],     #mesa1
            [0, -3],    #mesa2
            [-3, 0],    #mesa3
        ]
        print('Mesas: Mesa1 (%.1f, %.1f), Mesa2 (%.1f, %.1f), Mesa3 (%.1f, %.1f)' % (
            self.tables[0][0], self.tables[0][1],
            self.tables[1][0], self.tables[1][1],
            self.tables[2][0], self.tables[2][1]))
        self.current_table = 0

        self.goal_pose_x = self.tables[0][0]
        self.goal_pose_y = self.tables[0][1]

        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0

        self.action_size = 5
        self.max_step = 800

        self.done = False
        self.fail = False
        self.succeed = False

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 0.5
        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0
        self.is_front_min_actual_front = False

        self.prev_goal_distance = 1.0
        self.best_goal_distance = 1.0
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

    def make_environment_callback(self, request, response):
        self.get_logger().info('Make environment called')
        self.get_table_positions()
        self.goal_pose_x = self.tables[0][0]
        self.goal_pose_y = self.tables[0][1]
        self.get_logger().info(
            'Goal inicial: Mesa 1 [%.2f, %.2f]' % (self.goal_pose_x, self.goal_pose_y)
        )
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
        self.current_table = 0
        self.two_tables_bonus = False
        self.goal_pose_x = self.tables[0][0]
        self.goal_pose_y = self.tables[0][1]
        self.best_goal_distance = 999.0
        state = self.calculate_state()
        self.init_goal_distance = state[0]
        self.prev_goal_distance = self.init_goal_distance
        response.state = state

        return response

    def call_task_succeed(self):
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task succeed is not available, waiting ...')
        future = self.task_succeed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task succeed finished')
        else:
            self.get_logger().error('task succeed service call failed')

    def call_task_failed(self):
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task failed is not available, waiting ...')
        future = self.task_failed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.current_table = 0
            self.two_tables_bonus = False
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

        goal_distance = math.sqrt(
            (self.goal_pose_x - self.robot_pose_x) ** 2
            + (self.goal_pose_y - self.robot_pose_y) ** 2)
        path_theta = math.atan2(
            self.goal_pose_y - self.robot_pose_y,
            self.goal_pose_x - self.robot_pose_x)

        goal_angle = path_theta - self.robot_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def calculate_state(self):
        state = []
        dist_norm = min(self.goal_distance / 5.0, 1.0)
        state.append(float(dist_norm))
        angle_norm = self.goal_angle / math.pi
        state.append(float(angle_norm))
        table_norm = self.current_table / max(1, len(self.tables) - 1)
        state.append(float(table_norm))
        robot_x_norm = (self.robot_pose_x + 3.5) / 7.0
        robot_y_norm = (self.robot_pose_y + 3.5) / 7.0
        state.append(float(robot_x_norm))
        state.append(float(robot_y_norm))

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

        if self.goal_distance < 0.50:
            """"
            Codigo Profesor
            self.get_logger().info('Goal Reached')
            self.succeed = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            else:
                self.cmd_vel_pub.publish(TwistStamped())
            self.local_step = 0
            self.call_task_succeed()
            """
            self.get_logger().info(
            f'Mesa {self.current_table + 1} alcanzada'
            )

            self.table_reached_this_step = True
            self.current_table += 1
            if self.current_table == 2:
                self.two_tables_bonus = True

            if self.current_table >= len(self.tables):

                self.get_logger().info(
                    'Todas las mesas servidas'
                )

                self.succeed = True
                self.done = True

                self.local_step = 0

            else:

                self.goal_pose_x = self.tables[self.current_table][0]
                self.goal_pose_y = self.tables[self.current_table][1]
                self.best_goal_distance = 999.0
                self.prev_goal_distance = self.goal_distance

                self.get_logger().info(
                    f'Nuevo objetivo: Mesa {self.current_table + 1}'
                )


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
        yaw_reward = 1.0 - abs(self.goal_angle) / math.pi
        obstacle_reward = self.compute_weighted_obstacle_reward()
        delta_dist = self.prev_goal_distance - self.goal_distance
        if delta_dist > 0:
            distance_reward = delta_dist * 15.0
        else:
            distance_reward = delta_dist * 5.0
        step_penalty = -0.02
        side_penalty = -2.0 if self.min_obstacle_distance < 0.35 else 0.0

        reward = yaw_reward + obstacle_reward + distance_reward + step_penalty + side_penalty

        if self.goal_distance < self.best_goal_distance:
            reward += 0.5
            self.best_goal_distance = self.goal_distance

        if self.table_reached_this_step:
            reward += 100.0
            self.get_logger().info('BONUS +100 por Mesa %d' % (self.current_table))
            if self.two_tables_bonus:
                reward += 50.0
                self.get_logger().info('BONUS EXTRA +50 por 2 mesas alcanzadas')
                self.two_tables_bonus = False

        if self.succeed:
            reward += 300.0
        elif self.fail:
            reward -= 50.0

        if self.local_step % 50 == 0:
            print('Step: %d, Mesa: %d, GoalDist: %.2f, GoalAngle: %.2f, Delta: %.3f, Obst: %.2f, Total: %.2f' % (
                self.local_step, self.current_table + 1,
                self.goal_distance, self.goal_angle,
                delta_dist,
                obstacle_reward,
                reward))

        self.prev_goal_distance = self.goal_distance
        return reward

    def rl_agent_interface_callback(self, request, response):
        action = request.action
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
            self.prev_goal_distance = self.init_goal_distance
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
