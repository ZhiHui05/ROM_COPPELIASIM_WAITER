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
# # Authors: Ryan Shim, Gilbert, ChanHyeong Lee

import os
import random
import subprocess
import sys
import time
import json

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Goal


ROS_DISTRO = os.environ.get('ROS_DISTRO')
if ROS_DISTRO == 'humble':
    from std_srvs.srv import Trigger
    from std_srvs.srv import Empty
    from geometry_msgs.msg import Pose


class CoppeliaSimInterface(Node):

    def __init__(self):
        super().__init__('coppeliasim_interface')

        self.entity_name = 'goal_box'
        self.entity_pose_x = 0.25
        self.entity_pose_y = 0.0

        self.new_goal_client = self.create_client(Trigger, 'new_goal')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.initialize_env_service = self.create_service(
            Goal,
            'initialize_env',
            self.initialize_env_callback,
            callback_group=self.callback_group
        )
        self.task_succeed_service = self.create_service(
            Goal,
            'task_succeed',
            self.task_succeed_callback,
            callback_group=self.callback_group
        )
        self.task_failed_service = self.create_service(
            Goal,
            'task_failed',
            self.task_failed_callback,
            callback_group=self.callback_group
        )
        
    def new_goal(self):
        new_goal_req = Trigger.Request()
        while not self.new_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for new goal is not available, waiting ...')
        future = self.new_goal_client.call_async(new_goal_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            #print(f'{Response: {response.message}')
            data = json.loads(response.message)
            self.entity_pose_x = data["x"]
            self.entity_pose_y = data["y"]
            self.get_logger().info('service for task succeed finished')
        else:
            self.get_logger().error('task succeed service call failed')

    def reset_simulation(self):
        reset_req = Empty.Request()

        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for reset_simulation is not available, waiting ...')

        self.reset_simulation_client.call_async(reset_req)

    def task_succeed_callback(self, request, response):
        self.new_goal()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

    def task_failed_callback(self, request, response):
        self.reset_simulation()
        time.sleep(0.2)
        self.new_goal()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

    def initialize_env_callback(self, request, response):
        self.reset_simulation()
        time.sleep(0.2)
        self.new_goal()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=sys.argv)
    coppeliasim_interface = CoppeliaSimInterface()
    try:
        while rclpy.ok():
            rclpy.spin_once(coppeliasim_interface, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        coppeliasim_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
