#!/usr/bin/python

#Launch file for CeCar as part of my Bachelor's paper
#Modified for parameterization of namespace: Philipp Jass (3/9/2021)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	config = os.path.join(
	get_package_share_directory('cecar_base'),
	'launch',
	'cecar_parameters.yaml'
	),

	return LaunchDescription([
		#Define launch arguments for parameterization of launchConfiguration from command line                         
        DeclareLaunchArgument('ns', description='Namespace to launch nodes in.'),

		Node(
			package='cecar_base',
			node_namespace=LaunchConfiguration('ns'),
			node_executable='drive',
			output='screen',
                        parameters = [config]
		),

		Node(
			package='cecar_base',
			node_namespace=LaunchConfiguration('ns'),
			node_executable='self_protection',
			output='screen',
                        parameters = [config]
		),

		Node(
			package='cecar_base',
			node_namespace=LaunchConfiguration('ns'),
			node_executable='self_control',
			output='screen',
                        parameters = [config]
		),

		Node(
			package='cecar_base',
			node_namespace=LaunchConfiguration('ns'),
			node_executable='intelligence_collection',
			output='screen',
                        parameters = [config]
		),

		Node(
			package='cecar_base',
			node_namespace=LaunchConfiguration('ns'),
			node_executable='rcu_comm',
			output='screen',
                        parameters = [config]
		)

	])
