#!/usr/bin/python

#Remote control launch file for CeCar as part of my Bachelor's paper
#Author(s)		: Lukas Mirow
#Date of creation	: 7/27/2020
#Modified for parameterization of namespace by Philipp Jass (3/9/2021)

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
	'gamepad_parameters.yaml'
	),

	return LaunchDescription([
		#Define launch arguments for parameterization of launchConfiguration from command line                         
        DeclareLaunchArgument('ns', description='Namespace to launch nodes in.'),

		Node(
			package='cecar_base',
			node_executable='gamepad_rc',
			node_namespace=LaunchConfiguration('ns'),   #use launch argument for setting node namespace,
			output='screen',    #change to 'log' to receive node output in a log-file
			parameters=[config]
		)
	])
