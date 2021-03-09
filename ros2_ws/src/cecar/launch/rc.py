#!/usr/bin/python

#Remote control launch file for CeCar as part of my Bachelor's paper
#Author(s)		: Lukas Mirow
#Date of creation	: 7/27/2020

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='cecar',
			node_namespace='cecar1',
			node_executable='gamepad_rc',
			node_name='gamepad_rc'
		)
	])
