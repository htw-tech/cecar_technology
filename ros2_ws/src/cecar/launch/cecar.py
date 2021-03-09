#!/usr/bin/python

#Launch file for CeCar as part of my Bachelor's paper
#Author(s)		: Lukas Mirow
#Date of creation	: 7/27/2020

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([

		Node(
			package='cecar',
			node_namespace='cecar1',
			node_executable='drive',
			node_name='drive'
		),

		Node(
			package='cecar',
			node_namespace='cecar1',
			node_executable='self_protection',
			node_name='self_protection'
		),

		Node(
			package='cecar',
			node_namespace='cecar1',
			node_executable='self_control',
			node_name='self_control'
		),

		Node(
			package='cecar',
			node_namespace='cecar1',
			node_executable='intelligence_collection',
			node_name='intelligence_collection'
		),

		Node(
			package='cecar',
			node_namespace='cecar1',
			node_executable='rcu_comm',
			node_name='rcu_comm',
			parameters=['cecar/launch/parameters.yaml'] #FIXME: `ros2 launch` needs to be run in the correct directory for this to work
		)

	])
