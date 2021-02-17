from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():        
        return LaunchDescription([
            #Define launch arguments for parameterization of launchConfiguration from command line                         
            DeclareLaunchArgument('ns', description='Namespace to launch nodes in.'),
            DeclareLaunchArgument('topic_name', default_value='hello_world_topic', description='Name of topic the nodes should use to communicate.'),
            
            Node(# Launch listener node first
                package='node-configuration_example',
                node_executable='listener',
                node_namespace=LaunchConfiguration('ns'),   #use launch argument for setting node namespace
                output='screen',    #change to 'log' to receive node output in a log-file
                remappings=[('topic', LaunchConfiguration('topic_name'))]   #use launch argument for topic remapping
            ),

            Node(# Launch talker node
                package='node-configuration_example',
                node_executable='talker',
                node_namespace=LaunchConfiguration('ns'),   #use launch argument for setting node namespace
                output='screen',    #change to 'log' to receive node output in a log-file
                remappings=[('topic', LaunchConfiguration('topic_name'))]   #use launch argument for topic remapping             
            )
                
        ])