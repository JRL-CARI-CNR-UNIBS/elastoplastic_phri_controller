from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

package_name = 'elastoplastic_lugre_controller'

def generate_launch_description():
  default_config = os.path.join(get_package_share_directory(package_name), 'config', 'adaptive_hqp.yaml')

  launch_args = [
    DeclareLaunchArgument('config', default_value=default_config, description='Controller config path'),
  ]

  launch_actions = [
    OpaqueFunction(function=launch_setup),
  ]

  return LaunchDescription(launch_args + launch_actions)

def launch_setup(context):

  controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['adaptive_hqp', '--param-file', LaunchConfiguration('config'), '--inactive']
  )

  ft_bcast_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['arm_right_ft_sensor_robotiq', '--param-file', LaunchConfiguration('config')]
  )

  return [
    controller_spawner,
    ft_bcast_spawner,
  ]
