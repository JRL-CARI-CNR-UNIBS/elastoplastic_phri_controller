from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = 'elastoplastic_lugre_controller'

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument('config-pkg', default_value=package_name, description='Controller config pkg'),
    DeclareLaunchArgument('config-path', default_value='config/controller_config.yaml', description='Controller config path from pkg'),
  ]

  launch_actions = [
    OpaqueFunction(function=launch_setup),
  ]

  return LaunchDescription(launch_args + launch_actions)

def launch_setup(context):

  controller_config = PathJoinSubstitution([FindPackageShare(LaunchConfiguration('config-pkg')), LaunchConfiguration('config-path')])

  controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['elastoplastic_controller', '--param-file', controller_config, '--inactive']
  )

  return [
    controller_spawner
  ]
