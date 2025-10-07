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

  launch_args = [
  ]

  launch_actions = [
    OpaqueFunction(function=launch_setup),
  ]

  return LaunchDescription(launch_args + launch_actions)

def launch_setup(context):

  support_package = 'tiago_pro_elastoplastic'

  generate_cart_trj_node = Node(
    package=support_package,
    executable='generate_cartesian_trajectory',
    parameters=[{
      'frame' : 'robotiq_ft_frame_id',
      'world_frame' : 'map',
      'axis' : [1,1,1,0,0,0],
    }],
  )

  publish_trj_tf_node = Node(
    package=support_package,
    executable='publish_tf_trajectory',
  )

  return [
    generate_cart_trj_node,
    publish_trj_tf_node,
  ]
