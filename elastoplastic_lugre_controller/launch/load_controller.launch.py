# This Python file uses the following encoding: utf-8

from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument("ns",     default_value=None, description="controller_manager namespace"),
  ]

  return LaunchDescription(launch_args + OpaqueFunction(function=launch_setup))

def launch_setup(context):
  ns_arg = LaunchConfiguration("ns")
  if ns_arg.perform(context) is None:
    ns = ""
  else:
    ns = ns_arg.perform(context)

  controller_config = PathJoinSubstitution([FindPackageShare("elastoplastic_lugre_controller"), "config", "elastoplastic.yaml"])
  # cm_new_controller = {f"{ns}/controller_manager/elastoplastic_controller/type" : "phri/control/CartImpedanceLuGreController"}

  set_cm_new_controller = Command(f"ros2 param set {ns}/controller_manager elastoplastic_controller.type phri/control/CartImpedanceLuGreController")

  spawn_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["elastoplastic_lugre_controller",
               "-c",f"{ns}/controller_manager",
               "-p",controller_config
               ]
  )

  return [
    set_cm_new_controller,
    spawn_controller,
  ]
