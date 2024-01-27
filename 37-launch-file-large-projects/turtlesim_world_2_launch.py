import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_tutorial'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
# REMOVED - Specify namespace in launch file 
         # namespace='turtlesim2',
# REMOVED END - Specify namespace in launch file 
         name='sim',
         parameters=[config]
      )
   ])