# Copyright 2026 ros2_lidar_perception contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lidar_perception')
    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz', default_value='false'),
            Node(
                package='lidar_perception',
                executable='synthetic_cloud_publisher',
                parameters=[PathJoinSubstitution([share, 'config', 'synthetic.yaml'])],
                output='screen',
            ),
            Node(
                package='lidar_perception',
                executable='perception_node',
                name='lidar_perception',
                parameters=[PathJoinSubstitution([share, 'config', 'perception.yaml'])],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', PathJoinSubstitution([share, 'rviz', 'perception.rviz'])],
                condition=IfCondition(LaunchConfiguration('rviz')),
                output='screen',
            ),
        ]
    )
