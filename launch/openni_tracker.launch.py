#
# Copyright 2022 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # use skeleton_markers config file if provided
    launch_config_yaml = DeclareLaunchArgument('config_yaml', 
        default_value=TextSubstitution(
            text=get_package_share_directory('openni_tracker_ros2') 
                + '/config/skeleton_markers.yaml'))

    # fixed frame
    launch_fixed_frame = DeclareLaunchArgument('fixed_frame', 
        default_value=TextSubstitution(text='camera_depth_frame'))

    # used namespace for the nodes
    launch_ns = DeclareLaunchArgument('ns', 
        default_value=TextSubstitution(text='/'))

    # respawn node if exiting abnormal
    launch_respawn = DeclareLaunchArgument('respawn', 
        default_value="false")

    # launch with skeleton_marker node
    launch_marker = DeclareLaunchArgument('skeleton_marker', 
        default_value="true")

    # nodes

    tracker_node = Node(
        package='openni_tracker_ros2',
        executable='openni_tracker',
        name='openni_tracker',
        respawn=LaunchConfiguration('respawn'),
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[
            {"fixed_frame": LaunchConfiguration('fixed_frame')}
        ]
    )

    marker_node = Node(
        condition=IfCondition(LaunchConfiguration("skeleton_marker")),
        package='openni_tracker_ros2',
        executable='skeleton_markers',
        name='skeleton_markers',
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[LaunchConfiguration('config_yaml')]
    )

    return LaunchDescription([
        launch_config_yaml,
        launch_fixed_frame,
        launch_ns,
        launch_respawn,
        launch_marker,
        tracker_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=tracker_node, 
                on_start=[marker_node])
        ),
        RegisterEventHandler( # Shutdown if openni_tracker ends
            OnProcessExit(
                target_action=tracker_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='openni_tracker ended'))
                ]
            )
        )
    ])
