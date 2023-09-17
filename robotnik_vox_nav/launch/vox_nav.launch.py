# Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import PushRosNamespace, Node
from launch.conditions import IfCondition
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, IncludeLaunchDescription


def generate_launch_description():
    robotnik_vox_nav_share_dir = get_package_share_directory("robotnik_vox_nav")
    xacro_path = os.path.join(robotnik_vox_nav_share_dir, "config", "robot.urdf.xacro")

    params = LaunchConfiguration("params")
    localization_params = LaunchConfiguration("localization_params")

    vox_nav_params = "vox_nav_default_params.yaml"
    robot_localization_params = "robot_localization_params_real.yaml"

    decleare_params = DeclareLaunchArgument(
        "params",
        default_value=os.path.join(
            robotnik_vox_nav_share_dir, "config", vox_nav_params
        ),
        description="Path to the vox_nav parameters file.",
    )

    decleare_localization_params = DeclareLaunchArgument(
        "localization_params",
        default_value=os.path.join(
            robotnik_vox_nav_share_dir, "config", robot_localization_params
        ),
        description="Path to the localization parameters file.",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the vox_nav",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", xacro_path]),
                "use_sim_time": False,
            }
        ],
    )

    planner_server_node = Node(
        package="vox_nav_planning",
        executable="planner_server",
        name="vox_nav_planner_server_rclcpp_node",
        namespace="",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        # prefix=['xterm -e valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],# callgrind profiling
        # prefix=['xterm -e valgrind --tool=massif -v'],    # memory profiling
        parameters=[params],
    )
    controller_server_node = Node(
        package="vox_nav_control",
        executable="vox_nav_controller_server",
        name="vox_nav_controller_server_rclcpp_node",
        namespace="",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[params],
    )
    map_server_node = Node(
        package="vox_nav_map_server",
        executable="map_manager_no_gps",
        name="map_manager_no_gps_rclcpp_node",
        namespace="",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[params],
    )
    navigate_to_pose_server_node = Node(
        package="vox_nav_navigators",
        executable="navigate_to_pose_server_node",
        name="navigate_to_pose_server_node",
        namespace="",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[params],
    )
    navigate_through_poses_server_node = Node(
        package="vox_nav_navigators",
        executable="navigate_through_poses_server_node",
        name="navigate_through_poses_server_node",
        namespace="",
        output="screen",
        parameters=[params],
    )
    navigate_through_gps_poses_server_node = Node(
        package="vox_nav_navigators",
        executable="navigate_through_gps_poses_server_node",
        name="navigate_through_gps_poses_server_node",
        namespace="",
        output="screen",
        parameters=[params],
    )

    ekf_global_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_filter_node",
        # output='screen',
        output={"both": "log"},
        parameters=[localization_params],
    )

    tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0.0 0.0 0.0 0.0 0.0 0.0 map odom".split(" "),
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
    )

    sensors_share_dir = get_package_share_directory("sensor_drivers_bringup")
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_share_dir, "launch", "bringup.launch.py")
        )
    )

    return LaunchDescription(
        [
            decleare_params,
            decleare_localization_params,
            declare_namespace_cmd,
            declare_use_namespace_cmd,
            robot_state_publisher_node,
            sensors,
            ekf_global_filter_node,
            map_server_node,
            # tf
            # planner_server_node,
            # controller_server_node,
            # navigate_to_pose_server_node,
            # navigate_through_poses_server_node,
            # navigate_through_gps_poses_server_node,
        ]
    )
