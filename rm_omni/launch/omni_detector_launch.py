# Copyright 2023 Yunlong Feng
# Copyright 2025 Lihan Chen
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("rm_omni")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    container_name = LaunchConfiguration("container_name")
    use_external_container = LaunchConfiguration("use_external_container")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1", description="Namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "omni_detector.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="omni_detector_container",
        description="Container name",
    )

    declare_use_external_container_cmd = DeclareLaunchArgument(
        "use_external_container",
        default_value="false",
        description="Use external container",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "omni_default_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    # Create container node
    container_node = Node(
        name=container_name,
        package="rclcpp_components",
        executable="component_container_isolated",
        output="screen",
        condition=UnlessCondition(use_external_container),
    )

    load_detector = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="rm_omni",
                plugin="rm_omni::OmniNode",
                name="omni_detector",
                parameters=[configured_params, {"use_sim_time": use_sim_time}],
                namespace=namespace,
            )
        ],
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_config_file],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            (
                "/front_industrial_camera/camera_info",
                "front_industrial_camera/camera_info",
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_external_container_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(container_node)
    ld.add_action(load_detector)
    ld.add_action(start_rviz_cmd)

    return ld
