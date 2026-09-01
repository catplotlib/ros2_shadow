"""Two real Nav2 planners on one map, compared live.

Production runs NavFn, the candidate runs Smac 2D. Same map, same costmap
settings, same goals. No simulator: the probe supplies start poses explicitly,
so this exercises the planners themselves.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_MAPS = os.path.join(get_package_share_directory("nav2_bringup"), "maps")


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("ros2_shadow"), "params", "nav2_shadow_planners.yaml"
    )
    map_yaml = LaunchConfiguration("map")

    planners = [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace=namespace,
            parameters=[params],
            # A namespaced node resolves the relative "tf" topic to
            # /<namespace>/tf and never sees the global transform tree, so the
            # costmap waits on a transform that will never arrive. Both
            # planners share one tree here, so point them back at it.
            remappings=[("/tf", "/tf"), ("tf", "/tf"),
                        ("/tf_static", "/tf_static"), ("tf_static", "/tf_static")],
            output="screen",
        )
        for namespace in ("production", "candidate")
    ]

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(NAV2_MAPS, "warehouse.yaml")),

        # The planners need a pose for the robot base. Nothing is driving, so a
        # fixed transform is enough to let the costmaps come up.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="map_to_odom", output="log",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="odom_to_base", output="log",
             arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]),

        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[{"yaml_filename": map_yaml}], output="screen"),

        *planners,

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager", output="screen",
             parameters=[{
                 "autostart": True,
                 "bond_timeout": 60.0,
                 "node_names": ["map_server",
                                "/production/planner_server",
                                "/candidate/planner_server"],
             }]),

        Node(package="ros2_shadow", executable="nav2_probe", name="nav2_probe",
             output="screen"),
    ])
