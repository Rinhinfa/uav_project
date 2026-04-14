from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pipeline_mode = LaunchConfiguration("pipeline_mode")
    metrics_csv = LaunchConfiguration("metrics_csv")
    run_duration_sec = LaunchConfiguration("run_duration_sec")
    event_profile_file = LaunchConfiguration("event_profile_file")
    world_sdf = LaunchConfiguration("world_sdf")
    uav_count = LaunchConfiguration("uav_count")
    start_gazebo = LaunchConfiguration("start_gazebo")
    enable_gz_motion = LaunchConfiguration("enable_gz_motion")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("pipeline_mode", default_value="proposed"),
            DeclareLaunchArgument("metrics_csv", default_value="/tmp/orchard_metrics.csv"),
            DeclareLaunchArgument("run_duration_sec", default_value="600"),
            DeclareLaunchArgument("event_profile_file", default_value=""),
            DeclareLaunchArgument("world_sdf", default_value="/tmp/orchard_world.sdf"),
            DeclareLaunchArgument("uav_count", default_value="3"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("enable_gz_motion", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="$(find-pkg-share orchard_bringup)/config/orchard_sim.rviz",
            ),
            Node(
                package="orchard_sim",
                executable="world_generator",
                name="world_generator",
                parameters=[
                    {
                        "rows": 8,
                        "trees_per_row": 24,
                        "row_spacing": 5.0,
                        "tree_spacing": 3.0,
                        "output_sdf": world_sdf,
                        "include_uav_models": True,
                        "uav_count": uav_count,
                        "uav_row_spacing": 3.0,
                        "uav_start_x": -2.0,
                        "uav_start_z": 2.0,
                    }
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            [
                                "if command -v gz >/dev/null 2>&1; then ",
                                "gz sim -r ",
                                world_sdf,
                                "; else echo '[orchard_bringup] gz command not found. "
                                "Install with: sudo apt install ros-jazzy-ros-gz'; fi",
                            ],
                        ],
                        condition=IfCondition(start_gazebo),
                    )
                ],
            ),
            TimerAction(
                period=1.2,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "ros_gz_bridge",
                            "parameter_bridge",
                            "/world/orchard_world/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                        ],
                        condition=IfCondition(enable_gz_motion),
                    )
                ],
            ),
            Node(
                package="orchard_sim",
                executable="fleet_state_publisher",
                name="fleet_state_publisher",
                parameters=[{"uav_count": uav_count, "publish_hz": 5.0}],
            ),
            Node(
                package="orchard_sim",
                executable="spawn_plan_publisher",
                name="spawn_plan_publisher",
                parameters=[
                    {
                        "uav_count": uav_count,
                        "spawn_z": 2.0,
                        "orchard_width": 69.0,
                        "orchard_height": 35.0,
                        "outside_margin": 8.0,
                        "spawn_area_side": "left",
                        "spawn_area_width": 8.0,
                        "spawn_area_height": 10.0,
                        "spawn_area_center_y": 0.0,
                        "min_pair_spacing": 2.2,
                    }
                ],
            ),
            Node(
                package="orchard_sim",
                executable="viz_publisher",
                name="viz_publisher",
                parameters=[
                    {
                        "rows": 8,
                        "trees_per_row": 24,
                        "row_spacing": 5.0,
                        "tree_spacing": 3.0,
                        "tree_radius": 0.35,
                        "orchard_width": 60.0,
                        "orchard_height": 40.0,
                        "grid_size": 2.0,
                        "uav_count": uav_count,
                    }
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=["rviz2", "-d", rviz_config],
                        condition=IfCondition(start_rviz),
                    )
                ],
            ),
            TimerAction(
                period=3.5,
                actions=[
                    Node(
                        package="orchard_sim",
                        executable="gz_path_follower",
                        name="gz_path_follower",
                        parameters=[
                            {
                                "world_name": "orchard_world",
                                "uav_count": uav_count,
                                "uav_row_spacing": 3.0,
                                "uav_start_x": -2.0,
                                "allocation_topic": "/allocation/result_json",
                                "speed_mps": 1.2,
                                "cruise_z": 2.0,
                                "transit_z": 3.85,
                                "landing_z": 0.25,
                                "max_flight_time_sec": 1800.0,
                                "trail_step_m": 0.4,
                                "trail_max_pts": 500,
                                # 树冠回避参数（与 world_generator 保持严格一致）
                                "tree_rows": 8,
                                "trees_per_row": 24,
                                "tree_row_spacing": 5.0,
                                "tree_col_spacing": 3.0,
                                "tree_radius": 0.35,
                                "canopy_height_z": 2.9,
                                "avoid_safety_margin": 0.5,
                            }
                        ],
                        condition=IfCondition(enable_gz_motion),
                    )
                ],
            ),
            Node(
                package="orchard_task_allocation",
                executable="task_allocator",
                name="task_allocator",
                parameters=[
                    {
                        "algorithm_mode": pipeline_mode,
                        "uav_speed_mps": 1.2,
                        "max_flight_time_sec": 1800.0,
                        "usable_flight_ratio": 0.9,
                        "zone_bias_weight": 0.35,
                        "transit_z": 3.85,
                        "landing_z": 0.25,
                    }
                ],
            ),
            Node(package="orchard_task_allocation", executable="dynamic_scheduler", name="dynamic_scheduler"),
            Node(
                package="orchard_task_allocation",
                executable="event_profile_player",
                name="event_profile_player",
                parameters=[{"events_file": event_profile_file}],
            ),
            Node(
                package="orchard_traj_minco",
                executable="traj_optimizer_node",
                name="traj_optimizer_node",
                parameters=[{"planner_mode": pipeline_mode}],
            ),
            Node(
                package="orchard_ego_bridge",
                executable="ego_local_planner_stub_node",
                name="ego_local_planner_stub",
                parameters=[{"planner_mode": pipeline_mode}],
            ),
            Node(
                package="orchard_ego_bridge",
                executable="planning_arbitrator_node",
                name="planning_arbitrator",
                parameters=[{"uav_count": uav_count}],
            ),
            Node(
                package="orchard_evaluation",
                executable="metrics_aggregator",
                name="metrics_aggregator",
                parameters=[{"output_csv": metrics_csv, "pipeline_mode": pipeline_mode, "uav_count": uav_count}],
            ),
            TimerAction(
                period=run_duration_sec,
                actions=[EmitEvent(event=Shutdown(reason="batch run duration reached"))],
            ),
        ]
    )
