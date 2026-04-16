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
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    uav_count = LaunchConfiguration("uav_count")
    rows = LaunchConfiguration("rows")
    trees_per_row = LaunchConfiguration("trees_per_row")
    row_spacing = LaunchConfiguration("row_spacing")
    tree_spacing = LaunchConfiguration("tree_spacing")
    tree_radius = LaunchConfiguration("tree_radius")
    orchard_width = LaunchConfiguration("orchard_width")
    orchard_height = LaunchConfiguration("orchard_height")
    grid_size = LaunchConfiguration("grid_size")
    uav_speed_mps = LaunchConfiguration("uav_speed_mps")
    max_flight_time_sec = LaunchConfiguration("max_flight_time_sec")
    usable_flight_ratio = LaunchConfiguration("usable_flight_ratio")
    random_seed = LaunchConfiguration("random_seed")
    return LaunchDescription(
        [
            DeclareLaunchArgument("pipeline_mode", default_value="proposed"),
            DeclareLaunchArgument("metrics_csv", default_value="/tmp/orchard_metrics.csv"),
            DeclareLaunchArgument("run_duration_sec", default_value="360000"),
            DeclareLaunchArgument("event_profile_file", default_value=""),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="$(find-pkg-share orchard_bringup)/config/orchard_sim.rviz",
            ),
            DeclareLaunchArgument("uav_count", default_value="3"),
            DeclareLaunchArgument("rows", default_value="8"),
            DeclareLaunchArgument("trees_per_row", default_value="24"),
            DeclareLaunchArgument("row_spacing", default_value="5.0"),
            DeclareLaunchArgument("tree_spacing", default_value="3.0"),
            DeclareLaunchArgument("tree_radius", default_value="0.35"),
            DeclareLaunchArgument("orchard_width", default_value="60.0"),
            DeclareLaunchArgument("orchard_height", default_value="40.0"),
            DeclareLaunchArgument("grid_size", default_value="2.0"),
            DeclareLaunchArgument("uav_speed_mps", default_value="1.2"),
            DeclareLaunchArgument("max_flight_time_sec", default_value="1800.0"),
            DeclareLaunchArgument("usable_flight_ratio", default_value="0.9"),
            DeclareLaunchArgument("random_seed", default_value="-1"),
            Node(
                package="orchard_sim",
                executable="world_generator",
                name="world_generator",
                parameters=[
                    {
                        "rows": rows,
                        "trees_per_row": trees_per_row,
                        "row_spacing": row_spacing,
                        "tree_spacing": tree_spacing,
                        "tree_radius": tree_radius,
                        "uav_count": uav_count,
                        "output_sdf": "/tmp/orchard_world.sdf",
                    }
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
                        "orchard_width": orchard_width,
                        "orchard_height": orchard_height,
                        "random_seed": random_seed,
                    }
                ],
            ),
            Node(
                package="orchard_sim",
                executable="viz_publisher",
                name="viz_publisher",
                parameters=[
                    {
                        "rows": rows,
                        "trees_per_row": trees_per_row,
                        "row_spacing": row_spacing,
                        "tree_spacing": tree_spacing,
                        "tree_radius": tree_radius,
                        "orchard_width": orchard_width,
                        "orchard_height": orchard_height,
                        "grid_size": grid_size,
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
            Node(
                package="orchard_task_allocation",
                executable="task_allocator",
                name="task_allocator",
                parameters=[
                    {
                        "algorithm_mode": pipeline_mode,
                        "task_rows": rows,
                        "tasks_per_row": trees_per_row,
                        "task_row_spacing": row_spacing,
                        "task_col_spacing": tree_spacing,
                        "uav_speed_mps": uav_speed_mps,
                        "max_flight_time_sec": max_flight_time_sec,
                        "usable_flight_ratio": usable_flight_ratio,
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
            Node(package="orchard_ego_bridge", executable="planning_arbitrator_node", name="planning_arbitrator"),
            Node(
                package="orchard_evaluation",
                executable="metrics_aggregator",
                name="metrics_aggregator",
                parameters=[
                    {
                        "output_csv": metrics_csv,
                        "pipeline_mode": pipeline_mode,
                        "uav_count": uav_count,
                        "orchard_width": orchard_width,
                        "orchard_height": orchard_height,
                        "grid_size": grid_size,
                    }
                ],
            ),
            TimerAction(
                period=run_duration_sec,
                actions=[EmitEvent(event=Shutdown(reason="batch run duration reached"))],
            ),
        ]
    )
