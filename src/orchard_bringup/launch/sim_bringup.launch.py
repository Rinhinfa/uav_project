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
                parameters=[{"uav_count": uav_count}],
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
            Node(
                package="orchard_task_allocation",
                executable="task_allocator",
                name="task_allocator",
                parameters=[{"algorithm_mode": pipeline_mode}],
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
                parameters=[{"output_csv": metrics_csv, "pipeline_mode": pipeline_mode, "uav_count": uav_count}],
            ),
            TimerAction(
                period=run_duration_sec,
                actions=[EmitEvent(event=Shutdown(reason="batch run duration reached"))],
            ),
        ]
    )
