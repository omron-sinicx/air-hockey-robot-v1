from launch import LaunchDescription, actions
from launch_ros.actions import Node

def generate_launch_description():
    # env = {"DISPLAY": ":1"}
    return LaunchDescription([
        Node(
            package='air_hockey_robot',
            namespace='air_hockey_robot',
            executable='camera_driver',
            name='camera_driver',
            arguments=[('__log_level:=debug')]
        ),
        actions.TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='air_hockey_robot',
                    namespace='air_hockey_robot',
                    executable='camera_driver',
                    name='camera_driver',
                    output='screen',
                    # env=env,
                    arguments=[('__log_level:=debug')]
                ),
                Node(
                    package='air_hockey_robot',
                    namespace='air_hockey_robot',
                    executable='object_detector',
                    name='object_detector',
                    output='screen',
                    # env=env,
                    arguments=[('__log_level:=debug')]
                ),
                Node(
                    package='air_hockey_robot',
                    namespace='air_hockey_robot',
                    executable='velocity_estimator',
                    name='velocity_estimator',
                    arguments=[('__log_level:=debug')]
                ),
                Node(
                    package='air_hockey_robot',
                    namespace='air_hockey_robot',
                    executable='object_predictor',
                    name='object_predictor',
                    arguments=[('__log_level:=debug')]
                ),
                Node(
                    package='air_hockey_robot',
                    namespace='air_hockey_robot',
                    executable='motor_controller',
                    name='motor_controller',
                    arguments=[('__log_level:=debug')]
                ),
                Node(
                    package='air_hockey_robot',
                    namespace='air_hockey_robot',
                    executable='arm_motion_planner',
                    name='arm_motion_planner',
                    arguments=[('__log_level:=debug')]
                )
            ]
        )
    ])
                # Node(
                #     package='air_hockey_robot',
                #     namespace='air_hockey_robot',
                #     executable='display_image',
                #     name='display_image',
                #     output='screen',
                #     # env=env,
                #     arguments=[('__log_level:=debug')]
                # ),
                # Node(
                #     package='air_hockey_robot',
                #     namespace='air_hockey_robot',
                #     executable='frame_analyzer',
                #     name='frame_analyzer',
                #     output='screen',
                #     # env=env,
                #     arguments=[('__log_level:=debug')]
                # ),
                # Node(
                #     package='air_hockey_robot',
                #     namespace='air_hockey_robot',
                #     executable='puck_movement_analyzer',
                #     name='puck_movement_analyzer',
                #     arguments=[('__log_level:=debug')]
                # ),
                # Node(
                #     package='air_hockey_robot',
                #     namespace='air_hockey_robot',
                #     executable='motor_controller',
                #     name='motor_controller',
                #     arguments=[('__log_level:=debug')]
                # ),
                # Node(
                #     package='air_hockey_robot',
                #     namespace='air_hockey_robot',
                #     executable='game_controller',
                #     name='game_controller',
                #     arguments=[('__log_level:=debug')]
