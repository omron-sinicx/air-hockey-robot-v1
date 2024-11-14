import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='camera_driver',
            name='camera_driver',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='frame_analyzer',
            name='frame_analyzer',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='mallet_detector',
            name='mallet_detector',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='robo_arm_detector',
            name='robo_arm_detector',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='puck_movement_analyzer',
            name='puck_movement_analyzer',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='mallet_motion_analyzer',
            name='mallet_motion_analyzer',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='robo_arm_motion_analyzer',
            name='robo_arm_motion_analyzer',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='object_motion_predictor',
            name='object_motion_predictor',
            prefix='xterm -e',
            output='screen'
        ) 
    ])
