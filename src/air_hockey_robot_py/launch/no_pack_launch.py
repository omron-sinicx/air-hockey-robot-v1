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
            executable='object_detector',
            name='object_detector',
            prefix='xterm -e',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='velocity_estimator',
            name='velocity_estimator',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='object_predictor',
            name='object_predictor',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='communicate_motors',
            name='communicate_motors',
            prefix='xterm -e',
            output='screen'
        ), 
        launch_ros.actions.Node(
            package='air_hockey_robot',
            executable='arm_motion_planner',
            name='arm_motion_planner',
            prefix='xterm -e',
            output='screen'
        ) 
    ])