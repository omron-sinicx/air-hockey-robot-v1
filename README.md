# Air hockey robot (v1)

## Low-cost air hockey robot for human-robot-interaction research

M. Shinjo, C. C. Beltran-Hernandez, M. Hamaya, and K. Tanaka, 
Low-Cost Air Hockey Robot Using a Five-Bar Linkage Mechanism Driven by Position-Control Servomotors,
The 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024), 2024.

[Project page](https://omron-sinicx.github.io/air-hockey-robot-v1/) 

[Paper](https://www.researchgate.net/publication/384469803_Low-Cost_Air_Hockey_Robot_Using_a_Five-Bar_Linkage_Mechanism_Driven_by_Position-Control_Servomotors)

## Getting started

Install and setup `ros2`. 

### Build

```
colcon build
```

### Setup
Before running codes, setup as  

```
. install/setup.bash
```

### Run

Open *multiple* terminals. 
Run following commands in *each* terminal. 
These node can be used simultaneously making and using `ros2 launch` files.  

```
ros2 run air_hockey_robot camera_driver 
```

```
ros2 run air_hockey_robot object_detector 
```

```
ros2 run air_hockey_robot velocity_estimator 
```

```
ros2 run air_hockey_robot object_predictor 
```

```
python3 src/air_hockey_robot_py/air_hockey_robot_py/motor_communication.py 
```

```
python3 src/air_hockey_robot_py/air_hockey_robot_py/arm_motion_planner.py 
```

### TIPS
#### Motion generation type (hit back motion, block motion, random motion, ...)
Change `self.mode` in `arm_motion_planner.py`.

#### Camera calibration
Get and edit `CAMERA_MATRIX` and `DISTORTION_COEFFICIENTS` in `air_hockey_robot_config.hpp`.

### Camera and robot coordinate convertion
Get and edit `PUCK_KX_F`, `PUCK_KY_F`, ..., `PUCK_K_W`, `HUMAN_KX_F`, `HUMAN_KY_F`, ..., `HUMAN_K_W` in `air_hockey_robot_config.hpp`. 

Parameters can be loaded making and using `ros2 param` files.

#### Motor communication debug

Send commands as 

```
$ ros2 topic pub --once /motor_commands air_hockey_robot/msg/MotorCommands "{goal_position_id1: 2600, goal_position_id2: 2600, goal_speed_id1: 0, goal_speed_id2: 0, header:{stamp: {sec: 0, nanosec: 0}, frame_id: 0}}"
```


## Hardware

`.stp` design files in `cad`.


## Citation

```
@inproceedings{shinjo2024low,
    title={Low-Cost Air Hockey Robot Using a Five-Bar Linkage Mechanism Driven by Position-Control Servomotors},
    author={Shinjo, Mirai and Beltran-Hernandez, Cristian C and Hamaya, Masashi and Tanaka, Kazutoshi},
    booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    organization={IEEE}, 
    year={2024}
}
```

