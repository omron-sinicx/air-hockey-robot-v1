#!/usr/bin/env python3

# import time
import rclpy
from rclpy.node import Node
from air_hockey_robot.msg import ArmStatus
from air_hockey_robot.msg import ObjectPrediction
from air_hockey_robot.msg import MotorCommands
from air_hockey_robot.msg import TargetPoint
import numpy as np
import copy

counter = 0
MIN_MOTOR_POSITION = 950
MAX_MOTOR_POSITION = 3050
MOTOR_VELOCITY_RPM = 0.229 # rev/min
MOTOR_VELOCITY_RPMS = MOTOR_VELOCITY_RPM / (60 * 1000) # 60 * 1000 a constant
MOTOR_ENCODER_RESOLUTION = 4096
MOTOR_POSITION_OFFSET = np.pi
LINK_LENGTH = (200, 200)
CHECK_STEP = 12
D_VALID = 30.0 # [mm]
D_CROSS = 25.0 # [mm]
Y_MIN = -115.0 # [mm]
Y_MAX = +95.0 
X_MIN = 200.0 # [mm]
X_MIN_HIT = 230.0 # [mm]
X_MAX = 300.0 
ARM1_LINK1_LENGTH = 200
ARM1_LINK2_LENGTH = 200
D_REACH = 5.0 # [mm]
D_HIT = 20.0 # [mm]
D_VIA = 70.0 # [mm]
VELOCITY_LIMIT = 2047
D_BACK = 30.0 # [mm]
XD_HIT = 18.0 + 25.0 # [mm]
VELOCITY_MIN = 300
D_MIN = 50.0

class ArmMotionPlanner(Node):

    def __init__(self):
        super().__init__('arm_motion_planner')
        self.object_motion_prediction_subscriber = self.create_subscription(ObjectPrediction, 'puck_prediction', self.prediction_callback, 5)
        self.arm_status_subscriber = self.create_subscription(ArmStatus, 'arm_status', self.arm_status_callback, 5)
        self.command_publisher = self.create_publisher(MotorCommands, 'motor_commands', 5)
        self.target_publisher = self.create_publisher(TargetPoint, 'target_point', 5) # Taget mallet location for debug
        self.target_publisher2 = self.create_publisher(TargetPoint, 'target_point2', 5)
        self.object_prediction = None
        self.object_prediction_old = None
        self.arm_status = None
        self.future = None
        self.get_logger().info('init')
        # self.initial_point = [200.0, 0.0] # block middle
        self.initial_point = [250.0, 0.0] # block high
        # self.initial_point = [150.0, 0.0] # block low
        self.target_point = copy.deepcopy(self.initial_point)
        self.via_point = [170.0, -50.0]
        self.target_now = self.initial_point
        self.counter = 0
        self.initial_pos = self.point2angle(self.initial_point)
        self.target_pos = self.point2angle(self.target_point)
        self.snipe_mode = 'initial'
        self.snipe_goal = [509.8, 56.5] # right
        # self.snipe_goal = [579.9, 0.0] # left
        self.counter_ready = 0
        self.counter_hit = 0
        self.vel_motor = [0, 0]
        self.vel_target = [0, 0]
        self.send_commands(self.initial_pos, self.vel_motor)
        # self.mode = "block"
        self.mode = "four"
        
    # Target mallet location (point) -> target motor angles (goal_motor_pos)   
    def point2angle(self, point):
        theta = inverse_kinematics_2dof(LINK_LENGTH, point[0], point[1])
        goal_motor_pos = np.array(motor_position(theta))
        return goal_motor_pos

    # plan motion
    def prediction_callback(self, msg):
        # global counter
        self.object_prediction_old = self.object_prediction
        self.object_prediction = msg
        # print(counter)
        if self.mode == "block":
            # Block motion
            self.cross_point()
        elif self.mode == "snipe":
            # Hit back motion
            self.snipe()
        elif self.mode == "four":
            # Mallet speed evaluation with four constant target points
            self.four_points()
        elif self.mode == "random":
            # Mallet speed evaluation with random target points
            self.random_points()
        else:
            self.dummy_prediction()
        # self.two_points()
        self.counter = self.counter + 1

    # Plan motion using dummy prediction
    def dummy_prediction(self):
        # y = 0.0
        v = -852.0
        w = -200.0
        x0 = 579.0
        y0 = 70.0
        dt = 1.0 / 120.0
        num = len(self.object_prediction.trajectory)
        sec = self.object_prediction.trajectory[0].time.sec
        nanosec = self.object_prediction.trajectory[0].time.nanosec
        t0 = sec + np.clip(nanosec*1e-9, 0, 1)
        counter = int(self.object_prediction.header.frame_id)
        m = np.mod(counter, 90)
        t = float(m) 
        xs = []
        ys = []
        if t < 0.5:
            x1 = x0 + v * t
            y1 = y0 + w * t
            for n in range(num):
                t_ = dt * float(n + 1) 
                x = x1 + v * t_
                y = y1 + w * t_
                xs.append(x)
                ys.append(y)
        else:
            for n in range(num):
                xs.append(0.0)
                ys.append(0.0)
        for n in range(num):
            t2 = t0 + dt * float(n + 1) 
            sec = int(t2/ 1e+9)
            nanosec = int(np.clip(t2 - sec, 0, 1) * 1e+9)
            self.object_prediction.trajectory[n].point.x = xs[n]
            self.object_prediction.trajectory[n].point.y = ys[n]
            self.object_prediction.trajectory[n].time.sec = sec
            self.object_prediction.trajectory[n].time.nanosec = nanosec

    # Subscribe arm status topic
    def arm_status_callback(self, msg):
        self.arm_status = msg

    # Plan hit back motion
    def plan_hit_back(self):
        if self.arm_status is None:
            print("no motor")
            if self.object_prediction is not None:
                self.send_commands(self.initial_pos, self.vel_motor)
            return None, None
        x_puck = np.array([p.point.x for p in self.object_prediction.trajectory])
        y_puck = np.array([p.point.y for p in self.object_prediction.trajectory])
        t_puck = np.array([p.time.sec + np.clip(p.time.nanosec*1e-9, 0, 1) for p in self.object_prediction.trajectory])
        x_robot, y_robot = self.angle2point()
        xd_back = self.snipe_goal[0] - x_robot
        xd_puck = x_puck - x_robot
        yd_puck = y_puck - y_robot
        yd_back = (yd_puck / xd_puck) * xd_back
        y_back = y_robot + yd_back
        x_hit = x_puck + XD_HIT
        yd_hit = XD_HIT * (yd_puck / xd_puck)
        y_hit = y_puck + yd_hit
        d = np.sqrt((x_robot - x_hit)**2 + (y_robot - y_hit)**2)
        k = np.where((x_hit > X_MIN) & (x_hit < X_MAX) & (y_hit > Y_MIN) & (y_hit < Y_MAX) & (d > D_MIN))[0]
        if k.size == 0:
            # print("no k")
            return None, None
        else:
            # print("y_back min:{}, max:{}".format(np.min(y_back[k]), np.max(y_back[k])))
            l = np.argmin(np.abs(y_back[k] - self.snipe_goal[1]))
            d = np.min(np.abs(y_back[k] - self.snipe_goal[1]))
            if d > D_BACK:
                # print("d over. d={}".format(d))
                return None, None
            else:
                print("hit back")
                self.send_target([x_puck[k[l]], y_puck[k[l]]])
                self.send_target2([x_hit[k[l]], y_hit[k[l]]])
                return [x_hit[k[l]], y_hit[k[l]]], t_puck[k[l]]

    # Plan hit back motions
    def snipe(self):
        if self.arm_status is None:
            print("no motor")
            self.send_commands(self.initial_pos, self.vel_motor)
        if self.snipe_mode == 'initial':
            is_go, target = self.plan_hit()
            if is_go:
                print("to move to via point")
                self.snipe_mode = 'ready'
                self.counter_ready = self.counter
                self.target_point = copy.deepcopy(target)
                self.vel_motor = copy.deepcopy([0, 0])
        elif self.snipe_mode == 'ready':
            if True:
                print("reachd via point")
                target, t = self.plan_hit_back()
                if t is not None:
                    # print("t found")
                    target_old = copy.deepcopy(self.target_point)
                    self.target_point = copy.deepcopy(target)
                    print("self.target_point={}".format(self.target_point))
                    vel = self.move2velocity(t)
                    if vel is not None:
                        print("start hitting")
                        self.target_point = copy.deepcopy(target)
                        self.vel_motor = copy.deepcopy(vel)
                        self.snipe_mode = 'hit'
                        self.counter_hit = self.counter
                        # self.target_now = copy.deepcopy(self.target_point)
                    else:
                        # print("vel not found")
                        self.target_point = copy.deepcopy(target_old)
            if self.counter > self.counter_ready + 10:
                print("back inital point")
                self.snipe_mode = 'initial'
                self.target_point = copy.deepcopy(self.initial_point)
                self.vel_motor = copy.deepcopy([0, 0])
        else: # hit
            target, t = self.plan_hit_back()
            if t is not None:
                self.target_point = copy.deepcopy(target)
            if self.counter - self.counter_hit > 30: 
                self.target_point = copy.deepcopy(self.initial_point)
                self.vel_motor = copy.deepcopy([0, 0])
                self.snipe_mode = 'initial'
        pos = self.point2angle(self.target_point)
        self.send_commands(pos, self.vel_motor)
        
    def move2velocity(self, time):
        theta = inverse_kinematics_2dof(LINK_LENGTH, self.target_point[0], self.target_point[1]) # [rad]
        goal_motor_pos = np.array(motor_position(theta))
        if self.arm_status is None:
            return None
        # [VELOCITY_LIMIT, VELOCITY_LIMIT]
        p1 = self.arm_status.id1.present_position
        p2 = self.arm_status.id2.present_position
        dp1_val = np.abs(p1 - goal_motor_pos[0])
        dp2_val = np.abs(p2 - goal_motor_pos[1])
        dp1_deg = dp1_val * (360.0/4096.0)
        dp2_deg = dp2_val * (360.0/4096.0)
        dp1_rad = dp1_deg * (np.pi / 180.0)
        dp2_rad = dp2_deg * (np.pi / 180.0)
        vel_rads = np.array([dp1_rad, dp2_rad]) / time # [rad/s]
        # print("vel_rads={}".format(vel_rads))
        vel_rpm = 60.0 * vel_rads / (2.0*np.pi)
        # print("vel_rpm={}".format(vel_rpm))
        vel_val = vel_rpm / MOTOR_VELOCITY_RPM
        # print("vel_val={}".format(vel_val))
        vel = vel_val.astype(np.int32).tolist()
        if np.max(np.abs(vel)) < VELOCITY_LIMIT:
            if np.min(vel) > VELOCITY_MIN:
                return vel
            else:
                return [VELOCITY_MIN, VELOCITY_MIN]
                # return [200, 200]
        else:
            return None

    def plan_hit(self):
        if self.arm_status is None:
            print("no motor")
            return False, None
        x_puck = np.array([p.point.x for p in self.object_prediction.trajectory])
        y_puck = np.array([p.point.y for p in self.object_prediction.trajectory])
        xd_puck = x_puck - self.snipe_goal[0]
        yd_puck = y_puck - self.snipe_goal[1]
        vx = xd_puck / np.sqrt(xd_puck** 2 + yd_puck**2)
        vy = yd_puck / np.sqrt(xd_puck** 2 + yd_puck**2)
        x_via = x_puck + vx * D_VIA
        y_via = y_puck + vy * D_VIA
        x_robot, y_robot = self.angle2point()
        d_ready = np.sqrt((x_via - x_robot)**2 + (y_via - y_robot)**2)
        k = np.where((x_puck > X_MIN) & (x_puck < X_MAX) & (y_puck > Y_MIN) & (y_puck < Y_MAX) & (x_via > X_MIN) & (x_via < X_MAX) & (y_via > Y_MIN) & (y_via < Y_MAX))[0]
        if k.size == 0:
            # print("no k")
            # print(vx[:5])
            # print(vy[:5])
            # print(j0.size, j1.size)
            return False, None
        else:
            l = np.argmin(d_ready[k])
            # print("vx={}, yy={}".format(vx[k[l]], vy[k[l]]))
            self.send_target([x_via[k[l]], y_via[k[l]]])
            self.send_target2([x_puck[k[l]], y_puck[k[l]]])
            return True, [x_via[k[l]], y_via[k[l]]]
            
    # Plan block motions
    def cross_point(self):
        x_line = 260.0
        if (not (self.object_prediction_old is None)) and (not (self.object_prediction is None)) and self.is_valid_trajectory() and (not (self.arm_status is None)):
            point, _ = self.detect_cross_point(x_line)
            vel = [0, 0]
            if point is not None:
                # print([point])
                self.target_point = copy.deepcopy(point)
            else:
                self.target_point = copy.deepcopy(self.initial_point)
            pos = self.point2angle(self.target_point)
            self.send_commands(pos, vel)

    # Set motor angle and puclish motor commands (via point)
    def send_target(self, point):
        target = TargetPoint()
        target.x = point[0]
        target.y = point[1]
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.object_prediction.header.frame_id
        self.target_publisher.publish(target)

    # Set motor angle and puclish motor commands (target point)
    def send_target2(self, point):
        target = TargetPoint()
        target.x = point[0]
        target.y = point[1]
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.object_prediction.header.frame_id
        self.target_publisher2.publish(target)

    # Predicted puck locations include noise or not using one puck step length > D_VALID or not.
    def is_valid_trajectory(self):
        x1 = np.array([p.point.x for p in self.object_prediction.trajectory])
        y1 = np.array([p.point.y for p in self.object_prediction.trajectory])
        x0 = np.array([p.point.x for p in self.object_prediction_old.trajectory])
        y0 = np.array([p.point.y for p in self.object_prediction_old.trajectory])
        t1 = np.array([p.time.sec + np.clip(p.time.nanosec*1e-9, 0, 1) for p in self.object_prediction.trajectory]) + self.object_prediction.header.stamp.sec + np.clip(self.object_prediction.header.stamp.nanosec*1e-9, 0, 1)
        t0 = np.array([p.time.sec + np.clip(p.time.nanosec*1e-9, 0, 1) for p in self.object_prediction.trajectory]) + self.object_prediction_old.header.stamp.sec + np.clip(self.object_prediction_old.header.stamp.nanosec*1e-9, 0, 1)
        k = np.argmin(np.abs(t0 - t1[CHECK_STEP]))
        xd = x1[CHECK_STEP] - x0[k]       
        yd = y1[CHECK_STEP] - y0[k]       
        d = np.sqrt(xd**2 + yd**2)
        if d < D_VALID:
            return True
        else:
            return False

    # Detect cross point with x_line
    def detect_cross_point(self, x_line):
        x1 = np.array([p.point.x for p in self.object_prediction.trajectory])
        y1 = np.array([p.point.y for p in self.object_prediction.trajectory])
        t1 = np.array([p.time.sec + np.clip(p.time.nanosec*1e-9, 0, 1) for p in self.object_prediction.trajectory])
        x_valid = np.ones(x1.size)
        g = np.where(x1 < x_line)[0]
        if g.size > 0:
            x_valid[g[0]:] = 0
        n = np.where((y1 > Y_MIN) & (y1 < Y_MAX) & (x_valid > 0))[0]
        if n.size == 0:
            # print('over y limit')
            return None, None
        m = np.argmin(np.abs(x1[n] - x_line))
        k = n[m]
        if np.abs(x1[k] - x_line) > D_CROSS:
            # print('over x limit')
            return None, None
        else:
            # print('cross point found')
            return [x1[k], y1[k]], t1[k]

    # Mallet speed evaluation with four constant target points
    def four_points(self):
        xd = 80
        yd = 80
        xc = 250
        yc = 0 
        initial_point = [xc, yc]
        target_point1 = [xc - xd, yc - yd]
        target_point2 = [xc + xd, yc - yd]
        target_point3 = [xc + xd, yc + yd]
        target_point4 = [xc - xd, yc + yd]
        self.vel_motor = [0, 0]
        if np.mod(self.counter, 30) < 15:
            self.target_pos = self.point2angle(initial_point)
        elif np.mod(self.counter, 120) < 30:
            self.target_pos = self.point2angle(target_point1)
        elif np.mod(self.counter, 120) < 60:
            self.target_pos = self.point2angle(target_point2)
        elif np.mod(self.counter, 120) < 90:
            self.target_pos = self.point2angle(target_point3)
        else:
            self.target_pos = self.point2angle(target_point4)
        self.send_commands(self.target_pos, self.vel_motor)
            
    # Mallet speed evaluation with random target points
    def random_points(self):
        xd = 80
        yd = 80
        xc = 250
        yc = 0
        x_min, x_max, y_min, y_max = xc - xd, xc + xd, yc - yd, yc + yd
        if np.mod(self.counter, 30) == 15:
            d = 0
            D_MIN = 70
            while d < D_MIN:
                x = (x_max - x_min)*np.random.rand() + x_min
                y = (y_max - y_min)*np.random.rand() + y_min
                d = np.sqrt((x - xc)** 2 + (y - yc)**2)
            self.target_point = [x, y]
        elif np.mod(self.counter, 30) < 15:
            self.target_point = copy.deepcopy(self.initial_point)
        pos  = self.point2angle(self.target_point)
        self.send_commands(pos, [0, 0])

    # Current mallet location is near (point) or not
    def is_reach_point(self, point):
        if self.arm_status is None:
            return False
        x, y = self.angle2point()
        xd = point[0] - x
        yd = point[1] - y
        d = np.sqrt(xd**2 + yd**2)
        if d < D_REACH:
            return True
        else:
            return False
    
    # Present motor angles (present_position) -> mallet location (x, y)
    def angle2point(self):
        p1 = self.arm_status.id1.present_position
        p2 = self.arm_status.id2.present_position
        p1_deg = (360.0) - (p1 * (360.0/4096.0) - 180)
        p2_deg = p2 * (360.0/4096.0) - 180
        p1_rad = p1_deg * (np.pi / 180.0)
        p2_rad = p2_deg * (np.pi / 180.0)
        x = ARM1_LINK1_LENGTH * np.cos(p1_rad) + ARM1_LINK2_LENGTH * np.cos(p2_rad)
        y = ARM1_LINK1_LENGTH * np.sin(p1_rad) + ARM1_LINK2_LENGTH * np.sin(p2_rad)
        return x, y

    # Publish motor commands topic
    def send_commands(self, pos, vel): #publish not send also only send if executable
        commands = MotorCommands()
        commands.goal_position_id1 = int(pos[0])
        commands.goal_position_id2 = int(pos[1])
        commands.goal_speed_id1    = int(vel[0])
        commands.goal_speed_id2    = int(vel[1])
        commands.header.stamp = self.get_clock().now().to_msg()
        # commands.header.frame_id = str(counter) 
        if self.arm_status is None:
            commands.header.frame_id = "-1"
        else:
            commands.header.frame_id = self.object_prediction.header.frame_id 
        self.command_publisher.publish(commands)
        return True

# Motor angle calculation from target mallet location (L(arm length), X, Y) -> (theta1, theta2), (theta1, theta2) 
def inverse_kinematics_2dof(l, x, y):
    if (l[0] - l[1])**2 > (x**2 + y**2):
        return None
    elif (l[0] + l[1])**2 < (x**2 + y**2):
        return None
    temp = (x**2 + y**2 - l[0]**2 - l[1]**2) / (2 * l[0] * l[1])
    theta_id_2     = np.arccos(temp)
    theta_id_2_alt = - theta_id_2
    theta_id_1     = np.arctan2((-l[1] * np.sin(theta_id_2) * x + (l[0] + l[1] * temp) * y), ((l[0] + l[1] * temp) * x + l[1] * np.sin(theta_id_2) * y))
    theta_id_1_alt = np.arctan2((-l[1] * np.sin(theta_id_2_alt) * x + (l[0] + l[1] * temp) * y), ((l[0] + l[1] * temp) * x + l[1] * np.sin(theta_id_2_alt) * y))
    theta_id_2     = theta_id_1 + theta_id_2
    theta_id_2_alt = theta_id_1_alt + theta_id_2_alt
    return ((theta_id_1, theta_id_2), (theta_id_1_alt, theta_id_2_alt))

# Motor angle theta -> encode value (0--4095)
def motor_position(theta):
    motor_pos = [round(theta_2_motor_position(theta[i][j])) for i in range(2) for j in range(2)]
    for i in range(len(motor_pos)):
        if motor_pos[i] < MIN_MOTOR_POSITION:
            motor_pos[i] = MIN_MOTOR_POSITION
        elif motor_pos[i] > MAX_MOTOR_POSITION:
            motor_pos[i] = MAX_MOTOR_POSITION
    return (MOTOR_ENCODER_RESOLUTION - 1 - motor_pos[0], motor_pos[1]) # think whether it is possible to use alt thetas
    # return (MOTOR_ENCODER_RESOLUTION - 1 - motor_pos[2], motor_pos[3])

# Motor angle theta -> encode value (0--4095)
def theta_2_motor_position(theta):
    return (((MOTOR_ENCODER_RESOLUTION) / (2 * np.pi)) * (theta + MOTOR_POSITION_OFFSET))

def main(args=None):
    rclpy.init(args=args)
    arm_motion_planner = ArmMotionPlanner()
    rclpy.spin(arm_motion_planner)
    arm_motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
