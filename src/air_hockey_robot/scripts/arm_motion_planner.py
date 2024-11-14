# #!/usr/bin/env python3

# import random
# import time
# import rclpy
# from rclpy.node import Node
# from air_hockey_robot.msg import ObjectMotionPrediction
# from air_hockey_robot.msg import ArmStatus
# from air_hockey_robot.msg import ArmMotionPlan

# import numpy as np
# import matplotlib.pyplot as plt

# MIN_MOTOR_POSITION = 950
# MAX_MOTOR_POSITION = 3050
# MOTOR_VELOCITY_RPM = 0.229 # rev/min
# MOTOR_VELOCITY_RPMS = MOTOR_VELOCITY_RPM / (60 * 1000) # 60 * 1000 a constant
# MOTOR_ENCODER_RESOLUTION = 4096
# MOTOR_POSITION_OFFSET = np.pi
# LINK_LENGTH = (200, 200)

# class ArmMotionPlanner(Node):

#     def __init__(self):
#         super().__init__('arm_motion_planner')
#         self.object_motion_prediction_subscriber = self.create_subscription(ObjectMotionPrediction, 'object_motion_prediction', self.object_motion_prediction_callback, 10)
#         self.object_motion_prediction_subscriber # preventing unused variable warning
#         self.arm_status_subscriber = self.create_subscription(ArmStatus, 'arm_status', self.arm_status_callback, 5)
#         self.arm_status_subscriber               # preventing unused variable warning
#         self.arm_motion_plan_publisher = self.create_publisher(ArmMotionPlan, 'arm_motion_plan', 5)
#         self.object_motion_prediction = None
#         self.arm_status = None
#         self.future = None
#         self.get_logger().info('init')

#     def object_motion_prediction_callback(self, msg):
#         self.object_motion_prediction = msg
#         self.arm_motion_planner()

#     def arm_status_callback(self, msg):
#         self.arm_status = msg
#         self.arm_motion_planner()

#     def arm_motion_planner(self):
#         if self.object_motion_prediction is None or self.arm_status is None:
#             return False
#         p0 = (self.object_motion_prediction.puck_motion.point.x, self.object_motion_prediction.puck_motion.point.y)
#         p1 = (self.arm_status.current_point.x, self.arm_status.current_point.y)
#         p2 = (475, +114)
#         if len(self.object_motion_prediction.puck_trajectory) == 0:
#             self.get_logger().info('yo')
#             return False

#         else:
#             x = [i.point.x for i in self.object_motion_prediction.puck_trajectory]
#             y = [j.point.y for j in self.object_motion_prediction.puck_trajectory]
#             t = [k.time for k in self.object_motion_prediction.puck_trajectory] # watch out of t structure

#             x = np.array(x)
#             y = np.array(y)
#             t = np.array(t)


#             # x_m = [i.point.x for i in self.object_motion_prediction.mallet_trajectory]
#             # y_m = [j.point.y for j in self.object_motion_prediction.mallet_trajectory]
#             # t_m = [k.time for k in self.object_motion_prediction.mallet_trajectory] # watch out of t structure
#             # min_length = min(len(x), len(y), len(t), len(x_m), len(y_m), len(t_m))
#             # x = x[:min_length]
#             # y = y[:min_length]
#             # t = t[:min_length]
#             # x_m = x_m[:min_length]
#             # y_m = y_m[:min_length]
#             # t_m = t_m[:min_length]
#             # x = np.array(x)
#             # y = np.array(y)
#             # t = np.array(t)
#             # x_m = np.array(x_m)
#             # y_m = np.array(y_m)
#             # t_m = np.array(t_m)
#             # r = np.sqrt(np.abs(x - x_m)**2 + np.abs(y - y_m)**2)
#             # i = np.argmin(np.abs(r))


# #


#             # if r[i] < (20 + 25 + 30):
#             #     print('mallet')
#             #     # e = 1.5
#             #     x = x_m
#             #     y = y_m
#             #     t = t_m
#             #     # t = t_m * 0.7
#             #     # print(t)
#             #     # plt.figure(figsize=(10, 6))
#             #     # plt.plot(x, y, '-o', label='Trajectory')
#             #     # plt.xlabel('X Position')
#             #     # plt.ylabel('Y Position')
#             #     # plt.title('Object Trajectory in 2D space')
#             #     # plt.legend()
#             #     # plt.grid(True)
#             #     # for i in range(1, len(x)):
#             #     #     dx = x[i] - x[i-1]
#             #     #     dy = y[i] - y[i-1]
#             #     #     plt.annotate(f'dx={dx:.2f}, dy={dy:.2f}', ((x[i] + x[i-1])/2, (y[i] + y[i-1])/2))
#             #     # plt.show()
#             #     # # regenerate prediction
#             #     # x = x[:i+1]
#             #     # y = y[:y+1]
#             #     # t = t[:t+1]
#             #     # dx = x[i+1] - x[i]
#             #     # dy = y[i+1] - y[i]
#             # else:
#             #     print('puck')







#             p4 = default()
#             self.send_arm_motion_plan(p1, p4, p3=None, t=None)
#             time.sleep(15)

#             random.seed(3569310)
#             for i in range(300):
#                 p4 = (random.randint(220, 370), random.randint(-90, 90))
#                 self.send_arm_motion_plan(p1, p4, p3=None, t=None)
#                 time.sleep(1.5)
#                 print('test: ', i)
            
#             print('TEST DONE!!!!!')
#             while True:
#                 #blocking 
#                 print('COMPLETE!')

# #UNCOMENT HERE FOR REGULAR GAMEPLAY
#             # if self.object_motion_prediction.puck_motion.speed < 10.0:
#             #     if self.object_motion_prediction.puck_motion.in_own_half:
#             #         # p4 = sweep(p0, p1)
#             #         # if self.send_arm_motion_plan(p1, p4, p3=None, t=None):
#             #         #     self.get_logger().info('sweep')
#             #         #     return True
#             #         # else:
#             #         #     p4 = default()
#             #         #     self.send_arm_motion_plan(p1, p4, p3=None, t=None)
#             #         #     self.get_logger().info('default')
#             #         #     return True
#             #         p4 = default()
#             #         self.send_arm_motion_plan(p1, p4, p3=None, t=None)
#             #         self.get_logger().info('default')
#             #         return True
#             #     else:
#             #         p4 = default()
#             #         self.send_arm_motion_plan(p1, p4, p3=None, t=None)
#             #         self.get_logger().info('default')
#             #         return True

#             # print(t)
#             # plt.figure(figsize=(10, 6))
#             # plt.plot(x, y, '-o', label='Trajectory')
#             # plt.xlabel('X Position')
#             # plt.ylabel('Y Position')
#             # plt.title('Object Trajectory in 2D space')
#             # plt.legend()
#             # plt.grid(True)
#             # for i in range(1, len(x)):
#             #     dx = x[i] - x[i-1]
#             #     dy = y[i] - y[i-1]
#             #     plt.annotate(f'dx={dx:.2f}, dy={dy:.2f}', ((x[i] + x[i-1])/2, (y[i] + y[i-1])/2))
#             # plt.show()

#             p3, p4, t4 = snipe_attack(x, y, t, p1, p2)
#             if self.send_arm_motion_plan(p1, p4, p3, t4):
#                 self.get_logger().info('snipe')
#                 print('p3: ', p3)
#                 print('p4: ', p4)
#                 print('t4: ', t4)
#                 return True

#             # result = regular_attack(x, y, t)
#             # if result:
#             #     p4, t4 = result
#             #     if self.send_arm_motion_plan(p1, p4, t=t4):
#             #         self.get_logger().info('attack')
#             #         return True

#             # p3, p4, t4 = control(x, y, t, p1)
#             # if self.send_arm_motion_plan(p1, p4, p3, t4):
#             #     self.get_logger().info('control')
#             #     return True

#             # p4, t4 = block(x, y, t)
#             # if self.send_arm_motion_plan(p1, p4, t=t4):
#             #     self.get_logger().info('block')
#             #     return True

#             p4 = default()
#             if self.send_arm_motion_plan(p1, p4, p3=None, t=None):
#                 return True


#     def send_arm_motion_plan(self, p1, p4, p3=None, t=None): #publish not send also only send if executable
#         arm_motion_plan = ArmMotionPlan()

#         if p3:
#             pos, vel, t3 = calculate_motion_plan(p1, p3, False)
#             if pos is None:
#                 self.get_logger().info('1')
#                 return False

#             arm_motion_plan.way_position_id1 = int(pos[0])
#             arm_motion_plan.way_position_id2 = int(pos[1])
#             arm_motion_plan.speed_way_id1 = int(vel[0])
#             arm_motion_plan.speed_way_id2 = int(vel[1])

#         else:
#             arm_motion_plan.way_position_id1 = 0
#             arm_motion_plan.way_position_id2 = 0
#             arm_motion_plan.speed_way_id1 = 0
#             arm_motion_plan.speed_way_id2 = 0

#         pos, vel, t4 = calculate_motion_plan(p3 if p3 else p1, p4, True)
#         if pos is None:
#             self.get_logger().info('2')
#             return False
#         arm_motion_plan.goal_position_id1 = int(pos[0])
#         arm_motion_plan.goal_position_id2 = int(pos[1])
#         arm_motion_plan.speed_goal_id1 = int(vel[0])
#         arm_motion_plan.speed_goal_id2 = int(vel[1])
#         if pos[0] == 2047 and pos[1] == 3050: # CAN NOT FIGURE OUT THE BUG tha'ts why it's here
#             return False
#         if t is None:
#             arm_motion_plan.time.sec = 0
#             arm_motion_plan.time.nanosec = 0
#         else:
#             travel_time = int(t3 + t4 if p3 else t4)
#             if (travel_time) > (t.nanosec + (t.sec * 10**9)):
#                 print(travel_time)
#                 print(t.nanosec + (t.sec * 10**9))
#                 self.get_logger().info('3')
#                 return False
#             else:
#                 arm_motion_plan.time.sec = 0 # check here
#                 arm_motion_plan.time.nanosec = travel_time

#         arm_motion_plan.header.stamp = self.get_clock().now().to_msg()
#         self.arm_motion_plan_publisher.publish(arm_motion_plan)
#         return True







#         # arm_motion_plan = ArmMotionPlan()

#         # pos, vel, t3, t4 = None, None, None, None
#         # if p3 is not None:
#         #     pos, vel, t3 = calculate_motion_plan(p1, p3, False)
#         #     if pos is None:
#         #         return
#         #     arm_motion_plan.way_position_id1 = int(pos[0])
#         #     arm_motion_plan.way_position_id2 = int(pos[1])
#         #     arm_motion_plan.speed_way_id1 = int(vel[0])
#         #     arm_motion_plan.speed_way_id2 = int(vel[1])
#         # else:
#         #     arm_motion_plan.way_position_id1 = 0
#         #     arm_motion_plan.way_position_id2 = 0
#         #     arm_motion_plan.speed_way_id1 = 0
#         #     arm_motion_plan.speed_way_id2 = 0

#         # if p3 is not None:
#         #     pos, vel, t4 = calculate_motion_plan(p3, p4, True) # this assumes that p3 exsists
#         # if p3 is None:
#         #     pos, vel, t4 = calculate_motion_plan(p3, p4, True) # this assumes that p3 exsists
#         # if pos is None:
#         #     return
#         # arm_motion_plan.goal_position_id1 = int(pos[0])
#         # arm_motion_plan.goal_position_id2 = int(pos[1])
#         # arm_motion_plan.speed_goal_id1 = int(vel[0])
#         # arm_motion_plan.speed_goal_id2 = int(vel[1])

#         # if t4 is None:
#         #     arm_motion_plan.time.sec = 0
#         #     arm_motion_plan.time.nanosec = 0
#         #     # self.get_logger().info('t4 none')
#         # else:
#         #     if (t3 + t4) > (t.nanosec + (t.sec * 10**9)):
#         #         self.get_logger().info('here')
#         #         return False
#         #     else:
#         #         arm_motion_plan.time.sec = 0 # check here
#         #         arm_motion_plan.time.nanosec = 1000000000

#         # self.arm_motion_plan_publisher.publish(arm_motion_plan)
#         # return True

#         # if p3 is None:
#         #     arm_motion_plan.p3.x = 0.0
#         #     arm_motion_plan.p3.y = 0.0
#         #     arm_motion_plan.p3.z = 0.0
#         # else:
#         #     arm_motion_plan.p3.x = p3[0]
#         #     arm_motion_plan.p3.y = p3[1]
#         #     arm_motion_plan.p3.z = 0.0
#         # arm_motion_plan.p4.x = p4[0]
#         # arm_motion_plan.p4.y = p4[1]
#         # arm_motion_plan.p4.z = 0.0
#         # if t is None:
#         #     arm_motion_plan.time = time(seconds=0, nanoseconds=0)
#         # else:
#         #     arm_motion_plan.time = t
#         # self.future = self.arm_motion_plan_client.call_async(arm_motion_plan)
#         # rclpy.spin_until_future_complete(self, self.future)
#         # self.get_logger().info(f'result: {self.future.result()}')
#         # return self.future.result()
#         # return True

# def snipe_attack(x, y, t, p1, p2):
#     d = 7
#     dx = x - p2[0]
#     dy = y - p2[1]
#     dx2 = d * dy / np.sqrt(dx**2 + dy**2)
#     dx2 = d * dx / np.sqrt(dx**2 + dy**2) 
#     dy2 = d * dy / np.sqrt(dx**2 + dy**2)
#     xd = x + dx2 
#     yd = y + dy2
#     r = (xd - p1[0])**2 + (yd - p1[1])**2
#     i = np.argmin(r)
#     p3 = (xd[i], yd[i])
#     p4 = (x[i], y[i])
#     return p3, p4, t[i] # waypoint x, waypoint y, x, y, and then time t 

# def block(x, y, t):
#     defense_line = 220.0 # make this a constant
#     i = np.argmin(np.abs(x - defense_line))
#     p4 = (x[i], y[i])
#     return p4, t[i]

# def regular_attack(x ,y, t):
#     attack_line = 320.0 # make this a constant
#     i = np.argmin(np.abs(x - attack_line))
#     # i = np.argmin(np.abs(x - attack_line)[x >= attack_line])
#     p4 = (x[i], y[i])
#     if x[i] > attack_line:
#         return p4, t[i]
#     else:
#         return False

# def control(x, y, t, p1):
#     r = (x - p1[0])**2 + (y - p1[1])**2
#     i = np.argmin(r)
#     dy_dx = np.diff(y) / np.diff(x)
#     d2y_d2x = np.diff(dy_dx) / np.diff(x[:-1])
#     j = np.where(np.abs(d2y_d2x) > np.percentile(np.abs(d2y_d2x), 95))[0]
#     p3 = (x[i], y[i])
#     p4 = p3
#     for k in j:
#         if t[k + 2].nanosec> t[i].nanosec: # no sec??
#             p4 = (x[k + 2], y[k + 2])
#             break
#     return p3, p4, t[i]

# def sweep(p0, p1):
#     if p0[0] - p1[0] > 1e-6:
#         m = (p0[1] - p1[1]) / (p0[0] - p1[0])
#         x = p0[0] + 3.5
#         y = m * x + p0[1]
#         p4 = (x, y)
#         return p4
#     else:
#         return p0

# def default():
#     p4 = (245.0, 0.0)
#     return p4

# def inverse_kinematics_2dof(l, x, y):
#     if (l[0] - l[1])**2 > (x**2 + y**2):
#         return None
#     elif (l[0] + l[1])**2 < (x**2 + y**2):
#         return None
#     temp = (x**2 + y**2 - l[0]**2 - l[1]**2) / (2 * l[0] * l[1])
#     theta_id_2     = np.arccos(temp)
#     theta_id_2_alt = - theta_id_2
#     theta_id_1     = np.arctan2((-l[1] * np.sin(theta_id_2) * x + (l[0] + l[1] * temp) * y), ((l[0] + l[1] * temp) * x + l[1] * np.sin(theta_id_2) * y))
#     theta_id_1_alt = np.arctan2((-l[1] * np.sin(theta_id_2_alt) * x + (l[0] + l[1] * temp) * y), ((l[0] + l[1] * temp) * x + l[1] * np.sin(theta_id_2_alt) * y))
#     theta_id_2     = theta_id_1 + theta_id_2
#     theta_id_2_alt = theta_id_1_alt + theta_id_2_alt
#     return ((theta_id_1, theta_id_2), (theta_id_1_alt, theta_id_2_alt))

# def motor_position(theta):
#     motor_pos = [round(theta_2_motor_position(theta[i][j])) for i in range(2) for j in range(2)]
#     for i in range(len(motor_pos)):
#         if motor_pos[i] < MIN_MOTOR_POSITION:
#             motor_pos[i] = MIN_MOTOR_POSITION
#         elif motor_pos[i] > MAX_MOTOR_POSITION:
#             motor_pos[i] = MAX_MOTOR_POSITION
#     return (MOTOR_ENCODER_RESOLUTION - 1 - motor_pos[0], motor_pos[1]) # think whether it is possible to use alt thetas
#     # return (MOTOR_ENCODER_RESOLUTION - 1 - motor_pos[2], motor_pos[3])

# def theta_2_motor_position(theta):
#     return (((MOTOR_ENCODER_RESOLUTION) / (2 * np.pi)) * (theta + MOTOR_POSITION_OFFSET))

# def calculate_motion_plan(p0, p1, linear_motion): # no error handling, inv kin might return false
#     theta = inverse_kinematics_2dof(LINK_LENGTH, p1[0], p1[1])
#     if theta is None:
#         return None, None, None
#     goal_motor_pos = np.array(motor_position(theta))
#     theta = inverse_kinematics_2dof(LINK_LENGTH, p0[0], p0[1])
#     if theta is None:
#         return None, None, None
#     current_motor_pos = np.array(motor_position(theta))
#     delta_motor_pos = np.absolute(goal_motor_pos - current_motor_pos)
#     if np.all(delta_motor_pos == 0):
#         return current_motor_pos, (0, 0), 0
#     rev = delta_motor_pos / (MOTOR_ENCODER_RESOLUTION - 1)
#     travel_time = rev / MOTOR_VELOCITY_RPMS
#     if linear_motion:
#         if travel_time[0] > travel_time[1]:
#             rpms = rev[1] / travel_time[0]
#             motor_velocity = (0, rpms * (60 * 1000) * MOTOR_VELOCITY_RPM) # 0 means max speed
#         else:
#             rpms = rev[0] / travel_time[1]
#             motor_velocity = (rpms * (60 * 1000) * MOTOR_VELOCITY_RPM, 0)
#     else:
#         motor_velocity = (0, 0)
#     return goal_motor_pos, motor_velocity, np.max(travel_time)






#             # travel time is known here so compare the time here



#     # rev = []
#     # travel_time= []
#     # for i in range(len(delta_motor_pos)):
#     #     rev.append(delta_motor_pos(i) / (MOTOR_ENCODER_RESOLUTION - 1))
#     #     travel_time.append(rev(i) / MOTOR_VELOCITY_RPMS)
    




# # define p0 to be current puck location

# # def snipe_attack(object_motion_prediction, arm_status):
# #     travel_distance = float('inf')
# #     p3 = None
# #     p4 = None
# #     while object_motion_prediction.trajectory:
# #         f = calculate_slope_intercept(object_motion_prediction.trajectory.start, object_motion_prediction.trajectory.end)
# #         p1 = [arm_status.x, arm_status.y]
# #         p2 = [600, 0.0] # GOAL
# #         d = 3.0 # constant
# #         l, theta_ref = calculate_l(p1, p2, f)
# #         p3, p4 = calculate_optimized_waypoint(p1, p2, l, d, theta_ref)
# #         a = calculate_distance(p1, p3)
# #         if a < travel_distance:
# #             travel_distance = a
# #         object_motion_prediction.trajectory.pop()
# #     return p3, p4 # calctime in this function to determine if it is possible to do attack time wise

# # def calculate_slope_intercept(p1, p2):
# #     m = (p2[1] - p1[1]) / (p2[0] - p1[0])
# #     b = p1[1] - m * p1[0]
# #     return m, b

# # def calculate_distance(p1, p2):
# #     return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# # def format_equation(m, b):
# #     return f"y = {m}x + {b}" if b >= 0 else f"y = {m}x - {-b}"

# # def calculate_intersection(m1, b1, m2, b2):
# #     x = symbols('x')
# #     intersection = solve(Eq(m1*x + b1, m2*x + b2), x)
# #     if not intersection:
# #         return None, None
# #     x_intersection_val = float(intersection[0].evalf())
# #     y_intersection_val = m2 * x_intersection_val + b2
# #     return x_intersection_val, y_intersection_val

# # def calculate_l(p1, p2, f):
# #     m_g, b_g = calculate_slope_intercept(p1, p2)
# #     m_f = f[0]
# #     m_h = -1/m_f
# #     b_h = p2[1] - m_h * p2[0]
# #     x_intersection_fg, y_intersection_fg = calculate_intersection(m_g, b_g, m_f, f[1])
# #     x_intersection_hf, y_intersection_hf = calculate_intersection(m_h, b_h, m_f, f[1])
# #     if x_intersection_fg is None or x_intersection_hf is None:
# #         print('No intersection found.')
# #         return
# #     distance_p2_intersection_hf = calculate_distance(p2, (x_intersection_hf, y_intersection_hf))
# #     print("Distance between p2 and intersection of h(x) and f(x):", distance_p2_intersection_hf)

# #     return distance_p2_intersection_hf, np.arctan(m_h) if m_h >= 0 else np.pi + np.arctan(m_h)

# # def calculate_optimized_waypoint(p1, p2, l, d, theta_ref):
# #     # x_ref = p1[0] - p2[0]
# #     # y_ref = p1[1] - p2[1]
# #     # x_prime = x_ref * np.cos(theta_ref) - y_ref * np.sin(theta_ref)
# #     # y_prime = x_ref * np.sin(theta_ref) + y_ref * np.cos(theta_ref)
# #     # theta = symbols('theta')
# #     l_ = l / cos(theta)
# #     x_via = p2[0] + (l_ + d) * cos(theta_ref + theta)    
# #     y_via = p2[1] + (l_ + d) * sin(theta_ref + theta)    
# #     k = (x_via - p1[0]) ** 2 + (y_via - p1[1])**2
# #     # k = sqrt((x_prime - l - d * cos(theta))**2 + (y_prime - l * tan(theta) - d * sin(theta)) ** 2)
# #     k_first_derivative = diff(k, theta)
# #     k_second_derivative = diff(k_first_derivative, theta)
# #     k_third_derivative = diff(k_second_derivative, theta)
# #     func = lambdify(theta, k, 'numpy')
# #     func1 = lambdify(theta, k_first_derivative, 'numpy')
# #     func2 = lambdify(theta, k_second_derivative, 'numpy')
# #     func3 = lambdify(theta, k_third_derivative, 'numpy')
# #     initial_guess = np.radians(-60.0)
# #     root = newton(func1, initial_guess, fprime=func2, tol=0.001, maxiter=1000, fprime2=func3)
# #     # print("The root is approximately at theta =", np.degrees(root), "degrees.")
# #     theta = root + theta_ref
# #     # print("The theta_ref is approximately at theta_ref =", np.degrees(theta_ref), "degrees.")
# #     # print("The theta is approximately at theta =", np.degrees(theta), "degrees.")
# #     collision_point = []
# #     waypoint = []
# #     l = l / np.cos(root)
# #     collision_point.append(l * np.cos(theta) + p2[0])
# #     collision_point.append(l * np.sin(theta) + p2[1])
# #     waypoint.append(d * np.cos(theta) + collision_point[0])
# #     waypoint.append(d * np.sin(theta) + collision_point[1])
# #     return waypoint, collision_point

# # def main(p1, p2, f, d):
# #     l, theta_ref = calculate_l(p1, p2, f)
# #     p3, p4 = calculate_optimized_waypoint(p1, p2, l, d, theta_ref)
# #     print(p3)
# #     print(p4)

# # # p1 = [225, -50]
# # # p2 = [611, 0]
# # # f = (1, -380)
# # # d = 3.0

# # p1 = [-3, 5]  # xy coordinates for p1 CONSIDER USING TUPLES INSTEAD OF LISTS
# # p2 = [5, 1]  # xy coordinates for p2
# # f = (1, 0)  # values for m and b in f(x)
# # d = 3.0

# # main(p1, p2, f, d)



# # // WHEN THE PUCK MOVES, THE PUCK ALWAYS MOVES FROM POSITIVE X COORDINATE TO NEGATIVE X COORDINATE



# # // consider speed decreasing coefficient and reflection coefficient for now

# #         // if(this->mallet_motion_data->linear_motion &&this->puck_motion_data->in_defensive_half && this->puck_motion_data->speed < 10.0 && this->mallet_motion_data->speed > 550.0) // mallet in linear motion and puck slow
# #         // {
# #         //     // Use the fast moving mallet to predict the Trajectory
# #         // }
# #         // else if(this->puck_motion_data->speed > )
# #         // {
# #         //     // 
# #         // }

# # //         if(this->puck_motion_data->speed < 10.0)
# # //         {
# # //             if(this->puck_motion_data->in_opponents_side)
# # //             {
                
# # //             }
# # //         }


# # //         // // check for change in drection here
# # //         // this->update_motion_data_queue();
# # //         // if 

# # // void ObjectMotionPredictor::update_motion_data_queue()
# # // {
# # //     MotionData motion_data = {this->puck_motion_data, this->mallet_motion_data};
# # //     this->motion_data_queue.push_back(motion_data);
# # //     while(this->motion_data_queue.size() > 4)
# # //     {
# # //         this->motion_data_queue.pop_front();
# # //     }
# # // }

# # // // below is for motion analyzer
# # // bool ObjectMotionPredictor::mallet_straight_motion()
# # // {
# # //     std::size_t DATA_POINTS = 4;
# # //     double TOLERANCE = 0.1; // maybe use struct in global hpp for constants
# # //     if(this->motion_data_queue.size() < DATA_POINTS)
# # //     {
# # //         return false;
# # //     }
# # //     double distance = 0.0;
# # //     for(std::size_t i = 0; i < DATA_POINTS - 1; i++)
# # //     {
# # //         distance = calculate_distance(
# # //         this->motion_data_queue.[i].mallet_motion_data.xmm;
# # //         this->motion_data_queue.[i].mallet_motion_data.ymm;
# # //         this->motion_data_queue.[i + 1].mallet_motion_data.xmm;
# # //         this->motion_data_queue.[i + 1].mallet_motion_data.ymm;
# # //         );
# # //     }
# # //     double displacement = calculate_distance(
# # //         this->motion_data_queue.[0].mallet_motion_data.xmm;
# # //         this->motion_data_queue.[0].mallet_motion_data.ymm;
# # //         this->motion_data_queue.[DATA_POINTS - 1].mallet_motion_data.xmm;
# # //         this->motion_data_queue.[DATA_POINTS - 1].mallet_motion_data.ymm;     
# # //     )
# # //     return (distance - displacement) / displacement <= TOLERANCE;
# # // }

# # // double ObjectMotionPredictor::calculate_distance(double x1, double y1, double x0, double y0)
# # // {
# # //     return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
# # // }


# # // // if direction dramatically changes, predict the collision Point. Use collision Point and the new Point to predict the new path