import functools
import json
import math
import os
import sys
import threading
import time
import carla
import pygame
from config import client, world, env_map, blueprint_library, timed_function, destroy_car, draw_arrow, car_info, \
    Node_Connect, main_car_info
from car_attribute_method import (load_map, Vehicle_Traffic, get_speed, set_speed, get_car_steer)
from time import sleep
import execjs
import socketio
import numpy as np
import keyboard
import multiprocessing
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.controller import VehiclePIDController
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
import pandas as pd

departure = 0
stage = None  # 阶段
status = None  # 主车状态
stop_event_car_main = threading.Event()  # 用于控制主车的线程
stop_event_car_vice = threading.Event()  # 用于控制副车的线程
save_csv_status = threading.Event()  # 用于保存数据的线程
start_time = time.time()  # 用于记录每个阶段的时间


# 用于接收node的返回值
# def get_error_data(data):
#     global departure
#     departure = data


# 用于每个阶段的打印
def start_end_statement(func):
    def wrapper(*args, **kwargs):
        global stage
        print(f"{stage}路段开始~")
        result = func(*args, **kwargs)
        print(f"{stage}此路段结束~")
        return result

    return wrapper


@timed_function(30, save_csv_status)
def save_csv():
    try:
        # 找到最短列的长度
        min_length = min(len(col) for col in main_car_info.values())
        # 截取每列至最短列的长度
        trimmed_data = {key: value[:min_length] for key, value in main_car_info.items()}
        pd.DataFrame(trimmed_data).to_csv("data_400.csv", index=None)
    except Exception as e:
        print(e)


sio = socketio.Client()  # 创建socketio
sio.connect('http://localhost:3000')  # 与node建立连接
# sio.on("node_response", lambda data: globals().update(departure=data))  # 拿到node返回的偏离值

# 上
s6_1_location = carla.Location(x=-5067.266113, y=-2356.738525, z=10.1)
start_location = carla.Location(x=-3067.011230, y=-2378.194092, z=10.1)
c1_1_location = carla.Location(x=-666.382019, y=-2403.942383, z=10.1)
c2_1_location = carla.Location(x=1334.611572, y=-2425.406250, z=10.1)
s1_1_location = carla.Location(x=3334.577148, y=-2446.857178, z=10.1)
c1_2_location = carla.Location(x=5335.143555, y=-2468.314453, z=10.1)
c1_2_end_location = carla.Location(x=7335.095703, y=-2489.765625, z=10.1)
# 右
s2_1_location = carla.Location(x=7909.936523, y=-2058.584717, z=10.1)
c2_2_location = carla.Location(x=7946.371582, y=-58.130398, z=10.1)
c2_2_end_location = carla.Location(x=7983.767578, y=1922.940063, z=10.1)
# 下
c1_3_location = carla.Location(x=7566.672363, y=2346.073730, z=10.1)
s3_1_location = carla.Location(x=5566.446777, y=2367.419922, z=10.1)
c2_3_location = carla.Location(x=3565.909180, y=2388.772217, z=10.1)
s4_1_location = carla.Location(x=1565.524902, y=2410.120850, z=10.1)
c2_4_location = carla.Location(x=-435.543610, y=2433.525146, z=10.1)
s5_1_location = carla.Location(x=-2435.976562, y=2457.665283, z=10.1)
s5_1_end_location = carla.Location(x=-4436.812012, y=2481.809814, z=10.1)
# 左
c1_4_location = carla.Location(x=-5408.582520, y=2082.249023, z=10.1)
c2_5_location = carla.Location(x=-5443.394531, y=82.447830, z=10.1)
c2_5_end_location = carla.Location(x=-5478.225098, y=-1918.433105, z=10.1)


# 窗口显示,一个传感器在车子上
class Window:
    def __init__(self, car):
        self.car = car
        # 屏幕大小
        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1980, 1280
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1280, 900
        # 初始化窗口
        self.screen = None
        # 初始化pygame
        pygame.init()
        # 判断有没有方向盘
        if pygame.joystick.get_count() == 0:
            print("没有可用方向盘或者踏板")
            pygame.quit()
            exit(0)

        # 初始化第一个游戏手柄
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.clock = pygame.time.Clock()
        self.size = 18  # 字体大小
        self.fps = 60  # 帧率
        self.font = pygame.font.Font("TTF/宋体.ttf", self.size)  # 初始化字体对象

        # 初始化传感器
        self.blueprint = blueprint_library.find('sensor.camera.rgb')
        self.blueprint.set_attribute('image_size_x', f'{self.SCREEN_WIDTH}')  # 传感器获得的图片高度
        self.blueprint.set_attribute('image_size_y', f'{self.SCREEN_HEIGHT}')  # 传感器获得的图片宽度
        self.blueprint.set_attribute('fov', '110')
        spawn_point = carla.Transform(carla.Location(x=-6, z=4))  # 传感器相对车子的位置设置
        self.sensor = world.spawn_actor(self.blueprint, spawn_point, attach_to=self.car)  # 添加传感器

    def show_screen(self):  # 显示窗口
        # 初始化窗口
        pygame.display.set_caption("模拟场景")
        # 去掉pygame.RESIZABLE表示窗口不可变
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), pygame.HWSURFACE |
                                              pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.sensor.destroy()
                    pygame.quit()
                    pygame.joystick.quit()
                    sio.emit("dis_connect")
                    sleep(1)
                    os._exit(0)  # 强制退出所有程序，结束程序
            self.clock.tick(self.fps)

    # 加载图片
    def process_img(self, image):
        i = np.array(image.raw_data)
        i2 = i.reshape((self.SCREEN_HEIGHT, self.SCREEN_WIDTH, 4))
        i3 = i2[:, :, :3]
        i3 = np.rot90(i3, k=1, axes=(0, 1))
        i3 = i3[..., ::-1]
        img_surface = pygame.surfarray.make_surface(np.flip(i3, axis=0))
        self.screen.blit(img_surface, (0, 0))  # 绘制图片

        # 添加文字信息
        self.draw_word(f"速度：{round(get_speed(self.car), 2)}km/h", 20, (0, 0))
        self.draw_word(f"方向盘：{round(car_info['steering'] * 450, 2)}°", 20, (0, 20))
        self.draw_word(f"油门：{round(car_info['throttle'], 2)}", 20, (0, 40))
        self.draw_word(f"刹车：{round(car_info['brake'], 2)}", 20, (0, 60))
        self.draw_word(f"阶段：{stage}", 20, (0, 80))
        self.draw_word(f"状态：{status}", 20, (0, 100))
        # self.draw_word(f"该路段行驶时间：{round(time.time() - start_time, 2)}s", 20, (0, 120))

        pygame.display.flip()  # 刷新屏幕

    def draw_word(self, word, length, position, color=(255, 0, 0)):
        text = self.font.render(word.ljust(length), True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.screen.blit(text, text_rect)


# 主车类
class Main_Car:
    def __init__(self, car):
        self.car = car
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.change_control = True  # 人工/自动驾驶切换,True为人工，False为自动
        self.button = False  # 用于自动驾驶转人工按钮切换，当用户按下按钮才认为需要接管
        self.people_speed_limit = 150  # 人工速度限制
        self.auto_speed_limit = 100  # 自动驾驶速度限制
        # 初始化PID控制
        args_lateral_dict = {'K_P': 1.95, 'K_D': 0.2, 'K_I': 0.07, 'dt': 1.0 / 10.0}
        args_long_dict = {'K_P': 1, 'K_D': 0, 'K_I': 0.75, 'dt': 1.0 / 10.0}
        self.PID = VehiclePIDController(self.car, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)

    @timed_function(0.001, stop_event_car_main)
    def control_car(self):
        global status, main_car_info
        main_car_info["x"] += [self.car.get_location().x]
        main_car_info["y"] += [self.car.get_location().y]
        main_car_info["status"] += [status]
        main_car_info["speed"] += [get_speed(self.car)]
        main_car_info["steering"] += [car_info["steering"]]
        main_car_info["throttle"] += [car_info["throttle"]]
        main_car_info["brake"] += [car_info["brake"]]
        if self.change_control:  # 人工控制
            status = "0"
            self.button = False
            # 获取方向盘信息
            car_info["steering"], car_info["throttle"], car_info["brake"] = self.get_steering_wheel_info()
            # 如果值过小就让他为0，不然车子动不起来
            if car_info["throttle"] < 0.01:
                car_info["throttle"] = 0
            if car_info["brake"] < 0.01:
                car_info["brake"] = 0

            # 设置车子的最大速度限制，如果车子速度大于150，就控制在150
            if get_speed(self.car) > self.people_speed_limit:
                throttle = 0.5
            else:
                throttle = car_info["throttle"]
            control = carla.VehicleControl(steer=car_info["steering"], throttle=throttle, brake=car_info["brake"])
            # 应用控制
            self.car.apply_control(control)

            # 按下e键切换为自动驾驶
            if keyboard.is_pressed('e'):
                self.change_control = False
                print("切换为自动驾驶~")

        else:  # 自动驾驶控制
            if get_speed(self.car) < 99:  # 修改加速度
                set_speed(self.car, get_speed(self.car) + 0.16)

            status = "1"
            waypoint = env_map.get_waypoint(self.car.get_location()).next(10)
            if waypoint:
                waypoint = waypoint[0]
            else:
                print("前方没有路了")
                return
            control = self.PID.run_step(self.auto_speed_limit, waypoint)
            self.car.apply_control(control)
            car_info["steering"] = get_car_steer(self.car)

            # 按下w表示即将人工接管操作
            if keyboard.is_pressed('q'):
                # print("w被按下~")
                self.button = True
            # 得到方向盘偏移中间量,如果大于就改为人工驾驶  450-1
            if self.button and abs(self.get_steering_wheel_info()[0]) > 2 / 450:
                # 切换人工驾驶
                self.change_control = True
                sio.emit("dis_connect")
                print("切换为人工驾驶~")
            try:
                sio.emit("steering_data", {"message": 50 * get_car_steer(self.car) + 50})
            except Exception as e:
                print("服务器断开！！！！！！！！！！！！！！")
                return
            # 判断车子前方是否有交通标志
            for prop in list(world.get_actors().filter("*.prop.*")):
                # 交通标志所在道路
                prop_location = prop.get_location()
                prop_road_id = env_map.get_waypoint(prop_location).lane_id
                # 车子所在道路
                main_vehicle_location = self.car.get_location()
                main_vehicle_road_id = env_map.get_waypoint(main_vehicle_location).lane_id
                if prop_road_id == main_vehicle_road_id:  # 如果在同一条路
                    if prop_location.distance(main_vehicle_location) < self.auto_speed_limit / 3.6 * 8:  # 如果到达交通标志剩余8s
                        if not self.change_control:  # 发出警告后仍然是自动驾驶
                            print("警告！！！！！！！！！！！！！！")
                            if prop_location.distance(
                                    main_vehicle_location) < self.auto_speed_limit / 3.6 * 8 - 50:  # 如果车子继续前进，离标志很近还没有接管，停车
                                set_speed(self.car, 0)

    def get_steering_wheel_info(self):
        """
        return: 方向盘、油门、刹车
        """
        print(self.joystick.get_axis(0), (-self.joystick.get_axis(2) + 1) / 2, (-self.joystick.get_axis(3) + 1) / 2)
        return self.joystick.get_axis(0), (-self.joystick.get_axis(2) + 1) / 2, (-self.joystick.get_axis(3) + 1) / 2


# 副车类
class Vice_Car:
    def __init__(self, cars=None, speeds=113):
        self.cars = cars
        self.speeds = speeds
        # 初始化PID参数
        self.args_lateral_dict = {'K_P': 1.95, 'K_D': 0.2, 'K_I': 0.07, 'dt': 1.0 / 10.0}
        self.args_long_dict = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 10.0}

    @timed_function(0.001, stop_event_car_vice)
    def vice_car_follow_lane(self):
        if self.cars is None or len(self.cars) == 0:
            return
        for index, car in enumerate(self.cars):
            pid = VehiclePIDController(car, args_lateral=self.args_lateral_dict, args_longitudinal=self.args_long_dict)
            waypoint = env_map.get_waypoint(car.get_location()).next(10)
            if waypoint:
                waypoint = waypoint[0]
            else:  # 如果前方没有路，销毁车子
                car.destroy()
                del self.cars[index]  # 销毁车子
                continue

            car_exist = True  # 用于标志该车辆还存在
            car_location = car.get_location()  # 获取车子坐标
            car_road_id = env_map.get_waypoint(car_location).lane_id  # 获取车子所在道路
            for prop in list(world.get_actors().filter("*.prop.*")):  # 遍历所有交通标志
                prop_location = prop.get_location()
                prop_road_id = env_map.get_waypoint(prop_location).lane_id
                if prop_location.distance(car_location) < 10:
                    if prop_road_id == car_road_id:
                        # print("销毁成功！")
                        car.destroy()  # 销毁车子
                        del self.cars[index]  # 删除列表中的车子
                        car_exist = False  # 置为车子不存在
                        break
            # 如果没有车子进入下一次循环
            if not car_exist:
                continue
            try:  # 此进程还在运行，加入try为了防止副车被销毁后仍然去操控车子
                set_speed(car, self.speeds)  # 强制设置速度
                result = pid.run_step(self.speeds + 1, waypoint)  # 设置速度限制+1，给个阈值
                car.apply_control(result)
            except Exception as e:
                print(f"车子被销毁~{e}")


# 用于检测车子跟终点的距离,如果距离终点大于20就一直死循环在这里
def check_distance(car, end_location):
    while car.get_location().distance(end_location) > 20:
        sleep(1)


# 修改高度
def change_height(locations):
    for i in range(len(locations)):
        locations[i].z += 0.1
    return locations


# 熟悉过程
@start_end_statement
def know_well_course(main_vehicle, vehicle_traffic, vice_car, end_location):
    global start_time
    """
    :param main_vehicle: 主车
    :param vehicle_traffic: 创建车子的类
    :param traffics_info: 交通标志的信息
    """
    # 圆桶坐标
    know_prop_traffic_location1 = env_map.get_waypoint(start_location).next(1021)[0].transform.location  # 圆桶1
    know_prop_traffic_location2 = env_map.get_waypoint(start_location).next(1023)[0].transform.location  # 圆桶2
    know_prop_traffic_location3 = env_map.get_waypoint(start_location).next(2021)[
        0].get_left_lane().transform.location  # 圆桶3
    know_prop_traffic_location4 = env_map.get_waypoint(start_location).next(2023)[
        0].get_left_lane().transform.location  # 圆桶4
    know_prop_traffic_location5 = env_map.get_waypoint(start_location).next(2021)[0].transform.location  # 圆桶5
    know_prop_traffic_location6 = env_map.get_waypoint(start_location).next(2023)[0].transform.location  # 圆桶6
    know_prop_traffic_location7 = env_map.get_waypoint(start_location).next(2021)[
        0].get_right_lane().transform.location  # 圆桶7
    know_prop_traffic_location8 = env_map.get_waypoint(start_location).next(2023)[
        0].get_right_lane().transform.location  # 圆桶8
    # 800
    # 创建副车坐标列表
    vice_car_locations = []
    start_location_left1 = env_map.get_waypoint(start_location).get_left_lane().transform.location
    start_location_right1 = env_map.get_waypoint(start_location).get_right_lane().transform.location
    vice_car_locations.append(start_location_left1)
    vice_car_locations.append(start_location_right1)
    for i in range(800 // 110 - 3):
        start_location_left1 = env_map.get_waypoint(start_location_left1).next(110 * (i + 1))[0].transform.location
        start_location_right1 = env_map.get_waypoint(start_location_right1).next(110 * (i + 1))[0].transform.location
        vice_car_locations.append(start_location_left1)
        vice_car_locations.append(start_location_right1)

    # 创建熟悉过程所有圆桶标志列表
    props_location_list = [know_prop_traffic_location1, know_prop_traffic_location2, know_prop_traffic_location3,
                           know_prop_traffic_location4, know_prop_traffic_location5, know_prop_traffic_location6,
                           know_prop_traffic_location7, know_prop_traffic_location8]

    # 等待车子到达800米
    while main_vehicle.get_location().distance(start_location) < 800:
        sleep(1)
    # 创建两个圆桶交通标志
    vehicle_traffic.create_traffic(props_location_list[:2], 'static.prop.trafficcone01')
    # 创建多个副车1
    vice_car_locations = change_height(vice_car_locations)
    vice_vehicles1 = vehicle_traffic.create_vehicle(vice_car_locations, vehicle_model="mini")
    # 添加副车
    vice_car.cars = vice_vehicles1

    # 800+221=1021
    # 副车坐标列表
    vice_car_locations = []
    start_location_left2 = env_map.get_waypoint(start_location).next(1021)[
        0].get_left_lane().get_left_lane().transform.location
    start_location_right2 = env_map.get_waypoint(start_location).next(1021)[
        0].get_right_lane().get_right_lane().transform.location
    vice_car_locations.append(start_location_left2)
    vice_car_locations.append(start_location_right2)
    for i in range((1800 - 1021) // 110 - 2):
        start_location_left2 = env_map.get_waypoint(start_location_left2).next(110 * (i + 1))[0].transform.location
        start_location_right2 = env_map.get_waypoint(start_location_right2).next(110 * (i + 1))[0].transform.location
        vice_car_locations.append(start_location_left2)
        vice_car_locations.append(start_location_right2)
    while main_vehicle.get_location().distance(start_location) < 1021:
        sleep(1)
    # 创建剩余的圆桶
    vehicle_traffic.create_traffic(props_location_list[2:], 'static.prop.trafficcone01')
    # 创建多个副车2
    vice_car_locations = change_height(vice_car_locations)
    vice_vehicles2 = vehicle_traffic.create_vehicle(vice_car_locations, vehicle_model="mini")
    # 添加副车
    vice_car.cars += vice_vehicles2
    # 等待车子到达下一起点
    check_distance(main_vehicle, end_location)
    # 到达下一起点清除副车以及速度列表
    vice_car.cars = []
    # 销毁所有副车,里面有处理如果车子不存在就跳过
    destroy_car(vice_vehicles2 + vice_vehicles1)


@start_end_statement
def c1(main_vehicle, end_location):
    check_distance(main_vehicle, end_location)


@start_end_statement
def c2(main_vehicle, vehicle_traffic, end_location):
    """
    :param main_vehicle:主车
    :param vehicle_traffic:创建车子/交通标志的对象
    :param end_location:终点坐标
    """
    # 创建交通标志坐标
    location1 = env_map.get_waypoint(main_vehicle.get_location()).next(1521)[0].get_right_lane().transform.location
    location2 = env_map.get_waypoint(location1).get_right_lane().transform.location
    # 未到达目的地死循环不创建交通标志
    while main_vehicle.get_location().distance(location1) > 221:
        sleep(1)
    # 创建交通标志
    vehicle_traffic.create_traffic([location2, location1], "static.prop.trafficcone01")
    # 检查终点
    check_distance(main_vehicle, end_location)


@start_end_statement
def s1(main_vehicle, vehicle_traffic, end_location):
    # 创建交通标志坐标
    location1 = env_map.get_waypoint(main_vehicle.get_location()).next(1521)[0].transform.location
    location2 = env_map.get_waypoint(location1).next(2)[0].transform.location
    # 未到达目的地死循环不创建交通标志
    while main_vehicle.get_location().distance(location1) > 221:
        sleep(1)
    # 创建交通标志
    vehicle_traffic.create_traffic([location2, location1], "static.prop.trafficcone01")
    # 检查终点
    check_distance(main_vehicle, end_location)


@start_end_statement
def s2(main_vehicle, vehicle_traffic, vice_car, end_location):
    # 创建交通标志坐标
    traffic_location1 = env_map.get_waypoint(main_vehicle.get_location()).next(1521)[0].transform.location
    traffic_location2 = env_map.get_waypoint(traffic_location1).next(2)[0].transform.location
    traffic_locations = [traffic_location1, traffic_location2]

    # 未到达目的地死循环不创建交通标志
    while main_vehicle.get_location().distance(traffic_location1) > 221:
        sleep(1)
    # 创建交通标志
    vehicle_traffic.create_traffic(traffic_locations, "static.prop.trafficcone01")

    # 创建车流坐标
    vehicle_locations = []
    vehicle_location = env_map.get_waypoint(main_vehicle.get_location()).get_left_lane().previous(20)[
        0].transform.location
    vehicle_locations += [vehicle_location]
    for i in range(10):
        location = env_map.get_waypoint(vehicle_location).previous(110 * (i + 1))[0].transform.location
        vehicle_locations += [location]
    vehicle_locations = change_height(vehicle_locations)
    vice_vehicles = vehicle_traffic.create_vehicle(vehicle_locations, vehicle_model="mini")
    try:
        vice_car.cars += vice_vehicles
    except:
        vice_car.cars = vice_vehicles

    # 检查终点
    check_distance(main_vehicle, end_location)
    # 销毁车辆
    vice_car.cars = []
    destroy_car(vice_vehicles)


@start_end_statement
def s3(main_vehicle, vehicle_traffic, vice_car, end_location):
    # 圆桶坐标
    location1 = env_map.get_waypoint(main_vehicle.get_location()).next(1521)[0].transform.location
    location2 = env_map.get_waypoint(main_vehicle.get_location()).next(1523)[0].transform.location
    vehicle_location_now = main_vehicle.get_location()
    while main_vehicle.get_location().distance(vehicle_location_now) < 1300:
        sleep(1)
    # 副车列表坐标
    vice_locations = []
    vice_left_location1 = env_map.get_waypoint(main_vehicle.get_location()).get_left_lane().previous(20)[
        0].transform.location
    vice_right_location1 = env_map.get_waypoint(main_vehicle.get_location()).get_right_lane().previous(20)[
        0].transform.location
    vice_locations.append(vice_left_location1)
    vice_locations.append(vice_right_location1)
    for i in range(10):
        vice_left_location1 = env_map.get_waypoint(vice_left_location1).previous(110 * (i + 1))[0].transform.location
        vice_right_location1 = env_map.get_waypoint(vice_right_location1).previous(110 * (i + 1))[0].transform.location
        vice_locations.append(vice_left_location1)
        vice_locations.append(vice_right_location1)
    # 创建圆桶
    vehicle_traffic.create_traffic([location1, location2], traffic_model="static.prop.trafficcone01")
    # 创建车子
    vice_locations = change_height(vice_locations)
    vice_vehicles = vehicle_traffic.create_vehicle(vice_locations, vehicle_model="mini")
    vice_car.cars = vice_vehicles
    # 检查是否到达下一起点，没有到就一直死循环在这个函数
    check_distance(main_vehicle, end_location)
    destroy_car(vice_vehicles)

@start_end_statement
def s4():
    pass


@start_end_statement
def s5():
    pass


@start_end_statement
def s6():
    pass


def main():
    save_csv()
    global stage
    # 绘制起点箭头
    draw_arrow(
        [s6_1_location, start_location, c1_1_location, c2_1_location, s1_1_location, c1_2_location, c1_2_end_location,
         s2_1_location, c2_2_location, c2_2_end_location, c1_3_location, s3_1_location, c2_3_location, s4_1_location,
         c2_4_location, s5_1_location, s5_1_end_location, c1_4_location, c2_5_location, c2_5_end_location])

    # 创建车子以及交通标志的类
    vehicle_traffic = Vehicle_Traffic(world)
    # 创建主车
    main_vehicle = \
        vehicle_traffic.create_vehicle([s3_1_location], vehicle_model="mini", height=15)[0]
    # 创建窗口类
    window = Window(main_vehicle)
    # 启动窗口显示
    threading.Thread(target=window.show_screen).start()

    # 创建主车控制类
    main_car = Main_Car(main_vehicle)
    # 启动主车车辆的控制
    main_car.control_car()
    # 创建副车类
    vice_car = Vice_Car()
    # 启动副车
    vice_car.vice_car_follow_lane()

    # stage = "熟悉过程"
    # know_well_course(main_vehicle, vehicle_traffic, vice_car, c1_1_location)
    # # 改为自动驾驶  False为自动驾驶，True为人工驾驶
    # main_car.change_control = False
    #
    # stage = "c1_1"
    # c1(main_vehicle, c2_1_location)
    # # 改为自动驾驶  False为自动驾驶，True为人工驾驶
    # main_car.change_control = False
    #
    # stage = "c2_1"
    # c2(main_vehicle, vehicle_traffic, s1_1_location)
    # # 改为自动驾驶  False为自动驾驶，True为人工驾驶
    # main_car.change_control = False
    #
    # stage = "s1_1"
    # s1(main_vehicle, vehicle_traffic, c1_2_location)
    # # 改为自动驾驶  False为自动驾驶，True为人工驾驶
    # main_car.change_control = False
    #
    # stage = "c1_2"
    # c1(main_vehicle, c1_2_end_location)
    # # 改为自动驾驶  False为自动驾驶，True为人工驾驶
    # main_car.change_control = False
    #
    # stage = "自动转弯阶段"
    # check_distance(main_vehicle, s2_1_location)
    #
    # stage = "s2_1"
    # s2(main_vehicle, vehicle_traffic, vice_car, c2_2_location)
    # # 改为自动驾驶  False为自动驾驶，True为人工驾驶
    # main_car.change_control = False
    #
    # stage = "c2_2"
    # c2(main_vehicle, vehicle_traffic, c2_2_end_location)
    # main_car.change_control = False
    #
    # stage = "自动转弯阶段"
    # check_distance(main_vehicle, c1_3_location)
    #
    # stage = "c1_3"
    # c1(main_vehicle, s3_1_location)
    # main_car.change_control = False

    stage = "s3_1"
    s3(main_vehicle, vehicle_traffic, vice_car, c2_3_location)
    main_car.change_control = False


if __name__ == '__main__':
    main()

    # load_map(client, "400/400.xodr")
    # spectator = world.get_spectator()
    # spectator.set_transform(carla.Transform(start_location, carla.Rotation(pitch=0)))

    pass
