import threading
import carla
import pygame
from s04_config_traffic import *
import random
import csv
import time
from datetime import datetime
from sensor.steering_angle import parse_euler, get_steering_angle
from sensor.pedal import get_data,pedal_receiver
import numpy as np
import math
from pygame.locals import *
import os

vices_car_list = []  # 所有副车列表
drive_status = "自动驾驶"  
scene_status = "简单场景"  
system_fault = False

def change_weather(world, gradual_steps=10, duration=10):

    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=0.0,
        sun_azimuth_angle=30.0,
        sun_altitude_angle=90.0
    )
    world.set_weather(weather)

    for i in range(1, gradual_steps + 1):
        weather.cloudiness = i * (100 / gradual_steps)
        weather.precipitation = i * (100 / gradual_steps)
        weather.precipitation_deposits = i * (10 / gradual_steps)
        weather.sun_altitude_angle = max(90 - i * (90 / gradual_steps), 0)
        world.set_weather(weather)
        time.sleep(duration / gradual_steps)


class Vehicle_Traffic:
    def __init__(self, world, tm_port=8000):
        # 世界对象
        self.world = world
        # 地图对象
        self.env_map = self.world.get_map()
        # 创建蓝图
        self.blueprint_library = self.world.get_blueprint_library()

        # Traffic Manager
        self.tm = client.get_trafficmanager(tm_port)  # 默认Traffic Manager端口8000
        self.tm.set_synchronous_mode(True)  
        self.tm.global_percentage_speed_difference(-270)

    def create_vehicle(self, points=None,  vehicle_model=None):
            # 定义一个颜色列表
        colors = [
            '0,0,0',    
            '10,10,150',
            '230,230,0',  
            '255,165,0', 
            '255,255,255' 
        ]
        vehicles = []
        if vehicle_model:
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            cars = self.blueprint_library.filter('*Crown*')

        for index, point in enumerate(points):
            car_blueprint = random.choice(cars)
            car_color = random.choice(colors)
            car_blueprint.set_attribute('color', car_color)

            waypoint = self.env_map.get_waypoint(point)  
            transform = carla.Transform(point, waypoint.transform.rotation)
            vehicle = self.world.try_spawn_actor(car_blueprint, transform)
            if vehicle:
                vehicles.append(vehicle)
                # 加入Traffic Manager管理的车辆
                vehicle.set_autopilot(True, self.tm.get_port())
                self.tm.auto_lane_change(vehicle, False)
                vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Brake | carla.VehicleLightState.HighBeam))
            else:
                print(f"索引为：{index}的车子未成功生成！")
        return vehicles
    
    def create_main_vehicle(self, points=None, vehicle_model=None):
        vehicles = []
        if vehicle_model:
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            cars = self.blueprint_library.filter('*tesla*')

        for index, point in enumerate(points):
            waypoint = self.env_map.get_waypoint(point)  
            transform = carla.Transform(point, waypoint.transform.rotation)
            vehicle = self.world.try_spawn_actor(random.choice(cars), transform)
            if vehicle:
                vehicles.append(vehicle)
                light_front = carla.VehicleLightState.HighBeam
                vehicle.set_light_state(carla.VehicleLightState(light_front))

            else:
                print(f"索引为：{index}的车子未成功生成！")
        return vehicles
    
    def attach_collision_sensor(self, vehicle, callback):
        """
        在给定车辆上添加碰撞传感器并设置回调。
        :param vehicle: 车辆对象
        :param callback: 碰撞发生时调用的函数
        """
        blueprint = self.blueprint_library.find('sensor.other.collision')
        collision_sensor = self.world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
        collision_sensor.listen(callback)
        return collision_sensor
# 主车控制器
class Main_Car_Control:
    def __init__(self, main_car, world, window,instantaneous_speed=False):
        """
        主车控制类
        :param main_car: 主车对象
        :param instantaneous_speed: 是否瞬时速度
        """
        self.vehicle = main_car  # 主车对象
        self.instantaneous_speed = instantaneous_speed  
        self.scene_status = "简单场景"
        self.world = world
        self.autopilot_flag = False  
        self.speed_limit = 100  
        self.lead_vehicle = None
        self.next_event_time = time.time() + 30
        self.steer_duration = 0  
        self.steer_event_end_time = 0  
        self.random_steer_active = False
        self.random_steer_value = 0
        self.steer = 0
        self.throttle = 0
        self.brake = 0
        self.window = window  # 传递 Window 实例
        self.collision_occurred = False
        self.start_time = time.time()
        self.collision_time = None
        self.running = True

    def follow_road(self):
        global drive_status
        # pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while self.running:
            drive_status = "人工驾驶"
            steer, throttle, brake = get_sensor_data()
            self.steer = steer
            self.throttle = throttle
            self.brake = brake
            if system_fault:
                car_control(self.vehicle, steer, 0.85, 0)
                # set_speed(self.vehicle,90)
            else:
                car_control(self.vehicle, steer, throttle, brake)
                # car_control(self.vehicle, steer, abs(throttle),0.1)
            if self.collision_occurred:
                break
            # time.sleep(0.01)

    def collision_event(self, event):
        if not self.collision_occurred:
            self.collision_occurred = True
            self.collision_time = time.time() - self.start_time
            collision_message = f"Collision! {self.collision_time:.2f} s"
            self.window.set_collision_info(collision_message)  # 设置窗口中显示的碰撞信息
            print(collision_message)
            # self.stop_scenario()  # 停止场景

    def stop_scenario(self):
        self.running = False
        self.world.wait_for_tick()  # 等待当前tick完成
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            vehicle.apply_control(carla.VehicleControl(hand_brake=True, throttle=0.0))
        print("Scenario paused due to collision.")
class Window:
    def __init__(self, world, blueprint_library, vehicle):
        """
        创建车子的pygame窗口显示
        :param world: 世界对象
        :param blueprint_library: 蓝图对象
        :param vehicle: 车子对象
        """
        self.world = world
        self.vehicle = vehicle
        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360  
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 5760, 1080  
        self.screen = None  # 初始化屏幕窗口
        self.fonts = {} 
        pygame.init()  # 初始化pygame
        self.collision_info = None  # 存储碰撞信息

        # 初始化窗口设置
        self.clock = pygame.time.Clock()
        self.size = 18  # 字体大小
        self.fps = 90  # 帧率
        self.font = pygame.font.Font(r"TTF\宋体.ttf", self.size)  # 初始化字体对象

        # 初始化传感器
        self.blueprint_camera = blueprint_library.find('sensor.camera.rgb')  # 选择一个传感器蓝图
        self.blueprint_camera.set_attribute('image_size_x', f'{self.SCREEN_WIDTH}')  # 传感器获得的图片高度
        self.blueprint_camera.set_attribute('image_size_y', f'{self.SCREEN_HEIGHT}')  # 传感器获得的图片宽度
        self.blueprint_camera.set_attribute('fov', '140')  # 水平方向上能看到的视角度数
        spawn_point = carla.Transform(carla.Location(x=1.8, y = -0.3, z=1.25), carla.Rotation(pitch=-8, yaw=0, roll=0))  # 传感器相对车子的位置设置
        self.sensor = self.world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)  # 添加传感器
        self.show_esc = False
        self.start_show_esc_after = 30
        self.show_duration = 3
        self.start_time = time.time()
        self.show_esc_time = self.start_time + self.start_show_esc_after
        self.end_esc_time = self.show_esc_time + self.show_duration

        threading.Thread(target=self.show_screen).start()

    def show_screen(self):
        pygame.display.set_caption("pygame模拟场景")
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), pygame.RESIZABLE, 32)
        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            current_time = time.time()
            if self.show_esc_time <= current_time < self.end_esc_time:
                self.show_esc = True
                global system_fault
                system_fault = True

            else:
                self.show_esc = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    os._exit(0)
                elif event.type == pygame.VIDEORESIZE:
                    self.CreateWindow(event.w, event.h)
            pygame.display.update()
            self.clock.tick(self.fps)

    # 加载图片
    def process_img(self, image):
        global drive_status
        i = np.array(image.raw_data)
        i2 = i.reshape((self.SCREEN_HEIGHT, self.SCREEN_WIDTH, 4))
        i3 = i2[:, :, :3]
        i3 = np.rot90(i3, k=1, axes=(0, 1))
        i3 = i3[..., ::-1]
        img_surface = pygame.surfarray.make_surface(np.flip(i3, axis=0))
        self.screen.blit(img_surface, (0, 0))  # 绘制图片
        if self.show_esc:
            self.draw_text("动力系统故障！", 150, (self.SCREEN_WIDTH // 2 -300, self.SCREEN_HEIGHT // 3), bold=True,color=(255, 0, 0))

        if self.collision_info:
            self.draw_text(self.collision_info, 150, (self.SCREEN_WIDTH // 2 -300, self.SCREEN_HEIGHT // 3), bold=True, color=(255, 255, 255))

        pygame.display.flip()
        fps = self.clock.get_fps()

    def draw_text(self, word, size, position, bold=False, color=(255, 0, 0)):
        font_key = (size, bold)
        if font_key not in self.fonts:
            font = pygame.font.Font(r"TTF\宋体.ttf", size)
            font.set_bold(bold)
            self.fonts[font_key] = font
        else:
            font = self.fonts[font_key]
        
        text = font.render(word, True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.screen.blit(text, text_rect)

    def set_collision_info(self, info):

        self.collision_info = info

def car_control(vehicle, steer=0, throttle=1, brake=0):
    steer = round(steer, 3)
    throttle = round(throttle, 3)
    brake = round(brake, 3)
    control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
    vehicle.apply_control(control)
    # print(steer,throttle,brake)

def destroy_all_vehicles_traffics(world, vehicle_flag=True, traffic_flag=True):
    actors = []
    if vehicle_flag:
        actors += list(world.get_actors().filter('*vehicle*'))
    if traffic_flag:
        actors += list(world.get_actors().filter("*prop*"))
    for actor in actors:
        actor.destroy()


# def get_sensor_data():
#     K1 = 0.55
#     steer = round(joystick.get_axis(0),3) 
#     steerCmd = K1 * math.tan(1.1 * steer)
#     return steerCmd, (-joystick.get_axis(1) + 1)/2, (-joystick.get_axis(2) + 1)/2

def get_sensor_data():
    K1 = 0.35
    steer = get_steering_angle() / 450
    steerCmd = K1 * math.tan(1.1 * steer)
    acc,brake = get_data()
    if acc > 0.1:
        brake =0
    # print(steer, acc, brake)
    return  steerCmd, acc, brake 


def scene_jian( main_car_control, end_location):  # 简单场景
    global scene_status, vices_car_list
    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    scene_status = "简单场景"

def set_speed(vehicle, speed_kmh):

    speed = speed_kmh / 3.6
    forward_vector = vehicle.get_transform().rotation.get_forward_vector()
    vehicle.set_target_velocity(
        carla.Vector3D(forward_vector.x * speed, forward_vector.y * speed, forward_vector.z * speed))

def generate_random_locations_around_vehicle(base_location, num_vehicles=100, x_range=(-100, 100), y_range=(-50, 50), z=5, min_distance=7, safe_zone_radius=10):
    random_locations = []
    base_x, base_y, base_z = base_location.x, base_location.y, base_location.z
    
    while len(random_locations) < num_vehicles:
        random_x = base_x + random.uniform(*x_range)
        random_y = base_y + random.uniform(*y_range)
        valid_location = True

        # Check if the new location is within the safe zone around the base vehicle
        if ((base_x - random_x) ** 2 + (base_y - random_y) ** 2) ** 0.5 < safe_zone_radius:
            continue
        
        # Check if new location is too close to any existing location
        for location in random_locations:
            if ((location.x - random_x) ** 2 + (location.y - random_y) ** 2) ** 0.5 < min_distance:
                valid_location = False
                break
        
        if valid_location:
            random_locations.append(carla.Location(x=random_x, y=random_y, z=z))
    
    return random_locations



if __name__ == '__main__':
    client = carla.Client("127.0.0.1", 2000)  # 连接carla
    client.set_timeout(60)  # 设置超时
    world = client.get_world()  # 获取世界对象
    env_map = world.get_map()  # 获取地图对象
    spectator = world.get_spectator()  # ue4中观察者对象
    blueprint_library = world.get_blueprint_library()  # 获取蓝图，可以拿到可创建的对象
    vehicle_points = env_map.get_spawn_points()  # 拿到世界中所有可绘制车辆的点坐标
    # inside_junction = env_map.get_waypoint().is_junction()  # 获取路点的车道信息
    vehicle_models = blueprint_library.filter('*mini*')  # 拿到所有可绘制的车辆模型
    prop_model = blueprint_library.filter('*prop*')  # 拿到所有可绘制的交通标志模型

    pygame.init()
    pygame.mixer.init()

    # joystick = pygame.joystick.Joystick(0)
    # joystick.init()

    threading.Thread(target=pedal_receiver).start()
    threading.Thread(target=parse_euler,daemon=True).start()

    destroy_all_vehicles_traffics(world)  
    random_traffic_points = generate_random_locations_around_vehicle(
        easy_location1, 
        num_vehicles=50, 
        x_range=(50, 900),  
        y_range=(-12.5, 12.5),    
        z=3        
    )
    weather_thread = threading.Thread(target=change_weather, args=(world, 100, 30))
    weather_thread.start()
    vehicle_traffic = Vehicle_Traffic(world)  
    vehicle = vehicle_traffic.create_main_vehicle([easy_location1], vehicle_model="vehicle.tesla.model3")[0]
    random_traffic = vehicle_traffic.create_vehicle(points=random_traffic_points)

    window = Window(world, vehicle_traffic.blueprint_library, vehicle)
    main_car_control = Main_Car_Control(vehicle, world, window,True)
    collision_sensor = vehicle_traffic.attach_collision_sensor(vehicle, main_car_control.collision_event)

    scene_jian(main_car_control, interfere_one_location1)
    
    while True:
        world.tick()  # 确保同步更新
        time.sleep(0.01)
