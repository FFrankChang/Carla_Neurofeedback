import threading
import carla
import pygame
import random
import csv
import time
from datetime import datetime
from pygame.locals import *
import numpy as np
import os
import math
import socket
import json
import sys

from sensor.steering_angle import parse_euler, get_steering_angle
from sensor.pedal import get_data,pedal_receiver

drive_status = "自动驾驶"  
scene_status = "简单场景"  
system_fault = False
easy_location1 = carla.Location(x=100, y=13, z=5)


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
        self.tm.global_percentage_speed_difference(0)
        self.autopilot_vehicles = []  # 存储所有可能自动驾驶的车辆
        self.activation_distance = 100  # 激活距离（米）
        self.main_vehicle = None

    def create_vehicle(self, points=None, vehicle_model=None):
        colors = ['255,0,0']
        vehicles = []
        if vehicle_model:
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            cars = self.blueprint_library.filter('*Crown*')

        autopilot_indices = random.sample(range(len(points)), int(len(points) * 0.9))

        for index, point in enumerate(points):
            car_blueprint = random.choice(cars)
            car_color = random.choice(colors)
            car_blueprint.set_attribute('color', car_color)

            waypoint = self.env_map.get_waypoint(point)  
            transform = carla.Transform(point, waypoint.transform.rotation)
            vehicle = self.world.try_spawn_actor(car_blueprint, transform)
            
            if vehicle:
                vehicles.append(vehicle)
                if index in autopilot_indices:
                    self.autopilot_vehicles.append(vehicle)  # 添加到潜在自动驾驶车辆列表
                    # 初始状态为静止
                    vehicle.set_autopilot(False)
                
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

    def set_main_vehicle(self, vehicle):
        """设置主车引用"""
        self.main_vehicle = vehicle

# 主车控制器
class Main_Car_Control:
    def __init__(self, main_car, world, window, instantaneous_speed=False):
        """
        主车控制类
        :param main_car: 主车对象
        :param instantaneous_speed: 是否瞬时速度
        """
        self.vehicle = main_car
        self.instantaneous_speed = instantaneous_speed
        self.scene_status = "简单场景"
        self.world = world
        self.autopilot_flag = False
        self.speed_limit = 100
        self.steer = 0
        self.throttle = 0
        self.brake = 0
        self.window = window
        self.collision_occurred = False
        self.start_time = time.time()
        self.collision_time = None
        self.running = True

    def follow_road(self):
        global drive_status
        fault_start_time = None
        rapid_acceleration_time = None
        
        while self.running:
            drive_status = "人工驾驶"
            steer, throttle, brake = get_sensor_data()
            self.steer = steer
            self.throttle = throttle
            self.brake = brake
            self.speed = self.get_speed()
            self.window.speed = self.speed

            car_control(self.vehicle, steer, throttle, brake)
                
            if self.collision_occurred:
                break

    def collision_event(self, event):
        if not self.collision_occurred:
            self.collision_occurred = True
            self.collision_time = time.time() - self.start_time - self.window.start_show_esc_after
            collision_message = f"Collision!"
            self.window.set_collision_info(collision_message)
            print(collision_message)

    def get_speed(self):
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return int(speed * 3.6)

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
        self.start_show_esc_after = 3
        self.show_duration = 3
        self.start_time = time.time()
        self.show_esc_time = self.start_time + self.start_show_esc_after
        self.end_esc_time = self.show_esc_time + self.show_duration
        self.speed = 0
        self.show_png = False
        self.esp_png = pygame.image.load(r".\resource\esp-1.png") 
        self.esp_png = pygame.transform.scale(self.esp_png, (60, 60))  
        self.attention_png = pygame.image.load(r".\resource\attention.png")
        self.attention_png = pygame.transform.scale(self.attention_png, (100, 100)) 
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
        pro = (time.time() - self.start_time - self.start_show_esc_after)
        if pro <= 0:
            pro = 0
        pro = pro / 150
        
        # self.screen.blit(self.esp_png, (self.SCREEN_WIDTH // 2 -300, 90))  # 调整位置
        # self.draw_text("slipperiness of the ground", 30, (self.SCREEN_WIDTH // 2 -600, 90), bold=True,color=(255, 255, 255))
        self.draw_text(f"{self.speed}", 50, (self.SCREEN_WIDTH // 2, self.SCREEN_HEIGHT // 1.8), bold=True,color=(255, 255, 255))
        if self.collision_info:
            self.draw_text(self.collision_info, 150, (self.SCREEN_WIDTH // 2 -200, self.SCREEN_HEIGHT // 3 -50), bold=True, color=(255, 255, 255))
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
    current_control = vehicle.get_control()
    steer = round((current_control.steer + steer) * 0.5, 3)
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
#     steer = joystick.get_axis(0)
#     acc = round((-joystick.get_axis(6) + 1)/2,3) 
#     steerCmd = round(K1 * math.tan(1.1 * steer),3)
#     return steerCmd, acc, 0

def get_sensor_data():
    K1 = 0.55
    steer = get_steering_angle() / 450
    steerCmd = K1 * math.tan(1.1 * steer)
    acc,brake = get_data()
    if acc > 0.1:
        brake =0
    return  steerCmd, acc, brake 


def scene_jian( main_car_control):  # 简单场景
    global scene_status
    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    scene_status = "简单场景"

def set_speed(vehicle, speed_kmh):

    speed = speed_kmh / 3.6
    forward_vector = vehicle.get_transform().rotation.get_forward_vector()
    vehicle.set_target_velocity(
        carla.Vector3D(forward_vector.x * speed, forward_vector.y * speed, forward_vector.z * speed))


def generate_difficulty_increasing_obstacles(
    base_location, 
    num_vehicles=150, 
    x_range=(150, 1200),
    y_range=(-12.5, 12.5),
    z=3,
    min_distance=7,
    safe_zone_radius=10,
    num_lanes=5,
    segments=21,  
    segment_length=50  # 每段长度50米
):
    random_locations = []
    base_x, base_y, base_z = base_location.x, base_location.y, base_location.z
    
    # 定义车道中心位置
    lane_centers = np.linspace(y_range[0] + (y_range[1]-y_range[0])/(2*num_lanes), 
                               y_range[1] - (y_range[1]-y_range[0])/(2*num_lanes), 
                               num_lanes)
    
    # 计算每个段的障碍车数量，后面的段分配更多障碍车
    obstacles_per_segment = []
    total = 0
    for i in range(segments):
        # 使用线性增长或其他函数（如指数）来分配障碍车
        weight = i + 1  # 线性增长
        obstacles = weight * 2  # 每段基础障碍车数量
        obstacles_per_segment.append(obstacles)
        total += obstacles
    # 归一化以确保总数接近num_vehicles
    scale = num_vehicles / total
    obstacles_per_segment = [math.ceil(o * scale) for o in obstacles_per_segment]
    
    # 初始开放车道（可以随机选择）
    previous_open_lane = random.randint(0, num_lanes-1)
    
    for i in range(segments):
        current_x_min = x_range[0] + i * segment_length
        current_x_max = current_x_min + segment_length
        current_num = obstacles_per_segment[i]
        
        # 根据段的位置决定需要阻塞的车道数
        # 随着段数增加，逐步增加阻塞车道数量
        if i < segments * 0.3:
            blocked_lanes_count = 2  # 前30%段阻塞2条车道
        elif i < segments * 0.6:
            blocked_lanes_count = 3  # 中间30%段阻塞3条车道
        elif i < segments * 0.8:
            blocked_lanes_count = 4  # 后20%段阻塞4条车道
        else:
            blocked_lanes_count = 5  # 最后20%段阻塞所有车道（确保高难度）
        
        # 确保至少留出一条开放车道
        blocked_lanes_count = min(blocked_lanes_count, num_lanes - 1)
        
        # 随机选择要阻塞的车道，避免连续多个段开放同一车道
        available_lanes = set(range(num_lanes))
        # 尝试避免与前一段的开放车道相同
        possible_open_lanes = available_lanes.copy()
        if previous_open_lane in possible_open_lanes:
            possible_open_lanes.remove(previous_open_lane)
        open_lane = random.choice(list(possible_open_lanes)) if possible_open_lanes else previous_open_lane
        previous_open_lane = open_lane
        
        blocked_lanes = set()
        while len(blocked_lanes) < blocked_lanes_count:
            lane = random.randint(0, num_lanes-1)
            if lane != open_lane and lane not in blocked_lanes:
                blocked_lanes.add(lane)
        
        # 为当前段生成障碍车
        for _ in range(current_num):
            # 随机选择阻塞的车道
            if not blocked_lanes:
                break  # 无需阻塞车道
            lane = random.choice(list(blocked_lanes))
            random_y = lane_centers[lane] + np.random.uniform(-1.5, 1.5)  # 在车道中心附近随机偏移
            
            # x位置在当前段内均匀分布
            random_x = np.random.uniform(current_x_min, current_x_max)
            
            # 检查安全区域
            distance_to_base = math.hypot(base_x - random_x, base_y - random_y)
            if distance_to_base < safe_zone_radius:
                continue
            
            # 检查与已有障碍车的最小距离
            too_close = False
            for loc in random_locations:
                if math.hypot(loc.x - random_x, loc.y - random_y) < min_distance:
                    too_close = True
                    break
            if too_close:
                continue
            
            # 添加障碍车位置
            random_locations.append(carla.Location(x=random_x, y=random_y, z=z))
        
        while len(random_locations) < sum(obstacles_per_segment[:i+1]):
            lane = random.choice(list(blocked_lanes))
            random_y = lane_centers[lane] + np.random.uniform(-1.5, 1.5)
            random_x = np.random.uniform(current_x_min, current_x_max)
            
            distance_to_base = math.hypot(base_x - random_x, base_y - random_y)
            if distance_to_base < safe_zone_radius:
                continue
            
            too_close = False
            for loc in random_locations:
                if math.hypot(loc.x - random_x, loc.y - random_y) < min_distance:
                    too_close = True
                    break
            if too_close:
                continue
            
            random_locations.append(carla.Location(x=random_x, y=random_y, z=z))
    
    return random_locations


def get_locations(main_vehicle, vehicles):
    main_vehicle_location = main_vehicle.get_location()
    main_vehicle_info = {
        "id": main_vehicle.id,
        "loc": {
            "x": round(main_vehicle_location.x, 2),
            "y": round(main_vehicle_location.y, 2),
            "z": round(main_vehicle_location.z, 2)
        }
    }
    # 获取其他车辆的位置
    traffic_vehicles = []
    for vehicle in vehicles:
        location = vehicle.get_location()
        vehicle_info = {
            "id": vehicle.id,
            "loc": {
                "x": round(location.x, 2),
                "y": round(location.y, 2),
                "z": round(location.z, 2)
            }
        }
        traffic_vehicles.append(vehicle_info)
    loc_dic = {
        "main_vehicle": main_vehicle_info,
        "traffic": traffic_vehicles
    }
    return loc_dic

def forward_traffic_location(main_vehicle, traffic):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP套接字
    target_ip = "192.168.31.144"  
    target_port = 5005  
    
    while True:
        locations = get_locations(main_vehicle, traffic)
        message = json.dumps(locations)
        sock.sendto(message.encode(), (target_ip, target_port))
        time.sleep(0.1)

def random_locations(base_location, num_vehicles=10, x_range=(-100, 100), y_range=(-50, 50), z=5):
    random_locations = []
    base_x, base_y, base_z = base_location.x, base_location.y, base_location.z
    for i in range(num_vehicles):
        random_x = np.random.uniform(x_range[0], x_range[1])
        random_y = base_y + random.uniform(*y_range)
        random_locations.append(carla.Location(x=random_x, y=random_y, z=z))
    return random_locations

class DataRecorder:
    def __init__(self, subject, date, condition, frequency=60):
        self.last_record_time = time.time()
        self.interval = 1.0 / frequency
        self.collision_occurred = False
        self.collision_time = None
        self.subject = subject
        self.date = date
        self.condition = condition
        self.file = None
        self.filename = None
        self.init_file()

    def init_file(self):
        self.filename = f"./data/carla_s06_{self.subject}_{self.date}_{self.condition}_{datetime.now().strftime('%Y%m%d%H%M%S')}"
        temp_filename = f"{self.filename}.csv"
        self.file = open(temp_filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'Speed', 'Location_x', 'Location_y', 'Location_z', 
                            'Steer', 'Throttle', 'Brake', 'Collision_Time'])

    def record_collision(self, collision_time):
        self.collision_occurred = True
        self.collision_time = collision_time
        self.writer.writerow(['', '', '', '', '', '', '', '', collision_time])
        self.file.flush()

    def record_data(self, timestamp, speed, location, steer, throttle, brake):
        current_time = time.time()
        if current_time - self.last_record_time >= self.interval:
            self.writer.writerow([
                timestamp, speed, 
                location.x, location.y, location.z,
                steer, throttle, brake,
                '' 
            ])
            self.last_record_time = current_time
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            # 重命名文件以包含碰撞时间
            if self.collision_occurred and self.collision_time is not None:
                old_filename = f"{self.filename}.csv"
                new_filename = f"{self.filename}_{int(self.collision_time)}.csv"
                try:
                    os.rename(old_filename, new_filename)
                except Exception as e:
                    print(f"Error renaming file: {e}")

if __name__ == '__main__':
    subject_id = "SUBJECT" if len(sys.argv) < 2 else sys.argv[1]
    date = "DAY" if len(sys.argv) < 3 else sys.argv[2]
    condition = "CONDITION" if len(sys.argv) < 4 else sys.argv[3]
    
    client = carla.Client("127.0.0.1", 2000)  # 连接carla
    client.set_timeout(60)  
    world = client.get_world()  # 获取世界对象
    env_map = world.get_map()  # 获取地图对象
    spectator = world.get_spectator()  # ue4中观察者对象
    blueprint_library = world.get_blueprint_library()  # 获取蓝图，可以拿到可创建的对象
    prop_model = blueprint_library.filter('*prop*') 

    pygame.init()
    pygame.mixer.init()

    # joystick = pygame.joystick.Joystick(0)
    # joystick.init()

    threading.Thread(target=pedal_receiver).start()
    threading.Thread(target=parse_euler,daemon=True).start()


    destroy_all_vehicles_traffics(world)  
    random_traffic_points = generate_difficulty_increasing_obstacles(
        base_location=easy_location1, 
        num_vehicles=50,  
        x_range=(1000, 2500),
        y_range=(0, 25),
        z=3,
        min_distance=7,
        safe_zone_radius=10,
        num_lanes=5,
        segments=40,  
        segment_length=50
    )

    vehicle_traffic = Vehicle_Traffic(world)  
    traffic_1 = random_locations(easy_location1, num_vehicles=8, x_range=(0, 500), y_range=(-12.5, 12.5), z=3)
    vehicle_traffic.create_vehicle(points=traffic_1)

    vehicle = vehicle_traffic.create_main_vehicle([easy_location1], vehicle_model="vehicle.tesla.model3")[0]
    vehicle_traffic.set_main_vehicle(vehicle)  # 设置主车引用
    random_traffic = vehicle_traffic.create_vehicle(points=random_traffic_points)
    threading.Thread(target=forward_traffic_location, args=(vehicle,random_traffic), daemon=True).start()

    window = Window(world, vehicle_traffic.blueprint_library, vehicle)
    main_car_control = Main_Car_Control(vehicle, world, window, True)
    collision_sensor = vehicle_traffic.attach_collision_sensor(vehicle, main_car_control.collision_event)

    scene_jian(main_car_control)
    while True:
        world.tick()
        time.sleep(0.01)
