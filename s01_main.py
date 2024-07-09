import threading
import time
import socket
import carla
import pygame
from disposition import *
from s01_config import *
import random
import csv
from datetime import datetime 

vices_car_list = []  # 所有副车列表
drive_status = "自动驾驶"  # 驾驶状态
directions = []  # 变道顺序, 前提前方有车
scene_status = ""  # 干扰场景一   简单场景
left_right_qian_distance = 0
left_right_hou_distance = 0
volume_size=0.5  # 音量大小
global last_steer 
last_steer =0

class DataRecorder:
    def __init__(self, filename=f"./data/carla_{datetime.now().strftime('%Y%m%d%H%M%S')}.csv",frequency=100):
        self.filename = filename
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'Speed', 'Location_x', 'Location_y', 'Location_z', 'Steer', 'Acceleration_x', 'Acceleration_y', 'Acceleration_z', 'Gyro_x', 'Gyro_y', 'Gyro_z', 'Compass', 'Lead_Vehicle_Speed', 'Lead_Vehicle_X', 'Lead_Vehicle_Y', 'Lead_Vehicle_Z', 'Collision', 'TOR','Mode_Switched'])
        self.last_record_time = time.time()
        self.interval = 1.0 / frequency
        self.collision_detected = False
        self.mode_switched = False
        self.tor = False
    def record_collision(self):
        self.collision_detected = True

    def record_tor(self):
        self.tor = True

    def record_mode_switch(self):
        self.mode_switched = True

    def record_data(self, timestamp, speed, location, steer, acceleration, gyro, compass, lead_vehicle_speed, lead_vehicle_location):
        current_time = time.time()
        if current_time - self.last_record_time >= self.interval:
            collision_status = 'Yes' if self.collision_detected else 'No'
            mode_switched_status = 'Yes' if self.mode_switched else ''
            tor_status = 'Yes' if self.tor else ''
            self.writer.writerow([
                timestamp, speed, location.x, location.y, location.z, steer,
                acceleration.x, acceleration.y, acceleration.z, gyro.x, gyro.y, gyro.z,
                compass, lead_vehicle_speed,
                lead_vehicle_location.x if lead_vehicle_location else None,
                lead_vehicle_location.y if lead_vehicle_location else None,
                lead_vehicle_location.z if lead_vehicle_location else None,
                collision_status, tor_status,mode_switched_status
            ])
            self.last_record_time = current_time
            self.collision_detected = False  
            self.mode_switched = False  
            self.tor = False  
            self.file.flush()

    def close(self):
        self.file.close()

class Vehicle_Traffic:
    def __init__(self, world):
        # 世界对象
        self.world = world
        # 地图对象
        self.env_map = self.world.get_map()
        # 创建蓝图
        self.blueprint_library = self.world.get_blueprint_library()

    def create_vehicle(self, points=None, number=1, vehicle_model=None, height=0.05):
        """
            传入指定坐标点生成随机车辆(不传默认)，返回生成成功的下标,车辆对象,以及位置信息

            点的格式是   [(191.8, 120), (200, 100).......]

            默认生成车辆十辆

            可选车型：diamondback, harley-davidson, bmw, kawasaki, micro, yamaha, audi, ford, seat, chevrolet, tesla, dodge,
            volkswagen, mercedes, citroen, nissan, mitsubishi, lincoln, mini, bh, vespa, carlamotors, jeep, gazelle, toyota
        """
        vehicles = []
        # 如果传了车型就用指定车型，否则随机车型
        if vehicle_model:
            # 获取车辆的蓝图
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            # 筛选出指定车型
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            # 获取车辆的蓝图
            blueprint_car = self.blueprint_library.filter('*crown*')
            cars = [
                x for x in blueprint_car 
                if int(x.get_attribute('number_of_wheels')) == 4 
                and x.get_attribute('has_lights').as_bool() == True
                and x.get_attribute('base_type').as_str() == 'car'
            ]
        # 如果没有传入生成点，随机位置生成
        if not points:
            # 获取地图可创建的坐标点
            spawn_points = self.env_map.get_spawn_points()
            for index in range(number):
                # 随机位置生成车辆
                vehicle = self.world.try_spawn_actor(random.choice(cars), random.choice(spawn_points))
                if vehicle:
                    vehicles.append(vehicle)
                else:
                    print(f"第{index + 1}辆车未成功生成！")
            check_coordinate(vehicles)
            return vehicles
        # 创建指定数量的车
        for index, point in enumerate(points):
            waypoint = self.env_map.get_waypoint(point)  # 获取车辆生成点附近的道路信息
            # 设置生成点和初始旋转角度
            transform = carla.Transform(point, waypoint.transform.rotation)
            # 生成车辆
            vehicle = self.world.try_spawn_actor(random.choice(cars), transform)
            if vehicle:
                vehicles.append(vehicle)
            else:
                print(f"索引为：{index}的车子未成功生成！")
        check_coordinate(vehicles)
        return vehicles

    # 创建交通标志
    def create_traffic(self, points, traffic_model=None):
        traffics = []
        if not traffic_model:
            print("没有传入交通标志类型！")
            return None
        for location in points:
            # 获取交通标志的蓝图
            traffic_cone_bp = self.blueprint_library.find(traffic_model)
            # 创建交通标志
            traffic = self.world.try_spawn_actor(traffic_cone_bp, carla.Transform(location, carla.Rotation()))
            if traffic:
                traffics.append(traffic)
            else:
                print(f"{location}交通标志未生成成功！")
        return traffics


# 主车控制器
class Main_Car_Control:
    def __init__(self, main_car, world,data_recorder,instantaneous_speed=False):
        """
        主车控制类
        :param main_car: 主车对象
        :param instantaneous_speed: 是否瞬时速度
        """
        self.vehicle = main_car  # 主车对象
        self.instantaneous_speed = instantaneous_speed  # 是否瞬时到达目标速度
        self.scene_status = "简单场景"
        self.data_recorder = data_recorder
        self.world = world
        self.autopilot_flag = True  # 是否自动驾驶
        self.road_id = 4  # 主车所在道路id
        self.speed_limit = 100  # 主车速度限制
        self.flag = True
        self.udp_ip = "127.0.0.1"  # IP of the destination computer
        self.udp_port = 12346  # Port number on the destination computer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        self.imu_sensor = self.world.spawn_actor(imu_bp, carla.Transform(), attach_to=self.vehicle)
        self.imu_data = None
        self.imu_sensor.listen(lambda data: self._on_imu_update(data))
        self.lead_vehicle = None

    def update_lead_vehicle(self):
        vehicles = self.world.get_actors().filter('vehicle.*')
        vehicles = [v for v in vehicles if v.id != self.vehicle.id]
        self.lead_vehicle = None
        min_distance = float('inf')
        vehicle_location = self.vehicle.get_location()
        vehicle_forward = self.vehicle.get_transform().get_forward_vector()

        for v in vehicles:
            v_location = v.get_location()
            vector_to_v = v_location - vehicle_location
            distance = vector_to_v.length()
            if vector_to_v.dot(vehicle_forward) > 0:  # 确认车辆在前方
                if distance < min_distance:
                    min_distance = distance
                    self.lead_vehicle = v

    def _on_imu_update(self, data):
        self.imu_data = data
        self.update_lead_vehicle()
        # 记录前车速度和位置
        lead_vehicle_speed = get_speed(self.lead_vehicle) if self.lead_vehicle else None
        lead_vehicle_location = self.lead_vehicle.get_location() if self.lead_vehicle else None
        self.data_recorder.record_data(
            time.time(),
            get_speed(self.vehicle),
            self.vehicle.get_location(),
            get_steering_wheel_info(),
            data.accelerometer,
            data.gyroscope,
            data.compass,
            lead_vehicle_speed,
            lead_vehicle_location
        )


    def follow_road(self):
        global drive_status, scene_status, directions, volume_size
        self.flag = True
        takeover_prompted = False
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while self.flag:
            self.speed_limit = road_speed_limit[env_map.get_waypoint(self.vehicle.get_location()).lane_id]
            if self.autopilot_flag:
                drive_status = "自动驾驶"
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 4 or event.button == 5:
                            self.autopilot_flag = False
                            message = "play"
                            self.data_recorder.record_mode_switch()
                            self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port)) 
                # 瞬时速度
                if self.instantaneous_speed:
                    if get_speed(self.vehicle) < self.speed_limit:
                        set_speed(self.vehicle, self.speed_limit)

                # 判断变道
                if scene_status == "干扰场景一":
                    now_right_left_lane_info = get_now_road_car(self.vehicle, now_lane_flag=True)
                    now_lane_next_car_info = now_right_left_lane_info.get("now_lane").get("next_info")  # 前车信息
                    if now_lane_next_car_info:  # 如果有前车
                        if now_lane_next_car_info[0][1] < self.speed_limit:  # 如果满足条件
                            if directions:
                                right_left_lane(self.vehicle, direction=directions.pop(0))  # 这样不加这个就是自动识别变道
                            else:
                                right_left_lane(self.vehicle)  # 这样不加这个就是自动识别变道
                    else:
                        directions.clear()
                        # print("没有前车了")
                        pass
                elif scene_status == "简单场景":
                    if not pygame.mixer.music.get_busy():
                        # print("正在播放")
                        pass
                    
                    now_right_left_lane_info = get_now_road_car(self.vehicle, now_lane_flag=True)
                    now_lane_next_car_info = now_right_left_lane_info.get("now_lane").get("next_info")  # 前车信息
                    if now_lane_next_car_info:
                        if  not takeover_prompted:  # 检查是否已经提示过接管
                            print("请接管！！！！！！！！！！！！")
                            message = "tor"
                            self.data_recorder.record_tor()
                            self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
                            takeover_prompted = True
                # 获取前方道路
                waypoint = env_map.get_waypoint(self.vehicle.get_location()).next(
                    max(1, int(get_speed(self.vehicle) / 3)))
                if waypoint:
                    waypoint = waypoint[0]
                else:
                    print(f"前方没有路了,当前车子坐标{self.vehicle.get_location()},车子对象为{self.vehicle}")
                    self.vehicle.destory()
                    return

                result = pid.run_step(self.speed_limit, waypoint)
                if get_speed(self.vehicle) < self.speed_limit - 10:
                    set_speed(self.vehicle, self.speed_limit)
                self.vehicle.apply_control(result)
                sleep(0.001)
            else:
                drive_status = "人工驾驶"
                self.autopilot_flag = keyboard.is_pressed("e")

                # 人工控制车辆
                steer, throttle, brake = get_steering_wheel_info_modified()

                # if get_speed(self.vehicle) > self.speed_limit:  # 设置最高速度
                #     throttle = 0.5
                # if get_speed(self.vehicle) < 80:  # 设置最低速度
                #    set_speed(self.vehicle,80)
                car_control(vehicle, steer, throttle, brake)
                sleep(0.002)

    def stop_vehicle(self):
        self.data_recorder.close()
        for _ in range(10):
            set_speed(self.vehicle, 0)
            sleep(0.001)



# 副车控制器
class Vice_Control:
    def __init__(self, main_car):
        self.main_car = main_car  # 主车

        self.thread_car_number = 10  # 一个线程控制的车辆数
        self.whichOne = 0  # 用于记录遍历副车的索引
        self.thread_cut_speed = threading.Thread()  # 减速线程
        self.flag = True

    def follow_road(self):
        global vices_car_list
        self.flag = True
        with ThreadPoolExecutor(max_workers=len(vices_car_list) // 10 + 1) as executor:
            while self.flag:
                if not vices_car_list:  # 如果没有车流
                    continue
                else:
                    if self.whichOne + self.thread_car_number > len(vices_car_list):
                        executor.submit(self.control_car, vices_car_list[self.whichOne:])
                        self.whichOne = 0
                    else:
                        executor.submit(self.control_car,
                                        vices_car_list[self.whichOne: self.whichOne + self.thread_car_number])
                        self.whichOne += self.thread_car_number
                sleep(0.01)

    def control_car(self, cars):
        global vices_car_list, scene_status
        for car in cars:
            if scene_status == "简单场景" or scene_status == "干扰场景一":
                if not self.thread_cut_speed.is_alive():  # 如果没有车减速
                    qian_road_car_info = get_now_road_car(self.main_car, now_lane_flag=True).get("now_lane").get("next_info")
                    if qian_road_car_info:
                        for i, car_info in enumerate(qian_road_car_info):  
                            next_car = car_info[0]  # 获取前车对象
                            if i == 0:
                                vices_car_list = [car for car in vices_car_list if car.id != next_car.id]
                            if next_car:
                                self.thread_cut_speed = threading.Thread(target=self.brake_throttle_retard,
                                                                        args=(next_car, -8.5, 0, 0, i == 0))
                                self.thread_cut_speed.start()
                                
            pid = VehiclePIDController(car, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
            # 获取前方道路
            waypoint = env_map.get_waypoint(car.get_location()).next(max(1, int(get_speed(car) / 6)))
            if waypoint:
                waypoint = waypoint[0]
            else:
                print(f"前方没有路了,当前车子坐标{car.get_location()},车子对象为{car}")
                car.destroy()

            speed_limit = road_speed_limit[env_map.get_waypoint(car.get_location()).lane_id]  # 获取车道的速度限制
            result = pid.run_step(speed_limit, waypoint)
            if get_speed(car) < speed_limit - 20:
                set_speed(car, speed_limit)
            car.apply_control(result)

    def brake_throttle_retard(self,vehicle, acceleration, target_speed, delay=0, show_brake_lights=False):
        """
        加减速
        :param vehicle: 目标车辆
        :param acceleration: 加速度
        :param target_speed: 目标速度
        :param delay: 延迟
        :param show_brake_lights: 是否显示刹车灯
        :return:
        """
        pid = VehiclePIDController(vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        t = time.time()
        time.sleep(0.1)
        speed = get_speed(vehicle)
        while time.time() - t < delay:
            # 获取前方道路
            waypoint = env_map.get_waypoint(vehicle.get_location()).next(max(1, int(get_speed(vehicle) / 6)))
            if waypoint:
                waypoint = waypoint[0]
            else:
                print(f"前方没有路了,当前车子坐标{vehicle.get_location()},车子对象为{vehicle}")
                return
            result = pid.run_step(speed, waypoint)
            vehicle.apply_control(result)
            time.sleep(0.01)
        if acceleration < 0:
            if show_brake_lights:
                set_brake_lights(vehicle)
        t = time.time()
        speed = get_speed(vehicle)
        while abs(get_speed(vehicle) - target_speed) > 1:
            # 获取前方道路
            waypoint = env_map.get_waypoint(vehicle.get_location()).next(
                max(1, int(get_speed(vehicle) / 6)))
            if waypoint:
                waypoint = waypoint[0]
            else:
                print(f"前方没有路了,当前车子坐标{vehicle.get_location()},车子对象为{vehicle}")
                return
            result = pid.run_step(0, waypoint)
            result.brake = 1
            result.throttle = 0
            vehicle.apply_control(result)

            sp = speed + acceleration * (time.time() - t) * 3.6
            sp = (max(0, sp))
            set_speed(vehicle, sp)
            time.sleep(0.01)
        for _ in range(10):
            set_speed(vehicle, target_speed)
            time.sleep(0.01)



def right_left_lane(main_car, direction=None, min_direction=10, method="pid"):
    """
    左转或右转
    :param direction: 左转还是右转,接收"left"和"right"
    :param min_direction: 最小变道距离
    :param method: 变道所使用的方法，默认用pid，还有agent
    :return:
    """
    # 判断有没有可变道路,得到direction方向值
    if not direction:  # 如果没有传左/右变道
        # 先判断左右是否有道路
        direction = str(env_map.get_waypoint(main_car.get_location()).lane_change).lower()
        if not direction:  # 如果没有可变道路，找前方看有没有可以变道
            direction = str(
                env_map.get_waypoint(main_car.get_location()).next(max(get_speed(main_car), 5))[0].lane_change).lower()
            if not direction:
                print("没有可变道路！！！！！！！！")
                return
            elif direction == "both":
                direction = random.choice(["right", "left"])
        elif direction == "both":
            direction = random.choice(["right", "left"])

    # PID
    pid = VehiclePIDController(main_car, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
    # 获取当前速度
    speed = get_speed(main_car)
    # 设置速度防止车子在已有速度上突然减速
    set_speed(main_car, speed)
    # 变道距离,根据速度实现
    distance = max(speed + 20, min_direction)

    while True:  # 获取到变道后的终点坐标
        if direction == "right":
            waypoint = env_map.get_waypoint(main_car.get_location()).get_right_lane()
        else:
            waypoint = env_map.get_waypoint(main_car.get_location()).get_left_lane()
        if not waypoint:
            sleep(0.01)
            continue
        waypoint = waypoint.next(distance)[0]
        break

    end_location = waypoint.transform.location

    # draw_line(world, locations=[main_car.get_location(), end_location], life_time=2)  # 划线
    while True:
        speed_limit = road_speed_limit[env_map.get_waypoint(main_car.get_location()).lane_id]

        if main_car.get_location().distance(end_location) < 0.5:
            now_time = time.time()
            while time.time() - now_time < 1:  # 变道完成后再执行一秒往前开
                waypoint = env_map.get_waypoint(main_car.get_location()).next(int(get_speed(main_car)))[0]
                result = pid.run_step(speed_limit, waypoint)
                main_car.apply_control(result)
                sleep(0.01)
            print("变道完毕")
            return True
        result = pid.run_step(speed_limit, waypoint)
        main_car.apply_control(result)
        sleep(0.01)


def draw_line(location1=None, location2=None, locations=None, thickness=0.1, life_time=10,
              color=carla.Color(255, 0, 0)):
    """
    用直线连接两个点或者多个点
    :param location1: carla坐标点1
    :param location2: carla坐标点2
    :param locations: 坐标点列表，如果传入坐标点列表，前面两个点失效,carla坐标点列表
    :param thickness: 亮度
    :param life_time: 存活时间
    :param color: 颜色
    :return: 没有返回，绘制出连线点
    """
    if locations:
        for index, location in enumerate(locations[1:]):
            world.debug.draw_line(locations[index], location, thickness=thickness, color=color, life_time=life_time)
        return
    world.debug.draw_line(location1, location2, thickness=thickness, color=color, life_time=life_time)


# 设置刹车灯
def set_brake_lights(vehicle):
    lights = carla.VehicleLightState.All
    # # 设置车辆的灯光状态
    vehicle.set_light_state(carla.VehicleLightState(lights))



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
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 5760, 1080  # 屏幕大小
        self.collision_detected = False  # 添加此行来追踪碰撞状态

        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360  # 屏幕大小
        self.screen = None  # 初始化屏幕窗口
        self.fonts = {} 
        pygame.init()  # 初始化pygame

        # 初始化窗口设置
        self.clock = pygame.time.Clock()
        self.size = 18  # 字体大小
        self.fps = 60  # 帧率
        self.font = pygame.font.Font(r"TTF\宋体.ttf", self.size)  # 初始化字体对象

        # 初始化传感器
        self.blueprint_camera = blueprint_library.find('sensor.camera.rgb')  # 选择一个传感器蓝图
        self.blueprint_camera.set_attribute('image_size_x', f'{self.SCREEN_WIDTH}')  # 传感器获得的图片高度
        self.blueprint_camera.set_attribute('image_size_y', f'{self.SCREEN_HEIGHT}')  # 传感器获得的图片宽度
        self.blueprint_camera.set_attribute('fov', '145')  # 水平方向上能看到的视角度数
        spawn_point = carla.Transform(carla.Location(x=0.15, y = -0.21, z=1.15), carla.Rotation(pitch=7, yaw=0, roll=0))  # 传感器相对车子的位置设置
        self.sensor = self.world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)  # 添加传感器

        threading.Thread(target=self.show_screen).start()

    def show_screen(self):  # 显示窗口
        # 初始化窗口
        pygame.display.set_caption("pygame模拟场景")

        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT),
                                              pygame.RESIZABLE, 32)
        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:  # 窗口关闭
                    pygame.quit()
                    os._exit(0)
                elif event.type == VIDEORESIZE:
                    self.CreateWindow(event.w, event.h)
            pygame.display.update()
            self.clock.tick(self.fps)

    # 加载图片
    def process_img(self, image):
        global drive_status, left_right_qian_distance, left_right_hou_distance
        i = np.array(image.raw_data)
        i2 = i.reshape((self.SCREEN_HEIGHT, self.SCREEN_WIDTH, 4))
        i3 = i2[:, :, :3]
        i3 = np.rot90(i3, k=1, axes=(0, 1))
        i3 = i3[..., ::-1]
        img_surface = pygame.surfarray.make_surface(np.flip(i3, axis=0))
        self.screen.blit(img_surface, (0, 0))  # 绘制图片

        # 添加文字信息
        self.draw_text(f"自车速度：{round(get_speed(self.vehicle), 2)}km/h", 20, (0, 0))

        list_of_cars_ahead = get_now_road_car(self.vehicle, now_lane_flag=True)  # 获取到当前道路的车
        next_car_info = list_of_cars_ahead.get("now_lane").get("next_info")  # 获取到前车信息
        if next_car_info:
            self.draw_text(f"前车速度：{round(get_speed(next_car_info[0][0]), 2)}km/h", 20, (0, 20))
            self.draw_text(f"与前车距离：{round(next_car_info[0][1], 2)}m", 20, (0, 40))
        self.draw_text(f"场景状态：{scene_status}", 20, (0, 60))
        self.draw_text(f"状态：{drive_status}", 20, (0, 80))
        fps = self.clock.get_fps()
        self.draw_text(f"当前帧率：{int(fps)}", 20, (0, 100))
        if self.collision_detected:
            self.draw_text("Collision!", 160, (self.SCREEN_WIDTH // 2-200, self.SCREEN_HEIGHT // 2), bold=True,color=(255, 0, 0))
            self.collision_detected = False  # 重置碰撞状态
        # 刷新屏幕
        pygame.display.flip()

    def draw_text(self, word, size, position, bold=False, color=(255, 0, 0)):
        # 检查是否已经有缓存的字体，否则创建新的
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


def smooth_steer(steer_input):
    global last_steer
    alpha = 0.9  # 平滑系数，调整此值以改变平滑程度
    smoothed_steer = alpha * steer_input + (1 - alpha) * last_steer
    last_steer = smoothed_steer
    return smoothed_steer

def car_control(vehicle, steer=0, throttle=1, brake=0):
    """
    控制车辆
    :param vehicle:  车子对象
    :param throttle: 油门
    :param steer: 转角
    :param brake: 刹车
    :return:
    """
    # 保留三位小数，防止车辆不被控制，可能控制器只能接受三位小数
    steer = smooth_steer(steer)
    steer = round(steer, 3)
    throttle = round(throttle, 3)
    brake = round(brake, 3)
    control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
    vehicle.apply_control(control)


def is_vehicle_in_front(target_vehicle, reference_vehicle):
    """
    返回是在车子前方还是后方
    :param target_vehicle: 目标车
    :param reference_vehicle: 主车，这个是主车
    :return: 一个车辆列表，每个索引值是一个元组，包含车辆对象，前车or后车，距离
    """
    target_location = target_vehicle.get_location()
    target_forward = target_vehicle.get_transform().get_forward_vector()

    reference_location = reference_vehicle.get_location()

    # 计算从参考车辆指向目标车辆的向量
    vector_to_target = carla.Location(target_location.x - reference_location.x,
                                      target_location.y - reference_location.y,
                                      target_location.z - reference_location.z)

    # 计算向量夹角（使用点积）
    dot_product = target_forward.x * vector_to_target.x + target_forward.y * vector_to_target.y
    magnitude_product = math.sqrt(target_forward.x ** 2 + target_forward.y ** 2) * math.sqrt(
        vector_to_target.x ** 2 + vector_to_target.y ** 2)
    if magnitude_product == 0:
        return True
    angle = math.acos(dot_product / magnitude_product) * (180 / math.pi)

    # 一般情况下，如果夹角小于90度，则目标车辆在主车辆的前方
    return angle < 90

def create_vices(vehicle_traffic, vehicle):
    """
    创建车流
    :param vehicle_traffic: 创建车子的对象
    :param vehicle: 主车
    :return: 返回车流的车辆列表
    """
    vice_locations = []  # 副车的坐标列表n
    vehicle_location = vehicle.get_location()  # 主车坐标
    number = 5 # 每个方向生成的车辆数
    max_offset = 5  # 最大偏移量
    ahead_distance = 40
    back_distance = 30
    # 创建前方的车流
    for i in range(number):
        # 为每排车辆随机生成一个偏移量
        offset_center = random.randint(-max_offset, max_offset)
        offset_right1 = random.randint(-max_offset, max_offset)
        offset_right2 = random.randint(-max_offset, max_offset)
        offset_left1 = random.randint(-max_offset, max_offset)
        offset_left2 = random.randint(-max_offset, max_offset)

        # 中前
        location = env_map.get_waypoint(vehicle_location).next((i + 1) * ahead_distance + offset_center)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 右一前
        location = env_map.get_waypoint(vehicle_location).get_right_lane().next((i + 1) * ahead_distance + offset_right1)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 右二前
        location = env_map.get_waypoint(vehicle_location).get_right_lane().get_right_lane().next((i + 1) * ahead_distance + offset_right2)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 左一前
        location = env_map.get_waypoint(vehicle_location).get_left_lane().next((i + 1) * ahead_distance + offset_left1)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 左二前
        location = env_map.get_waypoint(vehicle_location).get_left_lane().get_left_lane().next((i + 1) * ahead_distance + offset_left2)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))


    # 创建后方的车流，只有一排
    for i in range(1):  # 只循环一次
        # 随机生成后方的偏移量
        offset_center = random.randint(-max_offset, max_offset)
        offset_right1 = random.randint(-max_offset, max_offset)
        offset_right2 = random.randint(-max_offset, max_offset)
        offset_left1 = random.randint(-max_offset, max_offset)
        offset_left2 = random.randint(-max_offset, max_offset)

        # 中后
        location = env_map.get_waypoint(vehicle_location).previous((i + 1) * back_distance + offset_center)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 右一后
        location = env_map.get_waypoint(vehicle_location).get_right_lane().previous((i + 1) * back_distance + offset_right1)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 右二后
        location = env_map.get_waypoint(vehicle_location).get_right_lane().get_right_lane().previous((i + 1) * back_distance + offset_right2)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 左一后
        location = env_map.get_waypoint(vehicle_location).get_left_lane().previous((i + 1) * back_distance + offset_left1)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

        # 左二后
        location = env_map.get_waypoint(vehicle_location).get_left_lane().get_left_lane().previous((i + 1) * back_distance + offset_left2)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))

    return vehicle_traffic.create_vehicle(vice_locations)



# 获取当前车道的车辆
def get_now_road_car(vehicle, now_lane_flag=False, left_lane_flag=False, right_lane_flag=False):
    """

    :param vehicle: 主车对象
    :param now_lane_flag: 是否获取当前车道
    :param left_lane_flag: 是否获取左边车道
    :param right_lane_flag: 是否获取右边车道
    :return: 一个字典，根据距离排好序的车辆
    """

    # 定义一个函数用于排序
    def sort_by_distance(item):
        return item[1]

    def distance_between_vehicles(vehicle, vehicle2):
        return vehicle.get_location().distance(vehicle2.get_location())

    def is_vehicle_in_front(target_vehicle, reference_vehicle):
        """
        返回是在车子前方还是后方
        :param target_vehicle: 目标车
        :param reference_vehicle: 主车，这个是主车
        :return: 一个车辆列表，每个索引值是一个元组，包含车辆对象，前车or后车，距离,  （前左右）
        """
        target_location = target_vehicle.get_location()
        target_forward = target_vehicle.get_transform().get_forward_vector()

        reference_location = reference_vehicle.get_location()

        # 计算从参考车辆指向目标车辆的向量
        vector_to_target = carla.Location(target_location.x - reference_location.x,
                                          target_location.y - reference_location.y,
                                          target_location.z - reference_location.z)

        # 计算向量夹角（使用点积）
        dot_product = target_forward.x * vector_to_target.x + target_forward.y * vector_to_target.y
        magnitude_product = math.sqrt(target_forward.x ** 2 + target_forward.y ** 2) * math.sqrt(
            vector_to_target.x ** 2 + vector_to_target.y ** 2)
        angle = math.acos(dot_product / magnitude_product) * (180 / math.pi)

        # 一般情况下，如果夹角小于90度，则目标车辆在主车辆的前方
        return angle < 90

    # 获取所有车辆对象
    vice_cars = list(world.get_actors().filter("vehicle.*"))
    vice_cars = [v for v in vice_cars if v.id != vehicle.id]  # 排除自车的所有车辆
    need_car_list = {}  # 需要的车辆列表
    if now_lane_flag:
        need_car_list["now_lane"] = [v for v in vice_cars if
                                     env_map.get_waypoint(v.get_location()).lane_id == env_map.get_waypoint(
                                         vehicle.get_location()).lane_id]  # 当前车道all车辆
    if left_lane_flag:
        need_car_list["left_lane"] = [v for v in vice_cars if
                                      env_map.get_waypoint(v.get_location()).lane_id == env_map.get_waypoint(
                                          vehicle.get_location()).get_left_lane().lane_id]  # 取左all车辆
    if right_lane_flag:
        need_car_list["right_lane"] = [v for v in vice_cars if
                                       env_map.get_waypoint(v.get_location()).lane_id == env_map.get_waypoint(
                                           vehicle.get_location()).get_right_lane().lane_id]  # 取右all车辆

    dict_0 = {
        "now_lane": {
            "next_info": [],
            "previous_info": []
        },
        "left_lane": {
            "next_info": [],
            "previous_info": []
        },
        "right_lane": {
            "next_info": [],
            "previous_info": []
        }
    }
    for i in need_car_list:
        for car in need_car_list.get(i):
            if is_vehicle_in_front(car, vehicle):
                dict_0[i]["next_info"].append((car, distance_between_vehicles(vehicle, car)))
            else:
                dict_0[i]["previous_info"].append((car, distance_between_vehicles(vehicle, car)))
    # 遍历字典中的每个键
    for lane, data in dict_0.items():
        # 对于每个键下的 "next" 和 "previous" 列表，都按照元组的第一个元素进行排序
        for direction in ["next_info", "previous_info"]:
            if dict_0[lane][direction]:
                dict_0[lane][direction] = sorted(dict_0[lane][direction], key=lambda x: x[1])
    return dict_0


# 销毁所有车辆
def destroy_all_vehicles_traffics(world, vehicle_flag=True, traffic_flag=True):
    """
    销毁所有车辆和交通标志
    """
    actors = []
    if vehicle_flag:
        actors += list(world.get_actors().filter('*vehicle*'))
    if traffic_flag:
        actors += list(world.get_actors().filter("*prop*"))
    # 销毁每个车辆
    for actor in actors:
        actor.destroy()


# 确保车辆坐标
def check_coordinate(vehicles):
    start_time = time.time()
    if not isinstance(vehicles, list):
        while True:
            if time.time() - start_time > 1:
                return False
            start_location = vehicles.get_location()
            if start_location.x != 0 and start_location.y != 0:
                break
            sleep(0.01)
        return True

    flags = []
    for vehicle in vehicles:
        start_time = time.time()
        while True:
            start_location = vehicle.get_location()
            if start_location.x != 0 and start_location.y != 0:
                break
            if time.time() - start_time > 1:
                flags.append(False)
                continue
            sleep(0.01)
        flags.append(True)
    return flags


# 设置车子速度
def set_speed(vehicle, speed_kmh):
    """
    设置车子速度
    :param vehicle: 车子对象
    :param speed_kmh: 车子速度
    :return:
    """
    speed = speed_kmh / 3.6
    # 获取车辆的当前速度方向
    forward_vector = vehicle.get_transform().rotation.get_forward_vector()
    # 设置车子速度
    vehicle.set_target_velocity(
        carla.Vector3D(forward_vector.x * speed, forward_vector.y * speed, forward_vector.z * speed))


# 获取方向盘信息
def get_steering_wheel_info_modified():

    def non_linear_steering(x):
        p = 1.5
        return np.sign(x) * np.abs(x)**p
    
    steering = joystick.get_axis(0)
    throttle = joystick.get_axis(1)
    brake = joystick.get_axis(2)
    adjusted_steering = non_linear_steering(steering)
    adjusted_throttle = (-throttle + 1) / 2
    adjusted_brake = (-brake + 1) / 2

    return adjusted_steering, adjusted_throttle, adjusted_brake

def get_steering_wheel_info():
    steering = joystick.get_axis(0)
    throttle = joystick.get_axis(1)
    brake = joystick.get_axis(2)
    adjusted_throttle = (-throttle + 1) / 2
    adjusted_brake = (-brake + 1) / 2
    return steering, adjusted_throttle, adjusted_brake

def destroy_lose_vehicle(main_car):  
    global vices_car_list

    def destroy():
        while True:
            all_vehicle = list(world.get_actors().filter('*vehicle*'))
            all_vehicle = [car for car in all_vehicle if car.id != main_car.id]  # 除去自车

            vices_car_ids = [car.id for car in vices_car_list]  # 有控制的副车列表id
            lose_vices = [car for car in all_vehicle if car.id not in vices_car_ids]  # 失去控制的车辆列表
            # print(lose_vices)
            for car in lose_vices:
                distance = main_car.get_location().distance(car.get_location())
                # print(f"{lose_vices},距离{distance}")
                if distance < 10:
                    # if not thread_up_speed.is_alive():
                    #     thread_up_speed = threading.Thread(target=brake_throttle_retard, args=(lose_vices[0], 10,))
                    #     thread_up_speed.start()
                    sleep(1)
                    car.destroy()
            sleep(0.1)

    threading.Thread(target=destroy).start()


def destroy_vice(vehicle):  # 销毁除自车以外的所有车辆
    actors = list(world.get_actors().filter('*vehicle*'))
    actors = [car for car in actors if car.id != vehicle.id]
    for car in actors:
        car.destroy()


def draw_arrow(locations, height=-1):
    debug = world.debug
    for location in locations:
        arrow_location = location + carla.Location(z=height)  # 假设箭头位置略高于地面

        # 绘制箭头
        debug.draw_arrow(arrow_location, env_map.get_waypoint(arrow_location).next(10)[0].transform.location,
                         thickness=0.3, arrow_size=0.5, color=carla.Color(255, 0, 0))


def scene_jian(vehicle, main_car_control, vice_car_control, end_location):  # 简单场景
    global scene_status, vices_car_list
    vices_car_list += create_vices(vehicle_traffic, vehicle)  # 创建副车

    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    threading.Thread(target=vice_car_control.follow_road).start()  # 启动副车

    scene_status = "等待36s开始"  # 36s
    t = time.time()
    time_gap = 36
    while time.time() - t < time_gap:
        scene_status = f"倒计时{int(time_gap - (time.time() - t))}s (简单场景)"
        # print(f"经历了{int(time.time() - t)}s了")
        sleep(1)
    scene_status = "简单场景"
    while vehicle.get_location().distance(end_location) > 30:  # 如果离终点小于十米就认为场景一结束
        sleep(1)  # 每间隔一秒判断是否到没有到终点
    main_car_control.flag = False  # 停止主车控制
    main_car_control.stop_vehicle()  # 停止主车运行
    vices_car_list.clear()
    destroy_vice(vehicle)  # 销毁除自车以外的所有车辆
    main_car_control.autopilot_flag = True
    print("到达终点")


def scene_gan(vehicle, main_car_control, vice_car_control, end_location):  # 干扰场景一
    global scene_status, vices_car_list
    scene_status = "干扰场景一"
    vices_car_list += create_vices(vehicle_traffic, vehicle)  # 创建副车
    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    threading.Thread(target=vice_car_control.follow_road).start()  # 启动副车

    while vehicle.get_location().distance(end_location) > 30:  # 如果离终点小于十米就认为干扰场景一
        sleep(1)  # 每间隔一秒判断是否到没有到终点
    main_car_control.flag = False  # 停止主车控制
    vice_car_control.flag = False  # 停止副车控制
    main_car_control.stop_vehicle()  # 停止主车运行
    vices_car_list.clear()
    destroy_vice(vehicle)  # 销毁除自车以外的所有车辆
    main_car_control.autopilot_flag = True
    print("到达终点")


def scene_gan2(vehicle, main_car_control, vice_car_control, end_location):  # 干扰场景二
    global scene_status, vices_car_list
    scene_status = "干扰场景二"
    vices_car_list += create_vices(vehicle_traffic, vehicle)  # 创建副车
    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    threading.Thread(target=vice_car_control.follow_road).start()  # 启动副车

    while vehicle.get_location().distance(end_location) > 30:  # 如果离终点小于十米就认为干扰场景一
        sleep(1)  # 每间隔一秒判断是否到没有到终点
    main_car_control.flag = False  # 停止主车控制
    vice_car_control.flag = False  # 停止副车控制
    main_car_control.stop_vehicle()  # 停止主车运行
    vices_car_list.clear()
    destroy_vice(vehicle)  # 销毁除自车以外的所有车辆
    main_car_control.autopilot_flag = True
    print("到达终点")


def interim(vehicle, main_car_control, end_location):  # 转弯过渡
    global scene_status
    scene_status = "转弯过渡"
    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    while vehicle.get_location().distance(end_location) > 30:  # 如果离终点小于十米就认为干扰场景一
        sleep(1)  # 每间隔一秒判断是否到没有到终点
    main_car_control.flag = False  # 停止主车控制
    main_car_control.stop_vehicle()  # 停止主车运行
    main_car_control.autopilot_flag = True

def attach_collision_sensor(vehicle, world, window,data_recorder):
    collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    collision_sensor = world.spawn_actor(collision_sensor_bp, collision_sensor_transform, attach_to=vehicle)
    collision_sensor.listen(lambda event: handle_collision(event, window, data_recorder))
    return collision_sensor

def handle_collision(event, window,data_recorder):
    print(f"Collision detected with {event.other_actor.type_id} at frame {event.frame}")
    data_recorder.record_collision()
    window.collision_detected = True  # 设置窗口类中的碰撞标志

if __name__ == '__main__':
    destroy_all_vehicles_traffics(world)  # 销毁所有车辆
    data_recorder = DataRecorder()

    draw_arrow([easy_location8, easy_location1, interfere_one_location1, interfere_two_location1, easy_location2,
                interfere_one_location2,
                end_location1, easy_location3, interfere_two_location2, end_location2, easy_location4, easy_location5,
                interfere_two_location3,
                easy_location6, interfere_two_location4, interfere_one_location3, end_location3, easy_location7,
                interfere_two_location5,
                end_location4])  # 划线
    vehicle_traffic = Vehicle_Traffic(world)  # 车辆创建对象
    vehicle = vehicle_traffic.create_vehicle([easy_location1], vehicle_model="vehicle.lincoln.mkz_2020")[0]  # 创建主车

    physics_control = vehicle.get_physics_control() # 修改车辆控制参数
    steering_curve = [
        carla.Vector2D(x=0.0, y=1.0),
        carla.Vector2D(x=20.0, y=0.9),
        carla.Vector2D(x=80.0, y=0.8),
        carla.Vector2D(x=120.0, y=0.2)
    ]
    physics_control.steering_curve = steering_curve
    vehicle.apply_physics_control(physics_control)

    destroy_lose_vehicle(vehicle)  # 销毁失控车辆线程启动
    window = Window(world, blueprint_library, vehicle)  # 创建窗口
    collision_sensor = attach_collision_sensor(vehicle, world, window, data_recorder)
    main_car_control = Main_Car_Control(vehicle,world,data_recorder, True)  # 主车控制类
    vice_car_control = Vice_Control(vehicle)  # 副车控制类

    # 简单场景一
    scene_jian(vehicle, main_car_control, vice_car_control, interfere_one_location1)

    # 简单场景
    while True:
        sleep(1)
