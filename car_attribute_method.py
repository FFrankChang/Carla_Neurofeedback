import functools
from disposition import *
import os
import threading
from time import sleep
import time
import keyboard
import numpy as np
import random
import math
from concurrent.futures import ThreadPoolExecutor
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.controller import VehiclePIDController
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.tools.misc import get_speed


def destroy_all_vehicles_traffics(vehicle_flag=True, traffic_flag=True):
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


def set_brake_lights(vehicle):
    lights = carla.VehicleLightState.Brake
    # # 设置车辆的灯光状态
    vehicle.set_light_state(carla.VehicleLightState(lights))


# 页面窗口
class Window:
    def __init__(self, vehicle):
        """
        创建车子的pygame窗口显示
        :param vehicle: 车子对象
        """
        self.vehicle = vehicle
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1600, 1000  # 屏幕大小
        self.screen = None  # 初始化屏幕窗口
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
        self.blueprint_camera.set_attribute('fov', '110')  # 水平方向上能看到的视角度数
        spawn_point = carla.Transform(carla.Location(x=-0.5,y=-0.3, z=1.2),
                                      carla.Rotation(pitch=0, yaw=0, roll=0))  # 传感器相对车子的位置设置
        self.sensor = world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)  # 添加传感器

        threading.Thread(target=self.show_screen).start()

    def show_screen(self):  # 显示窗口
        # 初始化窗口
        pygame.display.set_caption("pygame模拟场景")
        # 去掉pygame.RESIZABLE表示窗口不可变
        # self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), pygame.HWSURFACE |
        #                                       pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), pygame.HWSURFACE |
                                              pygame.DOUBLEBUF, 32)
        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:  # 窗口关闭
                    pygame.quit()
                    os._exit(0)
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
        self.draw_text(f"速度：{round(get_speed(self.vehicle), 2)}km/h", 20, (0, 0))

        cars = get_now_road_car(self.vehicle, next_vehicle=True)
        if cars:
            car = cars[0][0]
            self.draw_text(f"前车速度：{round(get_speed(car), 2)}km/h", 20, (0, 20))
            self.draw_text(f"与前车距离：{round(car.get_location().distance(self.vehicle.get_location()), 2)}m", 20,
                           (0, 40))

        # 刷新屏幕
        pygame.display.flip()

    def draw_text(self, word, length, position, color=(255, 0, 0)):
        text = self.font.render(word.ljust(length), True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.screen.blit(text, text_rect)


# 根据坐标点生成车辆
class Vehicle_Traffic:
    def __init__(self, ):
        pass

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
            blueprint_car = blueprint_library.filter('*vehicle*')
            # 筛选出指定车型
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            # 获取车辆的蓝图
            cars = blueprint_library.filter('*vehicle*')
        # 如果没有传入生成点，随机位置生成
        if not points:
            # 获取地图可创建的坐标点
            spawn_points = env_map.get_spawn_points()
            for index in range(number):
                # 随机位置生成车辆
                vehicle = world.try_spawn_actor(random.choice(cars), random.choice(spawn_points))
                if vehicle:
                    vehicles.append(vehicle)
                else:
                    print(f"第{index + 1}辆车未成功生成！")
            check_coordinate(vehicles)
            return vehicles
        # 创建指定数量的车
        for index, point in enumerate(points):
            waypoint = env_map.get_waypoint(point)  # 获取车辆生成点附近的道路信息
            # 设置生成点和初始旋转角度
            transform = carla.Transform(point, waypoint.transform.rotation)
            # 生成车辆
            vehicle = world.try_spawn_actor(random.choice(cars), transform)
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
            traffic_cone_bp = blueprint_library.find(traffic_model)
            # 创建交通标志
            traffic = world.try_spawn_actor(traffic_cone_bp, carla.Transform(location, carla.Rotation()))
            if traffic:
                traffics.append(traffic)
            else:
                print(f"{location}交通标志未生成成功！")
        return traffics


# 主车控制
class Vehicle_Control:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.speed_limit = 30
        self.args_lateral_dict = {'K_P': 1.95, 'K_D': 0.2, 'K_I': 0.07, 'dt': 1.0 / 10.0}
        self.args_long_dict = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 10.0}
        self.flag = True  # 是否继续控制车辆的代码

    def right_left_lane(self, direction=None, min_direction=10, method="pid"):
        """
        左转或右转
        :param direction: 左转还是右转,接收"left"和"right"
        :param min_direction: 最小变道距离
        :param method: 变道所使用的方法，默认用pid，还有agent
        :return:
        """
        self.flag = True
        # 判断有没有可变道路,得到direction方向值
        if not direction:  # 如果没有传左/右变道
            # 先判断左右是否有道路
            direction = str(env_map.get_waypoint(self.vehicle.get_location()).lane_change).lower()
            if not direction:  # 如果没有可变道路，找前方看有没有可以变道
                direction = str(
                    env_map.get_waypoint(self.vehicle.get_location()).next(max(get_speed(self.vehicle), 5))[
                        0].lane_change).lower()
                if not direction:
                    print("没有可变道路！！！！！！！！")
                    return
                elif direction == "both":
                    direction = random.choice(["right", "left"])
            elif direction == "both":
                direction = random.choice(["right", "left"])

        # PID
        pid = VehiclePIDController(self.vehicle, args_lateral=self.args_lateral_dict,
                                   args_longitudinal=self.args_long_dict)
        # 获取当前速度
        speed = get_speed(self.vehicle)
        # 设置速度防止车子在已有速度上突然减速
        set_speed(self.vehicle, speed)
        # 变道距离,根据速度实现
        distance = max(speed, min_direction)

        while True:  # 获取到变道后的终点坐标
            if direction == "right":
                waypoint = env_map.get_waypoint(self.vehicle.get_location()).get_right_lane()
            else:
                waypoint = env_map.get_waypoint(self.vehicle.get_location()).get_left_lane()
            if not waypoint:
                sleep(0.01)
                continue
            waypoint = waypoint.next(distance)[0]
            break

        end_location = waypoint.transform.location
        if method == "agent":
            self.autopilot_agent(end_location)
            return

        # draw_line(self.world, locations=[self.vehicle.get_location(), end_location], life_time=2)  # 划线
        while True and self.flag:
            if self.vehicle.get_location().distance(end_location) < 0.5:
                now_time = time.time()
                while time.time() - now_time < 1:  # 变道完成后再执行一秒往前开
                    waypoint = \
                        env_map.get_waypoint(self.vehicle.get_location()).next(int(get_speed(self.vehicle)))[0]
                    result = pid.run_step(self.speed_limit, waypoint)
                    self.vehicle.apply_control(result)
                    sleep(0.01)
                print("变道完毕")
                return True
            result = pid.run_step(self.speed_limit, waypoint)
            self.vehicle.apply_control(result)
            sleep(0.01)

    def autopilot_agent(self, end_location, ignore_traffic_light=True, mode="aggressive", distance=4.5, draw=True):
        """
        自动驾驶车辆到终点坐标
        :param end_location: 终点坐标
        :param ignore_traffic_light: 是否忽略交通规则
        :param mode: 模式，有三种，速度不一样  cautious(最慢)，normal(正常),aggressive(稍快)
        :param distance: 每个waypoint之间的距离
        :param draw: 是否绘制线路
        """
        self.flag = True
        agent = BehaviorAgent(self.vehicle, ignore_traffic_light=ignore_traffic_light, behavior=mode)
        agent.speed_limit = self.speed_limit + 5
        agent._sampling_resolution = distance

        path = agent.set_destination(self.vehicle.get_location(), end_location, clean=True)
        if draw:
            path = [i[0].transform.location for i in path]
            # draw_line(self.world, locations=path, life_time=60)

        while True and self.flag:
            agent.update_information(self.vehicle)
            # self.world.tick()  # 推进模拟时间，并更新世界中所有角色的状态
            if len(agent._local_planner.waypoints_queue) < 1:
                print('======== Success, Arrivied at Target Point!')
                return True
            # control = agent.run_step(debug=True)
            control = agent.run_step()
            self.vehicle.apply_control(control)
            sleep(0.01)

    def autopilot_pid(self, end_location):
        """
        前往目的地
        :param end_location: 终点坐标
        :return:
        """
        self.flag = True
        # PID
        pid = VehiclePIDController(self.vehicle, args_lateral=self.args_lateral_dict,
                                   args_longitudinal=self.args_long_dict)
        while True and self.flag:
            # 设置车子速度，防止他在原有基础速度上突然减速
            speed = get_speed(self.vehicle)
            set_speed(self.vehicle, speed)
            # 计算每个waypoint的距离，如果速度越快，相应waypoint距离增加，防止掉头
            sampling_resolution = max(int(get_speed(self.vehicle) // 10), 1)
            # 创建路由对象，方便获取路径
            dao = GlobalRoutePlannerDAO(env_map, sampling_resolution=4.5)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            # 获取前往终点的路径信息
            path = grp.trace_route(self.vehicle.get_location(), end_location)
            # 如果没有了，就认为到达目的地
            if len(path) < sampling_resolution + 2:
                print("目的地到达！！！！！！！！！！！！！")
                return
            # 省略前几个waypoint，防止速度过快往回跑
            waypoint = path[sampling_resolution + 1][0]
            draw_line(world, locations=[self.vehicle.get_location(), waypoint.transform.location], life_time=0.1)  # 划线
            result = pid.run_step(self.speed_limit, waypoint)
            self.vehicle.apply_control(result)
            sleep(0.01)

    def follow_lane(self, instantaneous_speed=False):
        """
        沿着当前道路走
        :return:
        """
        self.flag = True
        # PID
        pid = VehiclePIDController(self.vehicle, args_lateral=self.args_lateral_dict,
                                   args_longitudinal=self.args_long_dict)
        while True and self.flag:
            if instantaneous_speed:
                if get_speed(self.vehicle) < self.speed_limit - 5:
                    set_speed(self.vehicle, self.speed_limit)
            # 获取前方道路
            waypoint = env_map.get_waypoint(self.vehicle.get_location()).next(
                max(1, int(get_speed(self.vehicle) / 6)))
            if waypoint:
                waypoint = waypoint[0]
            else:
                print(f"前方没有路了,当前坐标车子坐标{self.vehicle.get_location()}")
                for i in range(5):
                    set_speed(self.vehicle, 0)
                    sleep(0.01)
                return False
            draw_line(world, locations=[self.vehicle.get_location(), waypoint.transform.location], life_time=0.1)  # 划线
            result = pid.run_step(self.speed_limit, waypoint)
            self.vehicle.apply_control(result)
            sleep(0.01)


# 控制车辆刹车
def brake_throttle_retard(vehicle, acceleration, target_speed, delay=None):
    """
    加减速
    :param vehicle: 目标车辆
    :param acceleration: 加速度
    :param target_speed: 目标速度
    :return:
    """
    speed = get_speed(vehicle)
    pid = VehiclePIDController(vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
    t = time.time()
    while abs(get_speed(vehicle) - target_speed) > 1:
        # 获取前方道路
        waypoint = env_map.get_waypoint(vehicle.get_location()).next(
            max(1, int(get_speed(vehicle) / 6)))
        if waypoint:
            waypoint = waypoint[0]
        else:
            print(f"前方没有路了,当前车子坐标{vehicle.get_location()},车子对象为{vehicle}")
            return
        result = pid.run_step(target_speed, waypoint)
        result.brake = 0
        result.throttle = 0
        sp = speed + acceleration * (time.time() - t) * 3.6
        sp = (max(0, sp))
        set_speed(vehicle, sp)
        vehicle.apply_control(result)
        sleep(0.01)
    for _ in range(10):
        set_speed(vehicle, target_speed)
        sleep(0.01)


# 控制车辆刹车
def brake_throttle_time(vehicle, acceleration):
    """
    加减速
    :param vehicle: 目标车辆
    :param acceleration: 加速度
    :return:
    """
    pid = VehiclePIDController(vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
    t = time.time()
    speed = get_speed(vehicle)
    while time.time() - t < 3:
        # 获取前方道路
        waypoint = env_map.get_waypoint(vehicle.get_location()).next(
            max(1, int(get_speed(vehicle) / 6)))
        if waypoint:
            waypoint = waypoint[0]
        else:
            print(f"前方没有路了,当前车子坐标{vehicle.get_location()},车子对象为{vehicle}")
            return
        result = pid.run_step(0, waypoint)
        result.brake = 0
        result.throttle = 0
        vehicle.apply_control(result)
        sp = speed + acceleration * (time.time() - t) * 3.6
        # print(sp)
        sp = (max(0, sp))
        set_speed(vehicle, sp)
        sleep(0.01)


def get_steering_wheel_info():
    """
    return: 方向盘、油门、刹车
    """
    # print(self.joystick.get_axis(0), (-self.joystick.get_axis(2) + 1) / 2, (-self.joystick.get_axis(3) + 1) / 2)
    return joystick.get_axis(0), (-joystick.get_axis(2) + 1) / 2, (-joystick.get_axis(3) + 1) / 2


# 获取当前车道的车辆，返回升序车辆列表
def get_now_road_car(vehicle, next_vehicle=False, previous_vehicle=False):
    """
    返回当前车道的车辆
    :param vehicle: 车子对象
    :param next_vehicle: 前方车辆，默认获取，置为True,只获取前车
    :param previous_vehicle: 后方车辆，默认获取，置为True,只获取后车
    :return:  返回以距离升序车辆列表，（车辆对象，前or后，距离）
    """

    def distance_between_vehicles(vehicle, vehicle2):
        return vehicle.get_location().distance(vehicle2.get_location())

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
        angle = math.acos(dot_product / magnitude_product) * (180 / math.pi)

        # 一般情况下，如果夹角小于90度，则目标车辆在主车辆的前方
        return angle < 90

    # 获取所有车辆对象
    vice_cars = list(world.get_actors().filter("vehicle.*"))
    vice_cars = [v for v in vice_cars if v.id != vehicle.id]  # 排除自车
    vice_cars = [v for v in vice_cars if env_map.get_waypoint(v.get_location()).lane_id == env_map.get_waypoint(
        vehicle.get_location()).lane_id]  # 只要当前车道车辆
    if not vice_cars:
        return None  # 该车道没有车

    distance_list = [(v, is_vehicle_in_front(v, vehicle), distance_between_vehicles(vehicle, v)) for v in vice_cars]
    sorted_list = sorted(distance_list, key=lambda x: x[2])

    # 筛选
    if next_vehicle:
        return [tup for tup in sorted_list if tup[1] is True]
    if previous_vehicle:
        return [tup for tup in sorted_list if tup[1] is False]
    return sorted_list


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


def load_map(xodr_path):
    """
    导入地图
    :param xodr_path :  xodr文件路径
    """
    with open(xodr_path, encoding="utf-8") as f:
        data = f.read()
        vertex_distance = 1
        max_road_length = 500
        wall_height = 0.5
        extra_width = 1
        world = client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=True,
                enable_mesh_visibility=True
            )
        )
    return world


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
    steer = round(steer, 3)
    throttle = round(throttle, 3)
    brake = round(brake, 3)
    control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
    vehicle.apply_control(control)
    sleep(0.01)


def draw_arrow(locations, height=0.5):
    debug = world.debug
    for location in locations:
        arrow_location = location + carla.Location(z=height)  # 假设箭头位置略高于地面

        # 绘制箭头
        debug.draw_arrow(arrow_location, env_map.get_waypoint(arrow_location).next(10)[0].transform.location,
                         thickness=0.3, arrow_size=0.5, color=carla.Color(255, 0, 0))


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


def draw_face(location, life_time=10.0, size=1, color=carla.Color(255, 0, 0)):
    """
    绘制一个面,方便看生成的位置在哪
    :param location: 绘制的坐标点
    :param life_time:存活时间 ，默认10s
    :param size:绘制的面大小
    :param color: 颜色
    :return: None
    """
    debug = world.debug
    debug.draw_point(location, size=size, color=color, life_time=life_time)


def get_vehicle_length(vehicle):
    """
    获取车子的长度
    :param vehicle: 车子对象
    :return: 返回车子的长宽高
    """
    car_length = vehicle.bounding_box.extent
    return car_length.x * 2, car_length.y * 2, car_length.z * 2


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
    vehicle.set_target_velocity(carla.Vector3D(forward_vector.x * speed, forward_vector.y * speed, forward_vector.z * speed))


def get_vehicle_steer(vehicle):
    """
    获取carla中车子方向盘值
    :param vehicle: 车子对象
    :return:
    """
    return vehicle.get_control().steer


def set_towards(vehicle):
    """
    单独设置车子的朝向
    :param vehicle: 车子对象
    :return:
    """
    transform = vehicle.get_transform()
    vehicle.set_transform(transform)  # 这个需要的是transform对象


def timed_function(interval, stop_event):
    """
    在函数上加上一下内容就可以实现定时器
    stop_event = threading.Event()  # 用于停止定时器，执行stop_event.set()就可以停止该定时器
    @timed_function(interval=0.01, stop_event=stop_event)
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # 定义定时器回调函数
            def timer_callback():
                func(*args, **kwargs)
                # 递归调用定时器，实现周期性执行
                if not stop_event.is_set():
                    threading.Timer(interval, timer_callback).start()

            # 启动定时器
            threading.Timer(interval, timer_callback).start()

        return wrapper

    return decorator
