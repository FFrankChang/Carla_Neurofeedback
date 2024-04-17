import threading
from disposition import *
from .tongji_config import *

vices_car_list = None  # 所有副车列表
drive_status = "自动驾驶"  # 驾驶状态
directions = ["left", "right", "left"]  # 变道顺序
scene_status = "干扰场景一"  # 干扰场景一   简单场景
left_right_qian_distance = 0
left_right_hou_distance = 0


def get_Vice():
    global vices_car_list
    if vices_car_list is None:
        return []
    return vices_car_list


def set_Vice(val):
    global vices_car_list
    vices_car_list = val
    return vices_car_list


# 根据坐标点生成车辆
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
            cars = self.blueprint_library.filter('*vehicle*')
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
    def __init__(self, main_car, instantaneous_speed=False):
        """
        主车控制类
        :param main_car: 主车对象
        :param instantaneous_speed: 是否瞬时速度
        """
        self.vehicle = main_car  # 主车对象
        self.instantaneous_speed = instantaneous_speed  # 是否瞬时到达目标速度
        self.scene_status = "简单场景"

        self.autopilot_flag = True  # 是否自动驾驶
        self.road_id = 4  # 主车所在道路id
        self.speed_limit = 100  # 主车速度限制

    def follow_road(self):
        global drive_status, scene_status, left_right_qian_distance, left_right_hou_distance
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while True:
            self.speed_limit = road_speed_limit[env_map.get_waypoint(self.vehicle.get_location()).lane_id]  # 速度限制
            if self.autopilot_flag:
                drive_status = "自动驾驶"
                self.autopilot_flag = not keyboard.is_pressed("q")

                # 瞬时速度
                if self.instantaneous_speed:
                    if get_speed(self.vehicle) < self.speed_limit:
                        set_speed(self.vehicle, self.speed_limit)

                # 判断变道
                if scene_status == "干扰场景一":
                    if directions:
                        pang_next_flag = True
                        pang_previous_flag = True
                        # 获取旁边车辆信息
                        if directions[0] == "right":
                            now_right_left_lane_info = get_now_road_car(self.vehicle, now_lane_flag=True,
                                                                        right_lane_flag=True)
                            now_lane_next_car_info = now_right_left_lane_info.get("now_lane").get("next_info")  # 前车信息
                            pang_lane_next_car_info = now_right_left_lane_info.get("right_lane").get(
                                "next_info")  # 旁前车信息
                            pang_lane_previous_car_info = now_right_left_lane_info.get("right_lane").get(
                                "previous_info")  # 旁后车信息
                        else:
                            now_right_left_lane_info = get_now_road_car(self.vehicle, now_lane_flag=True,
                                                                        left_lane_flag=True)
                            now_lane_next_car_info = now_right_left_lane_info.get("now_lane").get("next_info")  # 前车信息
                            pang_lane_next_car_info = now_right_left_lane_info.get("left_lane").get(
                                "next_info")  # 旁前车信息
                            pang_lane_previous_car_info = now_right_left_lane_info.get("left_lane").get(
                                "previous_info")  # 旁后车信息
                        left_right_qian_distance = pang_lane_next_car_info[0][1]
                        left_right_hou_distance = pang_lane_previous_car_info[0][1]
                        # 判断是否满足变道条件
                        if pang_lane_previous_car_info:
                            if pang_lane_previous_car_info[0][1] < 50:
                                pang_previous_flag = False
                        if pang_lane_next_car_info:
                            if pang_lane_next_car_info[0][1] < 30:
                                pang_next_flag = False
                        # 满足变道条件
                        # print(pang_previous_flag, pang_next_flag, now_lane_next_car_info[0][1] < self.speed_limit)
                        if pang_previous_flag and pang_next_flag and now_lane_next_car_info[0][
                            1] < self.speed_limit:  # 满足变道条件
                            right_left_lane(self.vehicle)  # 这样不加这个就是自动识别变道
                    else:
                        print("没有变道方向了")
                        pass
                elif scene_status == "简单场景":
                    now_right_left_lane_info = get_now_road_car(self.vehicle, now_lane_flag=True)
                    now_lane_next_car_info = now_right_left_lane_info.get("now_lane").get("next_info")  # 前车信息
                    # print(now_lane_next_car_info)
                    if now_lane_next_car_info:
                        if now_lane_next_car_info[0][1] < 80:
                            print("请接管！！！！！！！！！！！！")
                # 获取前方道路
                waypoint = env_map.get_waypoint(self.vehicle.get_location()).next(
                    max(1, int(get_speed(self.vehicle) / 6)))
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
                sleep(0.01)
            else:
                drive_status = "人工驾驶"
                self.autopilot_flag = keyboard.is_pressed("e")

                # 人工控制车辆
                steer, throttle, brake = get_steering_wheel_info()
                if get_speed(self.vehicle) > self.speed_limit:  # 设置最高速度
                    throttle = 0.5
                elif get_speed(self.vehicle) < 60:  # 设置最低速度
                    throttle = 1
                car_control(self.vehicle, steer, throttle, brake)
                sleep(0.01)


# 副车控制器
class Vice_Control:
    def __init__(self, main_car):
        self.main_car = main_car  # 主车

        self.thread_car_number = 10  # 一个线程控制的车辆数
        self.whichOne = 0  # 用于记录遍历副车的索引
        self.thread_cut_speed = threading.Thread()  # 减速线程

    def follow_road(self):
        vices_car = get_Vice()
        with ThreadPoolExecutor(max_workers=10) as executor:
            while True:
                # print(len(vices_car))
                if not vices_car:  # 如果没有车流
                    # print("Aaa")
                    sleep(0.01)
                    continue
                else:
                    if self.whichOne + self.thread_car_number > len(vices_car):
                        executor.submit(self.control_car, vices_car[self.whichOne:])
                        self.whichOne = 0
                    else:
                        executor.submit(self.control_car,
                                        vices_car[self.whichOne: self.whichOne + self.thread_car_number])
                        self.whichOne += self.thread_car_number
                sleep(0.01)

    def control_car(self, cars):
        global scene_status
        vices_car = get_Vice()
        for car in cars:
            if scene_status != "干扰场景二":
                if not self.thread_cut_speed.is_alive():  # 如果没有车减速
                    # print("前车开始减速")
                    qian_road_car_info = get_now_road_car(self.main_car, now_lane_flag=True).get("now_lane").get(
                        "next_info")
                    if qian_road_car_info:
                        next_car = qian_road_car_info[0][0]  # 前车对象
                        vices_car = [car for car in vices_car if car.id != next_car.id]  # 除去前车的控制
                        self.thread_cut_speed = threading.Thread(target=brake_throttle_retard,
                                                                 args=(next_car, -8.5, 0, 3,))  # 减速线程,第四个参数是延迟
                        self.thread_cut_speed.start()

            pid = VehiclePIDController(car, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
            # 获取前方道路
            waypoint = env_map.get_waypoint(car.get_location()).next(max(1, int(get_speed(car) / 6)))
            if waypoint:
                waypoint = waypoint[0]
            else:
                print(f"前方没有路了,当前车子坐标{car.get_location()},车子对象为{car}")
                car.destory()

            speed_limit = road_speed_limit[env_map.get_waypoint(car.get_location()).lane_id]  # 获取车道的速度限制

            result = pid.run_step(speed_limit, waypoint)
            if get_speed(car) < speed_limit - 20:
                set_speed(car, speed_limit)
            car.apply_control(result)


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
    distance = max(speed, min_direction)

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
        speed_limit = road_speed_limit[env_map.get_waypoint(main_car.get_location()).lane_id] + 20

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
    lights = carla.VehicleLightState.Brake
    # # 设置车辆的灯光状态
    vehicle.set_light_state(carla.VehicleLightState(lights))


# 加速度加减速
def brake_throttle_retard(vehicle, acceleration, target_speed, delay=0):
    """
    加减速
    :param vehicle: 目标车辆
    :param acceleration: 加速度
    :param target_speed: 目标速度
    :param delay: 延迟
    :return:
    """
    pid = VehiclePIDController(vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
    t = time.time()
    sleep(0.1)
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
        sleep(0.01)
    if acceleration < 0:
        # print(f"我开始减速了")
        pass
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
        sleep(0.01)
    for _ in range(10):
        set_speed(vehicle, target_speed)
        sleep(0.01)


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
        spawn_point = carla.Transform(carla.Location(x=-6, z=4), carla.Rotation(pitch=0, yaw=0, roll=0))  # 传感器相对车子的位置设置
        self.sensor = self.world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)  # 添加传感器

        threading.Thread(target=self.show_screen).start()

    def show_screen(self):  # 显示窗口
        # 初始化窗口
        pygame.display.set_caption("pygame模拟场景")

        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT),
                                              pygame.HWSURFACE | pygame.DOUBLEBUF, 32)
        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:  # 窗口关闭
                    pygame.quit()
                    os._exit(0)
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
            self.draw_text(f"左/右前距离：{round(left_right_qian_distance, 2)}m", 20, (0, 60))
            self.draw_text(f"左/右后距离：{round(left_right_hou_distance, 2)}m", 20, (0, 80))
            self.draw_text(f"状态：{drive_status}", 20, (0, 100))
        # 刷新屏幕
        pygame.display.flip()

    def draw_text(self, word, length, position, color=(255, 0, 0)):
        text = self.font.render(word.ljust(length), True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.screen.blit(text, text_rect)


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
    vice_locations = []  # 副车的坐标列表
    vehicle_location = vehicle.get_location()  # 主车坐标
    next_distance = 100
    previous_distance = 100
    for i in range(number):  # 中一
        location = env_map.get_waypoint(vehicle_location).next((i + 1) * next_distance)[0].transform.location  # 前面车
        vice_locations.append(location + carla.Location(z=0.5))
        location = env_map.get_waypoint(vehicle_location).previous((i + 1) * previous_distance)[
            0].transform.location  # 后面车
        vice_locations.append(location + carla.Location(z=0.5))
    for i in range(number):  # 右一
        location = env_map.get_waypoint(vehicle_location).get_right_lane().next((i + 1) * next_distance)[
            0].transform.location  # 右一前车
        vice_locations.append(location + carla.Location(z=0.5))
        location = env_map.get_waypoint(vehicle_location).get_right_lane().previous((i + 1) * previous_distance)[
            0].transform.location  # 右一后车
        vice_locations.append(location + carla.Location(z=0.5))
    for i in range(number):  # 右二
        location = \
            env_map.get_waypoint(vehicle_location).get_right_lane().get_right_lane().next((i + 1) * next_distance)[
                0].transform.location  # 右二前车
        vice_locations.append(location + carla.Location(z=0.5))
        location = \
            env_map.get_waypoint(vehicle_location).get_right_lane().get_right_lane().previous((i + 1) * previous_distance)[
                0].transform.location  # 右二后车
        vice_locations.append(location + carla.Location(z=0.5))
    for i in range(number):  # 左一
        location = env_map.get_waypoint(vehicle_location).get_left_lane().next((i + 1) * next_distance)[
            0].transform.location  # 左一前车
        vice_locations.append(location + carla.Location(z=0.5))
        location = env_map.get_waypoint(vehicle_location).get_left_lane().previous((i + 1) * previous_distance)[
            0].transform.location  # 左一后车
        vice_locations.append(location + carla.Location(z=0.5))
    for i in range(number):  # 左二
        location = env_map.get_waypoint(vehicle_location).get_left_lane().get_left_lane().next((i + 1) * next_distance)[
            0].transform.location  # 左二前车
        vice_locations.append(location + carla.Location(z=0.5))
        location = \
            env_map.get_waypoint(vehicle_location).get_left_lane().get_left_lane().previous((i + 1) * previous_distance)[
                0].transform.location  # 左二后车
        vice_locations.append(location + carla.Location(z=0.5))
    return vehicle_traffic.create_vehicle(vice_locations, vehicle_model="vehicle.mini.cooper_s_2021")


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
def get_steering_wheel_info():
    """
    return: 方向盘、油门、刹车
    """
    # print(self.joystick.get_axis(0), (-self.joystick.get_axis(2) + 1) / 2, (-self.joystick.get_axis(3) + 1) / 2)
    return joystick.get_axis(0), (-joystick.get_axis(2) + 1) / 2, (-joystick.get_axis(3) + 1) / 2


def destroy_lose_vehicle(main_car):  # 销毁失控车辆
    gvices_car = get_Vice()

    def destroy():
        thread_up_speed = threading.Thread()
        while True:
            all_vehicle = list(world.get_actors().filter('*vehicle*'))
            all_vehicle = [car for car in all_vehicle if car.id != main_car.id]  # 除去自车

            vices_car_ids = [car.id for car in gvices_car]  # 有控制的副车列表id
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

# if __name__ == '__main__':
#     destroy_all_vehicles_traffics(world)  # 销毁所有车辆
#
#     vehicle_traffic = Vehicle_Traffic(world)  # 车辆创建对象
#     vehicle = vehicle_traffic.create_vehicle([start_location], vehicle_model="mini")[0]  # 主车
#     destroy_lose_vehicle(vehicle)  # 销毁失控车辆
#     vices_car_list = create_vices(vehicle_traffic, vehicle)  # 创建副车
#
#     main_car_control = Main_Car_Control(vehicle, True)  # 主车控制类
#     vice_car_control = Vice_Control(vehicle)  # 副车控制类
#
#     threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
#     threading.Thread(target=vice_car_control.follow_road).start()  # 启动副车
#
#     window = Window(world, blueprint_library, vehicle)  # 创建窗口
#
#     while True:
#         sleep(1)
