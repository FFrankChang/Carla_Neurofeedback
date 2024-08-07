import threading
import time
import socket
import carla
import pygame
from disposition import *
from s01_config import *
import random


vices_car_list = []  # 所有副车列表
drive_status = "自动驾驶"  
scene_status = "简单场景"  

class Vehicle_Traffic:
    def __init__(self, world):
        # 世界对象
        self.world = world
        # 地图对象
        self.env_map = self.world.get_map()
        # 创建蓝图
        self.blueprint_library = self.world.get_blueprint_library()

    def create_vehicle(self, points=None, number=1, vehicle_model=None, height=0.05):
        vehicles = []
        if vehicle_model:
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            cars = self.blueprint_library.filter('*crown*')
        if not points:
            spawn_points = self.env_map.get_spawn_points()
            for index in range(number):
                vehicle = self.world.try_spawn_actor(random.choice(cars), random.choice(spawn_points))
                if vehicle:
                    vehicles.append(vehicle)
                else:
                    print(f"第{index + 1}辆车未成功生成！")
            check_coordinate(vehicles)
            return vehicles
        for index, point in enumerate(points):
            waypoint = self.env_map.get_waypoint(point)  
            transform = carla.Transform(point, waypoint.transform.rotation)
            vehicle = self.world.try_spawn_actor(random.choice(cars), transform)
            if vehicle:
                vehicles.append(vehicle)
            else:
                print(f"索引为：{index}的车子未成功生成！")
        check_coordinate(vehicles)
        return vehicles
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
    def __init__(self, main_car, world,instantaneous_speed=False):
        """
        主车控制类
        :param main_car: 主车对象
        :param instantaneous_speed: 是否瞬时速度
        """
        self.vehicle = main_car  # 主车对象
        self.instantaneous_speed = instantaneous_speed  # 是否瞬时到达目标速度
        self.scene_status = "简单场景"
        self.world = world
        self.autopilot_flag = True  # 是否自动驾驶
        self.road_id = 4  # 主车所在道路id
        self.speed_limit = 100  # 主车速度限制
        self.flag = True
        self.udp_ip = "192.168.3.9"  # IP of the destination computer
        self.udp_port = 12346  # Port number on the destination computer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket
        self.lead_vehicle = None


    def follow_road(self):
        global drive_status, scene_status
        self.flag = True
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while self.flag:
            self.speed_limit = road_speed_limit[env_map.get_waypoint(self.vehicle.get_location()).lane_id]
            if self.autopilot_flag:
                drive_status = "自动驾驶"
                if keyboard.is_pressed("q"):
                    self.autopilot_flag = False
                    # Send data over UDP when 'q' is pressed for manual takeover
                    message = "play"
                    self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
                
                # 瞬时速度
                if self.instantaneous_speed:
                    if get_speed(self.vehicle) < self.speed_limit:
                        set_speed(self.vehicle, self.speed_limit)

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
                steer, throttle, brake = get_steering_wheel_info()
                if get_speed(self.vehicle) < 80:  # 设置最低速度
                   set_speed(self.vehicle,80)
                car_control(vehicle, steer, throttle, brake)
                sleep(0.001)

    def stop_vehicle(self):
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
            if scene_status == "简单场景":
                if not self.thread_cut_speed.is_alive():  # 如果没有车减速
                    qian_road_car_info = get_now_road_car(self.main_car, now_lane_flag=True).get("now_lane").get("next_info")
                    if qian_road_car_info:
                        next_car = qian_road_car_info[0][0]  # 前车对象
                        vices_car_list = [car for car in vices_car_list if car.id != next_car.id]  # 除去前车的控制
                        self.thread_cut_speed = threading.Thread(target=brake_throttle_retard,args=(next_car, -8.5, 0, 3,))  # 减速线程,第四个参数是延迟
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
            if get_speed(car) < speed_limit:
                set_speed(car, speed_limit)
            car.apply_control(result)

def set_brake_lights(vehicle):
    lights = carla.VehicleLightState.Brake
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
        sleep(0.001)
    for _ in range(10):
        set_speed(vehicle, target_speed)
        sleep(0.001)


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
        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 5760, 1080  # 屏幕大小

        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360  # 屏幕大小
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
        global drive_status
        i = np.array(image.raw_data)
        i2 = i.reshape((self.SCREEN_HEIGHT, self.SCREEN_WIDTH, 4))
        i3 = i2[:, :, :3]
        i3 = np.rot90(i3, k=1, axes=(0, 1))
        i3 = i3[..., ::-1]
        img_surface = pygame.surfarray.make_surface(np.flip(i3, axis=0))
        self.screen.blit(img_surface, (0, 0))  # 绘制图片
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


def create_vices(vehicle_traffic, vehicle):
    """
    创建车流
    :param vehicle_traffic: 创建车子的对象
    :param vehicle: 主车
    :return: 返回车流的车辆列表
    """
    vice_locations = []  # 副车的坐标列表
    vehicle_location = vehicle.get_location()  # 主车坐标
    number = 3 # 每个方向生成的车辆数
    max_offset = 5  # 最大偏移量
    ahead_distance = 30
    back_distance = 30
    # 创建前方的车流
    for i in range(number):
        # 中前
        location = env_map.get_waypoint(vehicle_location).next((i + 1) * ahead_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 右一前
        location = env_map.get_waypoint(vehicle_location).get_right_lane().next((i + 1) * ahead_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 右二前
        location = env_map.get_waypoint(vehicle_location).get_right_lane().get_right_lane().next((i + 1) * ahead_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 左一前
        location = env_map.get_waypoint(vehicle_location).get_left_lane().next((i + 1) * ahead_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 左二前
        location = env_map.get_waypoint(vehicle_location).get_left_lane().get_left_lane().next((i + 1) * ahead_distance)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))


    # 创建后方的车流，只有一排
    for i in range(1):  # 只循环一次
        # 中后
        location = env_map.get_waypoint(vehicle_location).previous((i + 1) * back_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 右一后
        location = env_map.get_waypoint(vehicle_location).get_right_lane().previous((i + 1) * back_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 右二后
        location = env_map.get_waypoint(vehicle_location).get_right_lane().get_right_lane().previous((i + 1) * back_distance )[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 左一后
        location = env_map.get_waypoint(vehicle_location).get_left_lane().previous((i + 1) * back_distance)[0].transform.location
        vice_locations.append(location + carla.Location(z=0.5))
        # 左二后
        location = env_map.get_waypoint(vehicle_location).get_left_lane().get_left_lane().previous((i + 1) * back_distance)[0].transform.location
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
    return joystick.get_axis(0), (-joystick.get_axis(1) + 1)/2, (-joystick.get_axis(2) + 1)/2


def destroy_lose_vehicle(main_car):  # 销毁失控车辆
    global vices_car_list

    def destroy():
        while True:
            all_vehicle = list(world.get_actors().filter('*vehicle*'))
            all_vehicle = [car for car in all_vehicle if car.id != main_car.id]  # 除去自车

            vices_car_ids = [car.id for car in vices_car_list]  # 有控制的副车列表id
            lose_vices = [car for car in all_vehicle if car.id not in vices_car_ids]  # 失去控制的车辆列表
            for car in lose_vices:
                distance = main_car.get_location().distance(car.get_location())
                if distance < 10:
                    sleep(1)
                    car.destroy()
            sleep(0.1)
    threading.Thread(target=destroy).start()

def destroy_vice(vehicle):  # 销毁除自车以外的所有车辆
    actors = list(world.get_actors().filter('*vehicle*'))
    actors = [car for car in actors if car.id != vehicle.id]
    for car in actors:
        car.destroy()



def scene_jian(vehicle, main_car_control, vice_car_control, end_location):  # 简单场景
    global scene_status, vices_car_list
    vices_car_list += create_vices(vehicle_traffic, vehicle)  # 创建副车

    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    threading.Thread(target=vice_car_control.follow_road).start()  # 启动副车

    scene_status = "等待36s开始"  
    t = time.time()
    time_gap = 3
    while time.time() - t < time_gap:
        scene_status = f"倒计时{int(time_gap - (time.time() - t))}s (简单场景)"
        sleep(1)
    scene_status = "简单场景"



if __name__ == '__main__':
    destroy_all_vehicles_traffics(world)  # 销毁所有车辆

    vehicle_traffic = Vehicle_Traffic(world)  # 车辆创建对象
    vehicle = vehicle_traffic.create_vehicle([easy_location1], vehicle_model="vehicle.lincoln.mkz_2020")[0]  # 创建主车
    destroy_lose_vehicle(vehicle)  
    window = Window(world, blueprint_library, vehicle)  # 创建窗口
    main_car_control = Main_Car_Control(vehicle,world, True)  # 主车控制类
    vice_car_control = Vice_Control(vehicle)  # 副车控制类

    scene_jian(vehicle, main_car_control, vice_car_control, interfere_one_location1)
    while True:
        sleep(1)
