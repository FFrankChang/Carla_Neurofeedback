import threading
import carla
import pygame
from disposition import *
from s01_config import *
import random
import csv
import time
from datetime import datetime



vices_car_list = []  # 所有副车列表
drive_status = "自动驾驶"  
scene_status = "简单场景"  


class DataRecorder(threading.Thread):
    def __init__(self, control_instance, interval=0.1):
        super().__init__()
        self.control = control_instance
        self.interval = interval
        self.fields = ['timestamp', 'vehicle_x', 'vehicle_y', 'steer_value', 'event_triggered', 'event_value']
        self.running = True
        self.filename = self.setup_filename()
        self.setup_file()

    def setup_filename(self):
        directory = './data'
        if not os.path.exists(directory):
            os.makedirs(directory)
        date_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(directory, f'C04_{date_time}.csv')

    def setup_file(self):
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fields)
            writer.writeheader()

    def run(self):
        while self.running:
            current_time = time.time()
            vehicle = self.control.vehicle
            data = {
                'timestamp': current_time,
                'vehicle_x': vehicle.get_location().x,
                'vehicle_y': vehicle.get_location().y,
                'steer_value': self.control.steer,
                'event_triggered': self.control.random_steer_active,
                'event_value': self.control.random_steer_value if self.control.random_steer_active else 0
            }
            self.record_data(data)
            time.sleep(self.interval)

    def record_data(self, data):
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fields)
            writer.writerow(data)

    def stop(self):
        self.running = False
        self.join()

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

# 主车控制器
class Main_Car_Control:
    def __init__(self, main_car, world,instantaneous_speed=False):
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
        self.flag = True
        self.lead_vehicle = None
        self.next_event_time = time.time() + 6
        self.steer_duration = 0  
        self.steer_event_end_time = 0  
        self.random_steer_active = False
        self.random_steer_value = 0
        self.steer = 0

    def follow_road(self):
        global drive_status, scene_status
        self.flag = True
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while self.flag:
            current_time = time.time()
            self.speed_limit = road_speed_limit[env_map.get_waypoint(self.vehicle.get_location()).lane_id]
            if self.autopilot_flag:
                drive_status = "自动驾驶"
                if keyboard.is_pressed("q"):
                    self.autopilot_flag = False
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
                if get_speed(self.vehicle) < self.speed_limit :
                    set_speed(self.vehicle, self.speed_limit)
                self.vehicle.apply_control(result)
                sleep(0.01)
            else:
                drive_status = "人工驾驶"
                if keyboard.is_pressed("e"):  
                    self.autopilot_flag = True
                if not self.random_steer_active and current_time >= self.next_event_time:
                    self.trigger_random_steer_event()
                steer, throttle, brake = get_steering_wheel_info()
                self.steer = steer
                if self.random_steer_active:
                    if current_time <= self.steer_event_end_time:
                        steer += self.random_steer_value
                    else:
                        self.random_steer_active = False
                        self.next_event_time = current_time + random.randint(8, 12)
                car_control(self.vehicle, steer, 0, 0)
                set_speed(self.vehicle, 80)
                sleep(0.01)

    def trigger_random_steer_event(self):
        """Activate a random steer event."""
        self.random_steer_active = True
        self.random_steer_value = random.choice([-0.1, 0.1, -0.05, 0.05])
        self.steer_duration = random.uniform(0.1, 0.2)  
        self.steer_event_end_time = time.time() + self.steer_duration
        print(f"Random steer event triggered for {self.steer_duration:.2f} seconds with steer {self.random_steer_value:.2f}!")

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
        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360  # 屏幕大小
        self.screen = None  # 初始化屏幕窗口
        self.fonts = {} 
        pygame.init()  # 初始化pygame

        # 初始化窗口设置
        self.clock = pygame.time.Clock()
        self.size = 18  # 字体大小
        self.fps = 90  # 帧率
        self.font = pygame.font.Font(r"TTF\宋体.ttf", self.size)  # 初始化字体对象

        # 初始化传感器
        self.blueprint_camera = blueprint_library.find('sensor.camera.rgb')  # 选择一个传感器蓝图
        self.blueprint_camera.set_attribute('image_size_x', f'{self.SCREEN_WIDTH}')  # 传感器获得的图片高度
        self.blueprint_camera.set_attribute('image_size_y', f'{self.SCREEN_HEIGHT}')  # 传感器获得的图片宽度
        self.blueprint_camera.set_attribute('fov', '135')  # 水平方向上能看到的视角度数
        spawn_point = carla.Transform(carla.Location(x=0.15, y = -0.21, z=1.25), carla.Rotation(pitch=-5, yaw=0, roll=0))  # 传感器相对车子的位置设置
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
        fps = self.clock.get_fps()
        self.draw_text(f"fps：{int(fps)}", 20, (0, 0))

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
    # return 0, 0, 0


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



def scene_jian(vehicle, main_car_control, end_location):  # 简单场景
    global scene_status, vices_car_list

    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车

    scene_status = "等待开始"  
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
    
    recorder = DataRecorder(main_car_control)
    recorder.start()
    
    scene_jian(vehicle, main_car_control, interfere_one_location1)
    
    try:
        while True:
            sleep(1)
    finally:
        recorder.stop()