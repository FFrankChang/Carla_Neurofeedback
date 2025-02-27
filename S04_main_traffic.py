import threading
import carla
import pygame
from disposition import *
from s04_config_traffic import *
import random
import csv
import time
from datetime import datetime

vices_car_list = []  # 所有副车列表
drive_status = "自动驾驶"  
scene_status = "简单场景"  

class DataRecorder(threading.Thread):
    def __init__(self, control_instance, interval=0.01):
        super().__init__()
        self.control = control_instance
        self.interval = interval
        self.fields = ['timestamp', 'vehicle_x', 'vehicle_y', 'steer', 'throttle', 'brake', 'event_triggered', 'event_value']
        self.running = True
        self.filename = self.setup_filename()
        self.setup_file()

    def setup_filename(self):
        directory = './data'
        if not os.path.exists(directory):
            os.makedirs(directory)
        date_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(directory, f'C04_traffic_{date_time}.csv')

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
                'steer': self.control.steer,
                'throttle': self.control.throttle,
                'brake': self.control.brake,
                'event_triggered': self.control.random_steer_active,
                'event_value': self.control.random_steer_value if self.control.random_steer_active else 0
            }
            self.record_data(data)
            time.sleep(self.interval)

    def record_data(self, data):
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fields)
            writer.writerow(data)
            csvfile.flush()
            
    def stop(self):
        self.running = False
        self.join()

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
        print(self.tm)
        self.tm.set_synchronous_mode(True)  # 如果使用同步模式
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
            cars = self.blueprint_library.filter('*crown*')

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
            else:
                print(f"索引为：{index}的车子未成功生成！")
        return vehicles
    
    def create_main_vehicle(self, points=None, vehicle_model=None):
        vehicles = []
        if vehicle_model:
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            cars = self.blueprint_library.filter('*crown*')

        for index, point in enumerate(points):
            waypoint = self.env_map.get_waypoint(point)  
            transform = carla.Transform(point, waypoint.transform.rotation)
            vehicle = self.world.try_spawn_actor(random.choice(cars), transform)
            if vehicle:
                vehicles.append(vehicle)
            else:
                print(f"索引为：{index}的车子未成功生成！")
        return vehicles
# 主车控制器
class Main_Car_Control:
    def __init__(self, main_car, world, instantaneous_speed=False):
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
        self.next_event_time = time.time() + 30
        self.steer_duration = 0  
        self.steer_event_end_time = 0  
        self.random_steer_active = False
        self.random_steer_value = 0
        self.steer = 0
        self.throttle = 0
        self.brake = 0

    def follow_road(self):
        global drive_status
        self.flag = True
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while self.flag:
            drive_status = "人工驾驶"
            steer, throttle, brake = get_steering_wheel_info()
            self.steer = steer
            self.throttle = throttle
            self.brake = brake
            current_time = time.time()

            if not self.random_steer_active and current_time >= self.next_event_time:
                self.trigger_random_steer_event()

            if self.random_steer_active:
                if current_time <= self.steer_event_end_time:
                    steer += self.random_steer_value
                else:
                    self.random_steer_active = False
                    self.next_event_time = current_time + random.randint(8, 12)

            car_control(self.vehicle, steer, throttle, brake)

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
        self.blueprint_camera.set_attribute('fov', '140')  # 水平方向上能看到的视角度数
        spawn_point = carla.Transform(carla.Location(x=0.1, y = -0.3, z=1.25), carla.Rotation(pitch=-8, yaw=0, roll=0))  # 传感器相对车子的位置设置
        self.sensor = self.world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)  # 添加传感器

        threading.Thread(target=self.show_screen).start()

    def show_screen(self):  # 显示窗口
        # 初始化窗口
        pygame.display.set_caption("pygame模拟场景")
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), pygame.RESIZABLE, 32)
        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:  # 窗口关闭
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
    steer = round(steer, 3)
    throttle = round(throttle, 3)
    brake = round(brake, 3)
    control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
    vehicle.apply_control(control)

# 销毁所有车辆
def destroy_all_vehicles_traffics(world, vehicle_flag=True, traffic_flag=True):
    actors = []
    if vehicle_flag:
        actors += list(world.get_actors().filter('*vehicle*'))
    if traffic_flag:
        actors += list(world.get_actors().filter("*prop*"))
    for actor in actors:
        actor.destroy()

# 获取方向盘信息
def get_steering_wheel_info():
    return joystick.get_axis(0), (-joystick.get_axis(1) + 1)/2, (-joystick.get_axis(2) + 1)/2

# 场景
def scene_jian( main_car_control, end_location):  # 简单场景
    global scene_status, vices_car_list
    threading.Thread(target=main_car_control.follow_road).start()  # 启动主车
    scene_status = "简单场景"


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
    destroy_all_vehicles_traffics(world)  # 销毁所有车辆
    random_traffic_points = generate_random_locations_around_vehicle(
        easy_location1, 
        num_vehicles=100, 
        x_range=(-100, 600),  
        y_range=(-12.5, 12.5),    
        z=5        
    )

    vehicle_traffic = Vehicle_Traffic(world)  # 车辆创建对象
    vehicle = vehicle_traffic.create_main_vehicle([easy_location1], vehicle_model="vehicle.lincoln.mkz_2020")[0]
    random_traffic = vehicle_traffic.create_vehicle(points=random_traffic_points)
    window = Window(world, vehicle_traffic.blueprint_library, vehicle)
    main_car_control = Main_Car_Control(vehicle, world, True)
    recorder = DataRecorder(main_car_control)
    recorder.start()
    scene_jian(main_car_control, interfere_one_location1)
    
    try:
        while True:
            world.tick()  # 确保同步更新
            time.sleep(0.01)
    finally:
        recorder.stop()