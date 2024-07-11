from disposition import *
import time
from config import *
import threading
import csv
import datetime
import sys
import socket
class DataRecorder:
    def __init__(self, vehicle, subject,date,now_condition):
        self.vehicle = vehicle
        self.subject = subject
        self.date = date
        self.condition = now_condition
        self.vehicle = vehicle
        self.rate = 0.02  # Data recording rate in seconds
        self.running = True
        self.filename = f"./data/carla_s02_{self.subject}_{self.date}_{self.condition}_{time.time()}.csv"
        self.fields = ['Time', 'Speed', 'Acceleration', 'Location', 'Steering', 'Throttle', 'Brake','TOR','Takeover']
        self.tor = False
        self.takeover = False
        with open(self.filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.fields)
            writer.writeheader()
    def record_tor(self):
        self.tor = True

    def record_takeover(self):
        self.takeover = True

    def record_data(self):
        last_speed = get_speed(self.vehicle)
        last_time = time.time()

        while self.running:
            current_speed = get_speed(self.vehicle)
            current_time = time.time()
            acceleration = 0

            location = self.vehicle.get_location()
            steering, throttle, brake = get_steering_wheel_info()

            data = {
                'Time': time.time(),
                'Speed': current_speed,
                'Acceleration': acceleration,
                'Location': f"{location.x}, {location.y}, {location.z}",
                'Steering': steering,
                'Throttle': throttle,
                'Brake': brake,
                'TOR':self.tor,
                'Takeover':self.takeover,
            }

            with open(self.filename, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.fields)
                writer.writerow(data)

            last_speed = current_speed
            last_time = current_time

            time.sleep(self.rate)

    def start_recording(self):
        threading.Thread(target=self.record_data).start()

    def stop_recording(self):
        self.running = False

class Vehicle_Control:
    def __init__(self, vehicle,sock,data_recoder):
        self.vehicle = vehicle  # 车子对象
        self.autopilot_flag = True  # 是否自动驾驶，默认沿着当前道路，没有路了就停止，为False可用方向盘驾驶
        self.lose_control = False  # 这个是有控制权，为True是失去所有控制
        self.autopilot_speed_limit = 100  # 自动驾驶速度限制
        self.labour_speed_limit = 140  # 人工驾驶速度限制
        self.labour_low_speed_limit = 0  # 人工驾驶最低速度
        self.driver_status = "自动驾驶"
        self.instantaneous_speed = True
        self.sock = sock
        self.data_recoder = data_recoder

    def follow_lane(self):
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)

        def control():
            while True:
                if self.lose_control:  # 判断有没有失去控制
                    self.driver_status = "没有控制"
                    pass
                elif self.autopilot_flag:
                    self.driver_status = "自动驾驶"


                    for event in pygame.event.get():
                        if event.type == pygame.JOYBUTTONDOWN:
                            if event.button == 4 or event.button == 5:
                                self.autopilot_flag = False
                                self.data_recoder.record_takeover()
                                message = 'play'
                                sock.sendto(message.encode(), ('127.0.0.1', 12346)) 
                    # self.autopilot_flag = not keyboard.is_pressed("q")
                    # 自动驾驶车辆，瞬时速度
                    if self.instantaneous_speed:
                        if get_speed(self.vehicle) < self.autopilot_speed_limit:
                            set_speed(self.vehicle, self.autopilot_speed_limit)

                    waypoint = env_map.get_waypoint(self.vehicle.get_location()).next(
                        max(1, int(get_speed(self.vehicle) / 6)))
                    if waypoint:
                        waypoint = waypoint[0]
                    else:
                        print(f"前方没有路了,当前车子坐标{self.vehicle.get_location()},车子对象为{self.vehicle}")
                        set_speed(self.vehicle, 0)
                        return

                    result = pid.run_step(self.autopilot_speed_limit, waypoint)
                    self.vehicle.apply_control(result)
                else:
                    self.driver_status = "人工驾驶"
                    self.autopilot_flag = keyboard.is_pressed("e")
                    # 人工控制车辆
                    steer, throttle, brake = get_steering_wheel_info()
                    if get_speed(self.vehicle) > self.labour_speed_limit:  # 设置最高速度
                        throttle = 0.4
                    elif get_speed(self.vehicle) < self.labour_low_speed_limit:
                        throttle = 1  # 最第速度
                    result = carla.VehicleControl(steer=round(steer, 3), throttle=round(throttle, 3),
                                                  brake=round(brake, 3))
                    self.vehicle.apply_control(result)
                sleep(0.01)
 
        threading.Thread(target=control).start()

    def right_left_lane(self, direction=None, min_direction=10, method="pid", line_number=1, draw=False):
        """
        左转或右转
        :param direction: 左转还是右转,接收"left"和"right"
        :param min_direction: 最小变道距离
        :param method: 变道所使用的方法，默认用pid，还有agent
        :return:
        """
        self.lose_control = True
        # 判断有没有可变道路,得到direction方向值
        if not direction:  # 如果没有传左/右变道
            # 先判断左右是否有道路
            direction = str(env_map.get_waypoint(self.vehicle.get_location()).lane_change).lower()
            if not direction:  # 如果没有可变道路，找前方看有没有可以变道
                direction = str(env_map.get_waypoint(self.vehicle.get_location()).next(max(get_speed(self.vehicle), 5))[
                                    0].lane_change).lower()
                if not direction:
                    print("没有可变道路！！！！！！！！")
                    return
                elif direction == "both":
                    direction = random.choice(["right", "left"])
            elif direction == "both":
                direction = random.choice(["right", "left"])

        # PID
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        # 获取当前速度
        speed = get_speed(self.vehicle)
        # 设置速度防止车子在已有速度上突然减速
        set_speed(self.vehicle, speed)
        # 变道距离,根据速度实现
        distance = max(speed, min_direction)
        print(distance)

        while True:  # 获取到变道后的终点坐标
            location = self.vehicle.get_location()
            if direction == "right":
                for i in range(line_number):
                    location = env_map.get_waypoint(location).get_right_lane().transform.location
            else:
                for i in range(line_number):
                    location = env_map.get_waypoint(location).get_left_lane().transform.location
            waypoint = env_map.get_waypoint(location)
            if not waypoint:
                sleep(0.01)
                continue
            waypoint = waypoint.next(distance + 5 * line_number)[0]
            break

        end_location = waypoint.transform.location
        if method == "agent":
            self.autopilot_agent(end_location)
            return
        if draw:
            draw_line(locations=[self.vehicle.get_location(), end_location],
                      life_time=(15 - get_speed(self.vehicle)) / 10)  # 划线
        while True:
            if self.vehicle.get_location().distance(end_location) < 0.5:
                now_time = time.time()
                while time.time() - now_time < 1:  # 变道完成后再执行一秒往前开
                    waypoint = env_map.get_waypoint(self.vehicle.get_location()).next(int(get_speed(self.vehicle)))
                    if waypoint:
                        waypoint = waypoint[0]
                    else:
                        print("变道完前方没路了")
                    result = pid.run_step(self.autopilot_speed_limit, waypoint)
                    self.vehicle.apply_control(result)
                    sleep(0.01)
                print("变道完毕")
                self.lose_control = False
                return
            set_speed(self.vehicle, self.autopilot_speed_limit)
            result = pid.run_step(self.autopilot_speed_limit, waypoint)
            self.vehicle.apply_control(result)
            sleep(0.01)

    def autopilot_agent(self, end_location, ignore_traffic_light=True, mode="aggressive", distance=4.5, draw=False,
                        lift_time=10):
        """
        自动驾驶车辆到终点坐标
        :param end_location: 终点坐标
        :param ignore_traffic_light: 是否忽略交通规则
        :param mode: 模式，有三种，速度不一样  cautious(最慢)，normal(正常),aggressive(稍快)
        :param distance: 每个waypoint之间的距离
        :param draw: 是否绘制线路
        :param lift_time: 路线存活时间
        """
        self.lose_control = True
        agent = BehaviorAgent(self.vehicle, ignore_traffic_light=ignore_traffic_light, behavior=mode)
        agent.speed_limit = self.autopilot_speed_limit
        agent._sampling_resolution = distance

        agent.set_destination(self.vehicle.get_location(), end_location, clean=True)

        while True:
            agent.update_information(self.vehicle)
            if len(agent._local_planner.waypoints_queue) < 2:
                self.lose_control = False
                print('======== Success, Arrivied at Target Point!')
                return True
            control = agent.run_step(debug=draw)
            self.vehicle.apply_control(control)
            sleep(0.01)

    def autopilot_pid(self, end_location, draw=False):
        """
        自动驾驶到终点
        :param end_location: 终点坐标
        :return:
        """
        self.lose_control = True
        # PID
        pid = VehiclePIDController(self.vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        while True:
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
                self.lose_control = False
                print("======== Success, Arrivied at Target Point!")
                return
            # 省略前几个waypoint，防止速度过快往回跑
            waypoint = path[sampling_resolution + 1][0]
            if draw:
                draw_line(locations=[self.vehicle.get_location(), waypoint.transform.location], life_time=0.1)  # 划线
            result = pid.run_step(self.autopilot_speed_limit, waypoint)
            self.vehicle.apply_control(result)
            sleep(0.01)


class Window:
    def __init__(self, vehicle, sensor_type='sensor.camera.rgb'):
        """
        创建车子的pygame窗口显示
        :param vehicle: 车子对象
        """
        self.vehicle = vehicle
        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360  # 屏幕大小
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 5760, 1080  # 屏幕大小
        self.screen = None  # 初始化屏幕窗口
        pygame.init()  # 初始化pygame

        # 初始化窗口设置
        self.clock = pygame.time.Clock()
        self.size = 18  # 字体大小
        self.fps = 60  # 帧率
        self.font = pygame.font.Font(r"TTF\宋体.ttf", self.size)  # 初始化字体对象

        # 初始化传感器
        self.blueprint_camera = blueprint_library.find(sensor_type)  # 选择一个传感器蓝图
        self.blueprint_camera.set_attribute('image_size_x', f'{self.SCREEN_WIDTH}')  # 传感器获得的图片高度
        self.blueprint_camera.set_attribute('image_size_y', f'{self.SCREEN_HEIGHT}')  # 传感器获得的图片宽度
        self.blueprint_camera.set_attribute('fov', '110')  # 水平方向上能看到的视角度数
        spawn_point = carla.Transform(carla.Location(x=-0.1, y=-0.21, z=1.2), carla.Rotation(pitch=0, yaw=0, roll=0))  # 传感器相对车子的位置设置
        self.sensor = world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)  # 添加传感器

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
        i = np.array(image.raw_data)
        i2 = i.reshape((self.SCREEN_HEIGHT, self.SCREEN_WIDTH, 4))
        i3 = i2[:, :, :3]
        i3 = np.rot90(i3, k=1, axes=(0, 1))
        i3 = i3[..., ::-1]
        img_surface = pygame.surfarray.make_surface(np.flip(i3, axis=0))
        self.screen.blit(img_surface, (0, 0))  # 绘制图片

        # 添加文字信息
        self.draw_text(f"速度：{round(get_speed(self.vehicle), 2)}km/h", 20, (0, 0))
        # 刷新屏幕
        pygame.display.flip()

    def draw_text(self, word, length, position, color=(255, 0, 0)):
        text = self.font.render(word.ljust(length), True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.screen.blit(text, text_rect)


# 副车控制器
class Vice_Control:
    def __init__(self):
        self.vices = []  # 主车

        self.thread_car_number = 10  # 一个线程控制的车辆数
        self.thread_number = 1  # 这个是线程数
        self.whichOne = 0  # 用于记录遍历副车的索引
        self.speed_limit = 60  # 副车的速度限制

    def follow_road(self):
        def control():
            with ThreadPoolExecutor(max_workers=self.thread_number) as executor:
                while True:
                    if not self.vices:  # 如果没有车流
                        sleep(0.01)
                        continue
                    else:
                        if self.whichOne + self.thread_car_number > len(self.vices):
                            executor.submit(self.control_car, self.vices[self.whichOne:])
                            self.whichOne = 0
                        else:
                            executor.submit(self.control_car,
                                            self.vices[self.whichOne: self.whichOne + self.thread_car_number])
                            self.whichOne += self.thread_car_number
                    sleep(0.01)

        threading.Thread(target=control).start()

    def control_car(self, cars):
        for car in cars:
            pid = VehiclePIDController(car, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
            # 获取前方道路
            waypoint = env_map.get_waypoint(car.get_location()).next(max(1, int(get_speed(car) / 6)))
            if waypoint:
                waypoint = waypoint[0]
            else:
                print(f"前方没有路了,当前车子坐标{car.get_location()},车子对象为{car}")
                car.destory()
            result = pid.run_step(self.speed_limit, waypoint)
            if get_speed(car) < self.speed_limit - 10:
                set_speed(car, self.speed_limit)
            car.apply_control(result)

    def lose_vice(self, car):
        for i in self.vices:
            if i.id == car.id:
                self.vices.remove(car)
                break


def create_actor(locations, model="vehicle.mini.cooper_s_2021", height=0.1):
    def create(location):
        transform = env_map.get_waypoint(location).transform
        transform.location.z += height
        blueprint = random.choice(blueprint_library.filter(model))
        actor = world.try_spawn_actor(blueprint, transform)
        if not actor:
            return actor
        # while actor.get_location().x == 0 and actor.get_location().y == 0:
        #     print("dwadw")
        #     sleep(0.001)
        return actor

    if isinstance(locations, list):
        actors = []
        for location in locations:
            actors.append(create(location))
        return actors
    else:
        return create(locations)



def draw_line(location1=None, location2=None, locations=None, thickness=0.1, life_time=10.0,
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


def set_speed(vehicle, speed_kmh):
    """
    强制设置车子速度
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


def draw_arrow(locations, distance=10, height=1):
    """
    绘制起点专用的，绘制世界中箭头，一直存在的
    :param locations: carla.Location列表
    :param distance: 箭头绘制的距离
    :param height: 绘制的高度
    """
    debug = world.debug
    for location in locations:
        arrow_location = location + carla.Location(z=height)  # 假设箭头位置略高于地面
        target_location = env_map.get_waypoint(arrow_location).next(distance)[0].transform.location
        target_location.z += height
        # 绘制箭头
        debug.draw_arrow(arrow_location, target_location, thickness=0.3, arrow_size=0.5, color=carla.Color(255, 0, 0))


def get_steering_wheel_info():
    """
    return: 方向盘、油门、刹车
    """
    # 这里0,2,3根据实际情况的方向盘参数
    return joystick.get_axis(0), (-joystick.get_axis(1) + 1) / 2, (-joystick.get_axis(2) + 1) / 2


def destroy_all_vehicles_traffics(vehicles=None, vehicle_flag=True, traffic_flag=True, people_flag=True):
    """
    销毁世界中的所有车辆，交通标志，人
    :param vehicles:需要排除的车辆列表
    :param vehicle_flag: 是否销毁车辆，默认销毁
    :param traffic_flag: 是否销毁交通标志，默认销毁
    :param people_flag: 是否销毁人，默认销毁
    :return:
    """
    actors = []
    if vehicle_flag:
        actors += list(world.get_actors().filter('*vehicle*'))
    if traffic_flag:
        actors += list(world.get_actors().filter("*prop*"))
    if people_flag:
        actors += list(world.get_actors().filter("*walker*"))
    if vehicles:
        if isinstance(vehicles, list):
            car_id = [car.id for car in vehicles]
        else:
            car_id = [vehicles.id]
        actors = [car for car in actors if car.id not in car_id]
    # 销毁每个车辆
    for actor in actors:
        actor.destroy()




def brake_throttle_retard(vehicle, acceleration, target_speed, direction=True):
    """
    加减速
    :param vehicle: 目标车辆
    :param acceleration: 加速度
    :param target_speed: 目标速度
    :param direction: 方向
    :return:
    """
    pid = VehiclePIDController(vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
    t = time.time()
    speed = get_speed(vehicle)
    while abs(get_speed(vehicle) - target_speed) > 1:
        if direction:
            # 获取前方道路
            waypoint = env_map.get_waypoint(vehicle.get_location()).next(max(1, int(get_speed(vehicle) / 6)))
        else:
            # 获取前方道路
            waypoint = env_map.get_waypoint(vehicle.get_location()).previous(max(1, int(get_speed(vehicle) / 6)))
        if waypoint:
            waypoint = waypoint[0]
        else:
            print(f"前方没有路了,当前车子坐标{vehicle.get_location()},车子对象为{vehicle}")
            return
        result = pid.run_step(target_speed, waypoint)
        result.brake = 0
        result.throttle = 0
        vehicle.apply_control(result)  # 这个只控制方向盘

        sp = (max(0, speed + acceleration * (time.time() - t) * 3.6))
        set_speed(vehicle, sp)
        sleep(0.01)
    for _ in range(10):
        set_speed(vehicle, target_speed)
        sleep(0.01)


def right_left_lane(car, speed_limit=40, direction=None, min_direction=10, method="pid", line_number=1, draw=False):
    """
    左转或右转
    :param car: 车子对象
    :param speed_limit:速度限制
    :param direction: 左转还是右转,接收"left"和"right"
    :param min_direction: 最小变道距离
    :param method: 变道所使用的方法，默认用pid，还有agent
    :return:
    """
    # 判断有没有可变道路,得到direction方向值
    if not direction:  # 如果没有传左/右变道
        # 先判断左右是否有道路
        direction = str(env_map.get_waypoint(car.get_location()).lane_change).lower()
        if not direction:  # 如果没有可变道路，找前方看有没有可以变道
            direction = str(env_map.get_waypoint(car.get_location()).next(max(get_speed(car), 5))[
                                0].lane_change).lower()
            if not direction:
                print("没有可变道路！！！！！！！！")
                return
            elif direction == "both":
                direction = random.choice(["right", "left"])
        elif direction == "both":
            direction = random.choice(["right", "left"])

    # PID
    pid = VehiclePIDController(car, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
    # 获取当前速度
    speed = get_speed(car)
    # 设置速度防止车子在已有速度上突然减速
    set_speed(car, speed)
    # 变道距离,根据速度实现
    distance = max(speed, min_direction)

    while True:  # 获取到变道后的终点坐标
        location = car.get_location()
        if direction == "right":
            for i in range(line_number):
                location = env_map.get_waypoint(location).get_right_lane().transform.location
        else:
            for i in range(line_number):
                location = env_map.get_waypoint(location).get_left_lane().transform.location
        waypoint = env_map.get_waypoint(location)
        if not waypoint:
            sleep(0.01)
            continue
        waypoint = waypoint.previous(distance + 5 * line_number)[0]
        break

    end_location = waypoint.transform.location
    if draw:
        draw_line(locations=[car.get_location(), end_location],
                  life_time=(15 - get_speed(car)) / 10)  # 划线
    while True:
        if car.get_location().distance(end_location) < 0.5:
            now_time = time.time()
            while time.time() - now_time < 0.1:  # 变道完成后再执行一秒往前开
                waypoint = env_map.get_waypoint(car.get_location()).previous(int(get_speed(car)))
                if waypoint:
                    waypoint = waypoint[0]
                else:
                    print("变道完前方没路了")
                result = pid.run_step(speed_limit, waypoint)
                car.apply_control(result)
                sleep(0.01)
            print("变道完毕")
            return
        set_speed(car, speed_limit)
        result = pid.run_step(speed_limit, waypoint)
        car.apply_control(result)
        sleep(0.01)


def check_vehicle_destroy(vehicle, vices, location):
    def check():
        while True:
            if not vices:
                return
            for car in vices:
                distance1 = vehicle.get_location().distance(location)
                distance2 = car.get_location().distance(location)
                if distance1 > distance2 + 15:
                    car.destroy()
                    vices.remove(car)
            sleep(0.01)

    threading.Thread(target=check).start()


if __name__ == '__main__':
    udp_ip = "127.0.0.1"  # IP of the destination computer
    udp_port = 12346  # Port number on the destination computer
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

    subject_id = "SUBJECT" if len(sys.argv) < 2 else sys.argv[1]
    date_id = "DAY" if len(sys.argv) < 3 else sys.argv[2]
    condition = "CONDITION" if len(sys.argv) < 4 else sys.argv[3]
    destroy_all_vehicles_traffics()
  
    vehicle = create_actor(main_car_location, model="vehicle.lincoln.mkz_2020")  # 创建主车
    data_recorder = DataRecorder(vehicle,subject=subject_id,date=date_id,now_condition=condition)  # Initialize the data recorder
    data_recorder.start_recording()  
    vehicle_control = Vehicle_Control(vehicle,sock,data_recorder)  # 主车控制类

    Window(vehicle)  # 窗口

    # 主车设置
    # vehicle_control.autopilot_flag=False
    vehicle_control.autopilot_speed_limit = 30
    vehicle_control.labour_speed_limit = 30
    vehicle_control.labour_low_speed_limit = 0
    vehicle_control.follow_lane()



    for i in range(5):
        vice_control = Vice_Control()  # 副车控制类
        static_vice_car = []  # 静止车辆列表
        static_vice_car += [vehicle]
        # 副车设置
        vice_control.follow_road()
        vice_control.speed_limit = 15
        print(f"第{i + 1}次仿真")
        static_car1 = create_actor(
            env_map.get_waypoint(curve_start_one_location).previous(20)[0].transform.location)
        # 场景一
        while vehicle.get_location().distance(main_car_location) < 300:
            sleep(0.01)
        message = "tor"
        sock.sendto(message.encode(), (udp_ip, udp_port))
        data_recorder.record_tor()

        print("请接管")
        # 场景一静1
        static_vice_car += [static_car1]  # 添加静止车辆列表
        # 弯道处车
        vice_car = create_actor(line_vice_location_one)
        location = env_map.get_waypoint(curve_start_one_location).previous(40)[0].transform.location
        while vehicle.get_location().distance(location) > 5:  # 到达距离起点四十米时
            sleep(0.01)

        vice_control.vices += [vice_car]

        # 场景二
        # 创建直道二的车
        locations = []
        location1 = env_map.get_waypoint(curve_end_one_location).next(20)[0].transform.location  # 静止车
        location2 = env_map.get_waypoint(curve_end_one_location).next(30)[0].transform.location
        location3 = env_map.get_waypoint(curve_end_one_location).next(40)[0].transform.location
        locations += [location1, location2, location3]
        vices = create_actor(locations)
        static_vice_car += vices  # 添加静止车辆列表
        while vehicle.get_location().distance(curve_end_one_location) > 5:  # 到达弯道终点
            sleep(0.01)
        # 启动直段二的车
        vice_control.vices += vices[1:]
        check_vehicle_destroy(vehicle, vices[1:], curve_end_one_location)  # 检查车辆中的车
        # 等待到达场景二起点
        while vehicle.get_location().distance(curve_start_two_location) > 40:
            sleep(0.01)
        destroy_all_vehicles_traffics(static_vice_car)  # 销毁车辆
        # 场景二
        locations = [line_vice_location_two,
                     env_map.get_waypoint(line_vice_location_two).previous(15)[0].transform.location]
        vices = create_actor(locations)
        vice_control.vices = vices

        # 等待到达场景三
        locations = [env_map.get_waypoint(curve_start_three_location).previous(15)[0].transform.location,
                     env_map.get_waypoint(curve_start_three_location).previous(30)[0].transform.location,
                     env_map.get_waypoint(curve_start_three_location).previous(45)[0].transform.location]
        vices = create_actor(locations)
        static_vice_car += vices
        while vehicle.get_location().distance(vices[0].get_location()) > 10:
            sleep(0.01)
        brake_throttle_retard(vices[0], 4, 40)
        vice_control.vices = [vices[0]]
        vice_control.speed_limit = 40

        while vices[0].get_location().distance(curve_end_three_location) < 40:
            sleep(0.01)
        destroy_all_vehicles_traffics(static_vice_car)
        vice_control.vices = []
        right_left_lane(vices[0], 40, "left")
        brake_throttle_retard(vices[0], -8, 0, False)

        while vehicle.get_location().distance(curve_start_four_location) > 40:
            sleep(0.01)
        check_vehicle_destroy(vehicle, [vices[0]], line_vice_location_two)
        destroy_all_vehicles_traffics(static_vice_car)
        # 场景四
        vices = [create_actor(line_vice_end_location_four, model="vehicle.kawasaki.ninja")]
        locations = [env_map.get_waypoint(line_vice_end_location_four).previous(15)[0].transform.location,
                     env_map.get_waypoint(line_vice_end_location_four).previous(30)[0].transform.location,
                     env_map.get_waypoint(line_vice_end_location_four).previous(45)[0].transform.location,
                     env_map.get_waypoint(line_vice_end_location_four).previous(60)[0].transform.location,
                     env_map.get_waypoint(line_vice_end_location_four).previous(75)[0].transform.location,
                     env_map.get_waypoint(line_vice_end_location_four).previous(90)[0].transform.location]
        vices += create_actor(locations)
        static_vice_car += vices
        while vehicle.get_location().distance(curve_start_four_location) > 10:
            sleep(0.01)
        vice_control.vices = vices
        vice_control.speed_limit = 15
        destroy_all_vehicles_traffics(static_vice_car)

        vices = create_actor([line_vice_end_location_five1, line_vice_end_location_five2, line_vice_end_location_five3])
        while vehicle.get_location().distance(line_vice_end_location_five1) > 15:
            sleep(0.01)
        vice_control.vices += vices
        while True:
            distance1 = vehicle.get_location().distance(line_vice_end_location_five3)
            distance2 = vices[-1].get_location().distance(line_vice_end_location_five3)
            if distance1 > distance2 + 50:
                destroy_all_vehicles_traffics(vehicle)
                break
            sleep(0.01)
        while vehicle.get_location().distance(main_car_location) > 10:
            sleep(0.01)
        destroy_all_vehicles_traffics(vehicle)
    while True:
        sleep(1)
        # print(vehicle.get_location())
