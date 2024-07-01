import time

from s02_vehicle_method import *
from s02_config import *


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
    destroy_all_vehicles_traffics()
    vehicle = create_actor(main_car_location, model="vehicle.lincoln.mkz_2020")  # 创建主车
    vehicle_control = Vehicle_Control(vehicle)  # 主车控制类

    Window(vehicle)  # 窗口

    # 主车设置
    # vehicle_control.autopilot_flag=False
    vehicle_control.autopilot_speed_limit = 30
    vehicle_control.labour_speed_limit = 30
    vehicle_control.labour_low_speed_limit = 30
    vehicle_control.follow_lane()

    for i in range(5):
        vehicle_control.autopilot_speed_limit = 30
        vice_control = Vice_Control()  # 副车控制类
        static_vice_car = []  # 静止车辆列表
        static_vice_car += [vehicle]
        # 副车设置
        vice_control.follow_road()
        vice_control.speed_limit = 15
        print(f"第{i + 1}次仿真")
        # 场景一
        static_car1 = create_actor(
            env_map.get_waypoint(curve_start_one_location).previous(20)[0].transform.location)
        while vehicle.get_location().distance(main_car_location) < 300:
            sleep(0.01)
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
        print("================",vice_control.vices)
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
        vehicle_control.autopilot_speed_limit = 20
    while True:
        sleep(1)
        # print(vehicle.get_location())
