import carla
number = 3  # 车流的车辆数，指的是前车有多少辆，其他的都有
road_speed_limit = {
    2: 110,
    3: 100,
    4: 90,
    5: 100,
    6: 110,
}

start_location = carla.Location(x=6556.337891, y=3001.875000, z=15.142560)  # 起始点坐标
start_end_location = carla.Location(x=4056.337891, y=3001.875000, z=15.413071)  # 简单场景一末坐标
