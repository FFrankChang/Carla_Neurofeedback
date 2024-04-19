import carla
from disposition import *

number = 5  # 车流的车辆数，指的是前车有多少辆，其他的都有
road_speed_limit = {
    -1: 70,
    -2: 70,
    -3: 70,
    -4: 70,
    -5: 70,
    -6: 70,
    1: 70,
    2: 70,
    3: 70,
    4: 70,
    5: 70,
    6: 70,
}
easy_location8 = carla.Location(x=8106.258789, y=3001.847168, z=15.142560)
easy_location1 = carla.Location(x=5606.206055, y=3001.875000, z=15.142560)
interfere_one_location1 = carla.Location(x=3106.206055, y=3001.875000, z=15.413071)
interfere_two_location1 = carla.Location(x=606.206055, y=3001.875000, z=15.413071)
easy_location2 = carla.Location(x=-1893.793945, y=3001.8750, z=15.413071)
interfere_one_location2 = carla.Location(x=-4393.794922, y=3001.8750, z=15.413071)
end_location1 = carla.Location(x=-6893.794922, y=3001.875, z=15.413071)
easy_location3 = carla.Location(x=-7801.875000, y=1905.705078, z=15.413071)
interfere_two_location2 = carla.Location(x=-7801.875000, y=-594.294922, z=15.413071)
end_location2 = carla.Location(x=-7801.712891, y=-3094.3090, z=15.413071)
easy_location4 = carla.Location(x=-6908.300781, y=-3998.125000, z=15.413071)
easy_location5 = carla.Location(x=-4408.300781, y=-3998.125000, z=15.4)
interfere_two_location3 = carla.Location(x=-1908.300781, y=-3998.125000, z=15.4)
easy_location6 = carla.Location(x=591.699219, y=-3998.125000, z=15.4)
interfere_two_location4 = carla.Location(x=3091.699219, y=-3998.125000, z=15.4)
interfere_one_location3 = carla.Location(x=5591.699219, y=-3998.125000, z=15.4)
end_location3 = carla.Location(x=8091.694336, y=-3998.151611, z=15.4)
easy_location7 = carla.Location(x=8798.125000, y=-3107.863037, z=15.4)
interfere_two_location5 = carla.Location(x=8798.125000, y=-607.863037, z=15.4)
end_location4 = carla.Location(x=8798.125000, y=1892.136719, z=15.1)
print(env_map.get_waypoint(interfere_two_location5).next(2500)[0])
