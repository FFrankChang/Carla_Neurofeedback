import carla

main_car_location = carla.Location(x=-3.513154, y=1.750000)

curve_start_one_location = carla.Location(x=396.676880, y=1.750000)  # 弯道起点
curve_end_one_location = carla.Location(x=405.63, y=27.54)  # 弯道终点
curve_start_two_location = carla.Location(x=291.39, y=27.47)  # 弯道二起点
curve_end_two_location = carla.Location(x=291.39, y=60.52)  # 弯道二终点
curve_start_three_location = carla.Location(x=404.12, y=60.51)  # 弯道三起点
curve_end_three_location = carla.Location(x=409.66, y=85.77)  # 弯道三终点
curve_start_four_location = carla.Location(x=297.13, y=86.06)  # 弯道四起点
curve_end_four_location = carla.Location(x=297.13, y=118.73)  # 弯道四终点

line_vice_location_one = carla.Location(x=408.84, y=31.45)  # 副车一坐标
line_vice_location_two = carla.Location(x=291.39, y=56.43)  # 弯道二副车生成坐标

line_vice_location_four1 = carla.Location(x=278.12, y=97.18)  # 弯道处的车一
line_vice_location_four2 = carla.Location(x=278.12, y=103.36)  # 弯道处的车二
line_vice_end_location_four = carla.Location(x=295.77, y=112.71)  # 弯道四终点

line_vice_end_location_five1 = carla.Location(x=404.29, y=118.94)  # 弯道五起点
line_vice_end_location_five2 = carla.Location(x=422.28, y=129.43)  # 弯道五中点
line_vice_end_location_five3 = carla.Location(x=404.29, y=144.58)  # 弯道五终点
