import carla
import pygame
from pygame.locals import *
import math
import random
import time
import numpy as np
from time import sleep
import os
import keyboard
import threading
from concurrent.futures import ThreadPoolExecutor
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.controller import VehiclePIDController
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.tools.misc import get_speed

client = carla.Client("127.0.0.1", 2000)  # 连接carla
client.set_timeout(60)  # 设置超时
world = client.get_world()  # 获取世界对象
env_map = world.get_map()  # 获取地图对象
spectator = world.get_spectator()  # ue4中观察者对象
blueprint_library = world.get_blueprint_library()  # 获取蓝图，可以拿到可创建的对象
vehicle_points = env_map.get_spawn_points()  # 拿到世界中所有可绘制车辆的点坐标
# inside_junction = env_map.get_waypoint().is_junction()  # 获取路点的车道信息
vehicle_models = blueprint_library.filter('*mini*')  # 拿到所有可绘制的车辆模型
prop_model = blueprint_library.filter('*prop*')  # 拿到所有可绘制的交通标志模型

pygame.init()
pygame.mixer.init()
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
except Exception as e:
    print(f"没有外接方向盘{e}")

args_lateral_dict = {'K_P': 1.95, 'K_D': 0.2, 'K_I': 0.07, 'dt': 1.0 / 10.0}
args_long_dict = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 10.0}
