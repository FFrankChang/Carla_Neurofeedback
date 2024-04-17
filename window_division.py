import os
import carla
import random
import time
import numpy as np

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()


class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.size = 18
        self.fps = 60  # 帧率

        self.font = pygame.font.Font(r"TTF\宋体.ttf", self.size)  # 初始化字体对象
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def draw_text(self, word, length, position, color=(255, 0, 0)):
        text = self.font.render(word.ljust(length), True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.display.blit(text, text_rect)

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0] / self.grid_size[1]), int(self.window_size[1] / self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return
        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None


class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, reverse):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.time_processing = 0.0
        self.tics_processing = 0
        self.display_man.add_sensor(self)
        self.reverse = reverse

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))
            camera_bp.set_attribute('fov', str(40))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])
            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)
            return camera
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        if self.reverse == True:
            array = np.flip(array, axis=1)  # 将画面左右翻转
        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


def window(world, vehicle):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    display_manager = DisplayManager(grid_size=[2, 3], window_size=[1800, 1000])  # 1280, 720
    SensorManager(world, display_manager, 'RGBCamera',
                  carla.Transform(carla.Location(x=3.5, y=0, z=1.5), carla.Rotation(yaw=-40)),
                  vehicle, {}, display_pos=[0, 0], reverse=False)
    SensorManager(world, display_manager, 'RGBCamera',
                  carla.Transform(carla.Location(x=3.5, y=0, z=1.5), carla.Rotation(yaw=+00)),
                  vehicle, {}, display_pos=[0, 1], reverse=False)
    SensorManager(world, display_manager, 'RGBCamera',
                  carla.Transform(carla.Location(x=3.5, y=0, z=1.5), carla.Rotation(yaw=+40)),
                  vehicle, {}, display_pos=[0, 2], reverse=False)
    SensorManager(world, display_manager, 'RGBCamera',
                  carla.Transform(carla.Location(x=0.7, y=-1.2, z=1.1), carla.Rotation(yaw=-170)),
                  vehicle, {}, display_pos=[1, 0], reverse=True)
    SensorManager(world, display_manager, 'RGBCamera',
                  carla.Transform(carla.Location(x=-4, y=0, z=1.3), carla.Rotation(yaw=-180)),
                  vehicle, {}, display_pos=[1, 1], reverse=True)
    SensorManager(world, display_manager, 'RGBCamera',
                  carla.Transform(carla.Location(x=0.7, y=+1.2, z=1.1), carla.Rotation(yaw=+170)),
                  vehicle, {}, display_pos=[1, 2], reverse=True)
    # #######################################################
    while True:
        # world.tick()
        display_manager.render()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                os._exit(0)
        display_manager.draw_text("我爱你", 20, (0, 40))
        display_manager.clock.tick(display_manager.fps)


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    bp = world.get_blueprint_library().filter('charger_2020')[0]
    # print(random.choice(world.get_map().get_spawn_points()))
    start_location = carla.Location(x=-6998.000000, y=-3990.625000, z=19)
    start_transform = carla.Transform(start_location, carla.Rotation())
    vehicle = world.try_spawn_actor(bp, start_transform)
    vehicle.set_autopilot(True)

    window(world, vehicle)


if __name__ == '__main__':
    main()
