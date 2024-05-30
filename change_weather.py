import pygame
import re
import carla

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

class WeatherController:
    def __init__(self, world):
        self.world = world
        self.weather_options = find_weather_presets()
        self.current_weather = 0  
    def toggle_weather(self):
        self.current_weather = (self.current_weather + 1) % len(self.weather_options)
        self.world.set_weather(self.weather_options[self.current_weather][0])
        print(f"Weather changed to {self.weather_options[self.current_weather][1]}")

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Weather Controller")
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    weather_controller.toggle_weather()
                    
        pygame.display.flip()
    pygame.quit()

if __name__ == '__main__':
    # 创建CARLA世界连接，并初始化WeatherController对象
    client = carla.Client("localhost", 2000)  # 根据需要调整服务器地址和端口
    client.set_timeout(10.0)
    world = client.get_world()
    
    weather_controller = WeatherController(world)
    main()
