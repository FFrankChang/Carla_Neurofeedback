import numpy as np
import random
import matplotlib.pyplot as plt

def generate_random_locations_around_vehicle(base_location, num_vehicles=100, x_range=(-100, 100), y_range=(-50, 50), z=5, min_distance=7, safe_zone_radius=10):
    random_locations = []
    base_x, base_y, base_z = base_location
    
    x_values = np.linspace(x_range[0], x_range[1], num=200)  
    weights = np.linspace(1, 10, num=200) 
    
    while len(random_locations) < num_vehicles:
        random_x = np.random.choice(x_values, p=weights/weights.sum())
        random_y = base_y + random.uniform(*y_range)

        valid_location = True

        if ((base_x - random_x) ** 2 + (base_y - random_y) ** 2) ** 0.5 < safe_zone_radius:
            continue
        
        for location in random_locations:
            if ((location[0] - random_x) ** 2 + (location[1] - random_y) ** 2) ** 0.5 < min_distance:
                valid_location = False
                break
        
        if valid_location:
            random_locations.append((random_x, random_y, z))  # 使用元组来表示位置
    
    return random_locations


easy_location1 = (100, 13, 5)

random_traffic_points = generate_random_locations_around_vehicle(
        easy_location1, 
        num_vehicles=75, 
        x_range=(100, 900),  
        y_range=(-12.5, 12.5),    
        z=3        
    )

x_coords = [point[0] for point in random_traffic_points]
y_coords = [point[1] for point in random_traffic_points]

# 绘制图形
plt.figure(figsize=(8, 6))
plt.scatter(x_coords, y_coords, c='grey', label='Vehicles', alpha=0.6)
plt.scatter(easy_location1[0], easy_location1[1], c='red', label='Base Location', marker='X', s=100)

# 设置图表标题和标签
plt.title('Generated Vehicle Locations Around Base Location')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()

# 显示图形
plt.grid(True)
plt.show()
