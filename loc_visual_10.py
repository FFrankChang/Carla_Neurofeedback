import socket
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.image as mpimg
import numpy as np

# UDP IP and port
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# Create socket and bind
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1)  # Set timeout to avoid indefinite blocking on recvfrom

fig, ax = plt.subplots()

# Load images for vehicles
traffic_img = mpimg.imread(r'C:\Users\Lenovo\Desktop\tongji_shi\Carla_Neurofeedback\resource\car1.png')  
main_vehicle_img = mpimg.imread(r'C:\Users\Lenovo\Desktop\tongji_shi\Carla_Neurofeedback\resource\car2.png')  

# Initial scatter object setup with dummy data
traffic_positions = [(0, 0)]
main_vehicle_position = [(0, 0)]

# Initial imshow objects, hidden off plot
traffic_icons = [ax.imshow(traffic_img, extent=(0, 1, 0, 1), visible=False) for _ in range(100)] 
main_vehicle_icon = ax.imshow(main_vehicle_img, extent=(0, 1, 0, 1), visible=False)

ax.set_xlim(0, 100)
ax.set_ylim(0, 30)
ax.set_aspect('equal', 'box')

# Invert y-axis to set origin to top-left corner
ax.invert_yaxis()

# Drawing horizontal lines at specified locations
for x in range(0, 31, 5):
    ax.hlines(x, 0, 3000, colors='gray', linestyles='dashed', linewidth=0.5)

def update(frame):
    global traffic_positions, main_vehicle_position
    ax.set_aspect('equal', 'box')

    try:
        data, addr = sock.recvfrom(16000)  # Receive data
        if not data:
            return []  # If no data, do nothing

        locations = json.loads(data.decode())  # Parse JSON data

        # Update main vehicle position
        main_vehicle_position = (locations['main_vehicle']['loc']['x'], locations['main_vehicle']['loc']['y'])
        main_vehicle_icon.set_visible(False)
        main_vehicle_icon.set_extent((main_vehicle_position[0]-4, main_vehicle_position[0]+4, main_vehicle_position[1]-2, main_vehicle_position[1]+2))
        main_vehicle_icon.set_visible(True)

        # Update traffic vehicle positions
        traffic_positions = [(vehicle['loc']['x'], vehicle['loc']['y']) for vehicle in locations['traffic']]
        for icon, position in zip(traffic_icons, traffic_positions):
            icon.set_extent((position[0]-4, position[0]+4, position[1]-2, position[1]+2))
            icon.set_visible(True)

        ax.set_xlim(main_vehicle_position[0] - 20, main_vehicle_position[0] + 150)

    except socket.timeout:
        # If no data is received within the timeout period, continue without update
        return [main_vehicle_icon] + traffic_icons

    except Exception as e:
        # Catch any other unexpected exceptions
        print(f"Error: {e}")
        return [main_vehicle_icon] + traffic_icons

    return [main_vehicle_icon] + traffic_icons

# Create animation
ani = FuncAnimation(fig, update, blit=False, interval=100)

# Graceful window closure handling
try:
    plt.show()
except KeyboardInterrupt:
    print("Window closed. Exiting program.")
finally:
    sock.close()  # Close the socket connection when the program ends
