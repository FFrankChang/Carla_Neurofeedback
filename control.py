import socket
import subprocess
import datetime
import csv
import time

# Setup UDP server
UDP_IP = "0.0.0.0"  # Local IP
UDP_PORT = 5005       # Port number
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Create a socket for sending messages
# send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# send_ip = "127.0.0.1"
# send_port = 12347

process = None

# Script paths configuration, can be adjusted as needed
scripts = {
    "1": "E:\\Frank_Projects\\Carla_Neurofeedback_Frank\\S07_main.py",
    "2": "E:\\Frank_Projects\\Carla_Neurofeedback_Frank\\S06_main.py",
    "3": "E:\\Frank_Projects\\Carla_Neurofeedback_Frank\\s02_main.py"
}

# CSV file to log script events
csv_file_path = 's01_script_events.csv'
# Duration to run script 1 (seconds)
RUN_TIME = 600  # 10 minutes

try:
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        message = data.decode()
        print(f"Received message: {message}")

        # Parse the received message to extract number and additional data if present
        parts = message.split(" | ")
        command = parts[0].strip()

        if command in ["1","2", "3"]:
            # Expected message format for 2 and 3 is: "2 | Condition: feedback | Subject: S01 | Day: D01"
            condition = parts[1].split(": ")[1].strip()
            subject = parts[2].split(": ")[1].strip()
            day = parts[3].split(": ")[1].strip()

        if command in scripts:
            script_path = scripts[command]
            if process is None:  # Check if a process is already running
                # Start the script with additional parameters for script 2 and 3
                if command in ["2", "3"]:
                    print(2)
                    process = subprocess.Popen(["python", script_path, subject, day, condition])
                else:
                    print(3)
                    process = subprocess.Popen(["python", script_path])
                
                start_time = time.time()
                print(f"Started script {script_path} with parameters {subject}, {day}, {condition}.")

            else:
                print("Another script is already running.")
        elif command == "0":
            if process is not None:
                process.terminate()  # Terminate process
                print("Terminated the script.")
                if process.args[1] == scripts["1"]:
                    with open(csv_file_path, 'a', newline='') as csvfile:
                        csvwriter = csv.writer(csvfile)
                        csvwriter.writerow(['stop_script_1', datetime.datetime.now()])
                process = None

            else:
                print("No script is running.")
finally:
    sock.close()
