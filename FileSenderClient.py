import socket
import os
import tkinter as tk
from tkinter import filedialog

def send_file(server_ip, server_port, file_path):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((server_ip, server_port))
            file_size = os.path.getsize(file_path)
            file_name = os.path.basename(file_path)

            s.sendall(str(file_size).encode())
            s.recv(1024)
            s.sendall(file_name.encode())
            s.recv(1024)

            with open(file_path, 'rb') as f:
                while True:
                    data = f.read(4096)
                    if not data:
                        break
                    s.sendall(data)
            print(f"Sent file: {file_name}")
    except Exception as e:
        print(f"Error: {e}")

def select_files_and_send(server_ip, server_port):
    root = tk.Tk()
    root.withdraw()  
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, "data")
    initial_dir = data_dir if os.path.exists(data_dir) else script_dir  
    file_paths = filedialog.askopenfilenames(initialdir=initial_dir)  
    if file_paths:
        for file_path in file_paths:
            send_file(server_ip, server_port, file_path)

if __name__ == "__main__":
    server_ip = "192.168.3.9"
    server_port = 7999
    select_files_and_send(server_ip, server_port)
