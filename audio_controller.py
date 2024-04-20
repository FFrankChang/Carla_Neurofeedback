import socket
import keyboard

def main():
    host = '192.168.3.9'  # 接收端的IP地址，如果在同一台机器上运行接收端和发送端，则使用 localhost
    port = 12345  # 接收端正在监听的端口号
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("按 'P' 播放音乐, 按 'S' 暂停音乐, 按 'E' 停止音乐并退出程序.")
    
    try:
        while True:
            if keyboard.is_pressed('p'):  # 按 P 键播放
                client_socket.sendto(b'play', (host, port))
                print("发送播放命令")
                while keyboard.is_pressed('p'):
                    pass  # 等待按键释放

            if keyboard.is_pressed('s'):  # 按 S 键暂停
                client_socket.sendto(b'pause', (host, port))
                print("发送暂停命令")
                while keyboard.is_pressed('s'):
                    pass  # 等待按键释放

            if keyboard.is_pressed('e'):  # 按 E 键停止并退出
                client_socket.sendto(b'stop', (host, port))
                print("发送停止命令")
                break  # 退出循环

    finally:
        client_socket.close()
        print("程序已停止")

if __name__ == '__main__':
    main()
