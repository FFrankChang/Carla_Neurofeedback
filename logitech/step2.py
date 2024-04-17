import time
import math
import socketio
from time import sleep



class SocketIOClient:
    def __init__(self, server_url):
        self.server_url = server_url

        self.sio = socketio.Client()  # 初始化实例
        self.response = None  # node的响应值

        self.sio.on('connect', self.on_connect)  # connect与node中保持一致
        self.sio.on('disconnect', self.on_disconnect)  # disconnect与node中保持一致
        self.sio.on('response', self.on_response)  # response与node中保持一致

        self.sio.connect(self.server_url)  # 连接服务器,必须放在注册函数之后，不然connect，disconnect不起作用

    def on_connect(self):  # 连接前处理函数
        print('Connected to server')

    def on_disconnect(self):  # 断开连接后处理的函数
        print('Disconnected from server')

    def on_response(self, data):  # 接受node的返回值
        self.response = data
        print(f"node返回值:{self.response}")

    def send(self, value):
        # value = json.dumps({"steering": value})
        self.sio.emit('data', value)  # 这里data与node中data保持一致
        sleep(0.01)


if __name__ == "__main__":
    client = SocketIOClient('http://localhost:3000')
    while True:
        #value = 10 * math.sin(time.time()) + 50
        client.send(50)  # 所传的的值范围在0-100,0→最右，100→最左
