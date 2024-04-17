const express = require('express');
const http = require('http');
const socketIO = require('socket.io');

const g = require('logitech-g29')
var connected = true;
var options = {
    autocenter: false, // set to false so the wheel will not fight itself when we rotate it
    debug: false,
    range: 900
}
var wheel = {
    currentPos: 0, // initial value does not matter
    moveToPos: 0,  // initial value does not matter
    moved: true
}

const turn_wheel = (set_point) => {
    if (connected) {
        connected = false
        g.connect(options, function (err) {
            if (err) {
                console.log('Oops -> ' + err)
            }
            g.forceFriction(0.5) // 方向盘摩擦力
            g.on('wheel-turn', function (val) {
                wheel.currentPos = val;
            })
        })
    }

    var error = set_point - wheel.currentPos;
    var p = 0.1;
    var u = p * error; // u may range from -0.5 to 0.5
    var u_clipped = Math.max(-0.5, Math.min(0.5, u));
    g.forceConstant(0.5 + u_clipped);
}


const startServer = () => {
    const app = express();
    const server = http.createServer(app);
    const io = socketIO(server);

    const PORT = 3000;
    io.on('connect', (socket) => {
        console.log('Client connected');

        // 监听来自Python客户端的数据
        socket.on('data', (data) => {
            turn_wheel(data)
            // 在这里可以对接收到的数据进行处理
            // 这里只是简单地发送一个字符串给Python客户端
            // socket.emit('response', 'Data received by Node.js');
        });

        socket.on('disconnect', () => {
            console.log('Client disconnected');
            connected = true

            g.disconnect()

            options.autocenter = true
            g.connect(options)
            // 等待 10毫秒，保证方向盘回中
            setTimeout(() => {
                options.autocenter=false
                g.disconnect()
            }, 100);
        });
    });

    server.listen(PORT, () => {
        console.log(`Server is running on http://localhost:${PORT}`);
    });
}

startServer(); // 调用函数启动服务器