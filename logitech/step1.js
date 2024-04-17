const express = require('express');
const http = require('http');
const socketIO = require('socket.io');

const startServer = () => {
  const app = express();
  const server = http.createServer(app);
  const io = socketIO(server);

  const PORT = 3000;

  io.on('connect', (socket) => {
    console.log('Client connected');

    // 监听来自Python客户端的数据
    socket.on('data', (data) => {
      console.log('接受值:', data);
      // 在这里可以对接收到的数据进行处理
      // 这里只是简单地发送一个字符串给Python客户端
      socket.emit('response', 'Data received by Node.js');
    });

    socket.on('disconnect', () => {
      console.log('Client disconnected');
    });
  });

  server.listen(PORT, () => {
    console.log(`Server is running on http://localhost:${PORT}`);
  });
}

startServer(); // 调用函数启动服务器

