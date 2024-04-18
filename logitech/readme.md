```
        if (devices[i].vendorId === 1133 &&
            (devices[i].productId === 49766 || devices[i].product === 'G29 Driving Force Racing Wheel') &&
            (devices[i].interface === 0 || devices[i].usagePage === 1)) {
            devicePath = devices[i].path
            break
        }
```

logitech\node_modules\logitech-g29\code\index.js
修改id