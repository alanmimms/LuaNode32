## Introduction

![github](https://github.com/Nicholas3388/LuaNode/raw/master/images/ota_tcp.jpg "OTA TCP update")

Although there is an OTA example provided by esp-idf, the new firmware download from http server, I found
that some times the connection is failed when firmware updating. In this sample, we provide a TCP method to 
update firmware via TCP server. If a package is sent failed, the client (ESP32) will request the server to
send package again.

* The request sent by client will have a string format like this: `req0\r\n\r\n` or `req1\r\n\r\n`, where "req0" 
  means client request server send next package, "req1" means there are some errors occur when client receive
  package, request server to send previous package again.

* The package sent by server will have the following format: `total: 123456\r\nlength: 321\r\n\r\n$#&*@#$......`, 
  where "total: 123456\r\nlength: 321\r\n\r\n" is package header, there are two key and value. 
  the key "total" refers to the total length of this firmware, the other key "length" refers to current package 
  length. With these information, we can show update progress to user. The package header end with "\r\n\r\n", 
  and then follow by firmware data. 512 bytes of firmware data for each package.

## How to use

* Modify the SSID and password in `user_config.h` in client source. Then build client and 
  flash the generated firmware to your ESP32 device

* Store your new firmware, which used for updating and the `ota_server.py` on your laptop, 
  the two file must in the same folder, 
  then run the server on your laptop (You must install python on you laptop first), note that 
  the laptop and ESP32 must connect to the same AP.

* when ESP32 powerup, it will connect to the python server run on your laptop and download 
  the new firmware.

You can run the `ota_server.py` on remote server and store your new firmware on the remote 
server for updating. Just change the server IP and PORT defined in `user_config.h`
