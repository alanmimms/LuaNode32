# BLE scan results send to UDP server

In this example, ESP32 scan BLE devices and then send the scan results to UDP server. User can 
setup SSID, password, server IP and server port via uart. Run a uart debug tool, the following 
commands can send to ESP32 via uart0.

* ssid: after input `ssid`, ESP32 will require user to input AP's SSID which ESP32 will connect to
* pass: after input `pass`, ESP32 will require user to input AP's password
* ip: the UDP server's IP
* port: the UDP server's port
* help: show help info
* wifi: start BLE scanning and then send scan results via WiFi. Before input this command, you have to setup `ssid`, `pass`, `ip` and `port`.

To test this example, you can use UDP debug tool to create a UDP server, and then run this example on 
ESP32, it will send the scan result to the server.
