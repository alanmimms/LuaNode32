## How to use

In the function `alexa_init` function defined in `alexa.c`, you can call `create_new_device` 
to add a new device to be controlled by Alexa. As for the parameters of `create_new_device`, 
the first parameter is the device name, the second one is the TCP server port, each device need 
a TCP server to communicate with Alexa Echo device, the third one is a callback when user ask "Alexa, 
turn on `device_name`", the last one is a callback when user ask "Alexa, turn off `device_name`", where 
`device_name` refer to the first parameter.


In this app, when you ask Alexa device "Alexa, turn on receptacle", it will require ESP32 to invoke 
the function `turn_on_relay` defined in `alexa.c`. When you ask "Alexa, turn off backlight", Alexa 
will require ESP32 to invoke funtion named `turn_off_backlight`.
