idf_component_register(SRCS "node.c" "file.c" "gpio.c" "utils.c" "tmr.c"
					REQUIRES lua utils spiffs driver esp_timer esp_event
					INCLUDE_DIRS "include")
