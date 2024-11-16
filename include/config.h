#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID ""  // your wifi ssid
#define WIFI_PASSWORD ""  // your wifi password

// MQTT Configuration
#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASS ""
#define MQTT_TOPIC "ESP32Soil/soil_sensors"

// Pin Definitions for ESP32 DOIT DevKit V1
#define RS485RX GPIO_NUM_16  // Can use GPIO16 (RX2)
#define RS485TX GPIO_NUM_17  // Can use GPIO17 (TX2)

// I2C Pins for OLED Display
#define I2C_SDA GPIO_NUM_21  // Default I2C SDA pin
#define I2C_SCL GPIO_NUM_22  // Default I2C SCL pin

// LED Pin
#define LED_BUILTIN GPIO_NUM_2  // Built-in LED on DOIT DevKit V1

// Sensor Configuration
#define SENSOR_FRAME_SIZE 19
#define SENSOR_WAITING_TIME 1000
#define SENSOR_ID 0x01
#define SENSOR_FUNCTION 0x03
#define SENSOR_BYTE_RESPONSE 0x0E

// Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Task Configuration
#define WDT_TIMEOUT 30
#define TASK_RESET_THRESHOLD 3
#define WIFI_TIMEOUT_MS 10000
#define TASK_STACK_SIZE 4096
#define SENSOR_READ_INTERVAL 10000  // 10 seconds
#define DISPLAY_UPDATE_INTERVAL 100
#define MQTT_UPDATE_INTERVAL 10000
#define MONITOR_CHECK_INTERVAL 5000

// Task Core Assignments
#define CORE_0 0  // Protocol CPU
#define CORE_1 1  // Application CPU

// Task Priorities
#define PRIORITY_LOW 1
#define PRIORITY_MEDIUM 2
#define PRIORITY_HIGH 3

// Queue Configuration
#define QUEUE_TIMEOUT_MS 1000

// Debug Configuration
#define SERIAL_DEBUG_BAUD 115200

#endif