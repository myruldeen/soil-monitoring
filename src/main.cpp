#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <HardwareSerial.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "types.h"

// Task handles
TaskHandle_t taskSensorHandle = nullptr;
TaskHandle_t taskDisplayHandle = nullptr;
TaskHandle_t taskMQTTHandle = nullptr;
TaskHandle_t taskMonitorHandle = nullptr;

// Queue handles
QueueHandle_t sensorQueue = nullptr;
// Constants
static const int QUEUE_SIZE = 5;

// Semaphore for I2C bus access
SemaphoreHandle_t i2cSemaphore = nullptr;

// Task monitoring flags
volatile bool sensorTaskAlive = false;
volatile bool displayTaskAlive = false;
volatile bool mqttTaskAlive = false;

// Error counters
volatile int sensorErrors = 0;
volatile int displayErrors = 0;
volatile int mqttErrors = 0;

// Object instances
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial sensor(2);
//SoftwareSerial sensor(RS485RX, RS485TX);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensor communication
unsigned char byteRequest[8] = {0X01, 0X03, 0X00, 0X00, 0X00, 0X07, 0X04, 0X08};
unsigned char byteResponse[19] = {};

// Function declarations
void TaskMonitor(void *parameter);
void TaskReadSensors(void *parameter);
void TaskUpdateDisplay(void *parameter);
void TaskMQTT(void *parameter);
bool checkWiFiConnection();
void reconnectMQTT();
int sensorValue(int x, int y);
void mqtt_callback(char* topic, byte* payload, unsigned int length);

// Helper function for sensor value calculation
int sensorValue(int x, int y) {
    return (x * 256) + y;
}

// Function to check and reconnect WiFi
bool checkWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost. Reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && 
               millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
            delay(100);
        }
        return WiFi.status() == WL_CONNECTED;
    }
    return true;
}

// MQTT callback
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

// MQTT reconnect function
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
            Serial.println("connected");
            client.subscribe(MQTT_TOPIC);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 5 seconds");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

// Task to monitor other tasks
void TaskMonitor(void *parameter) {
    esp_task_wdt_add(NULL);
    
    while (1) {
        esp_task_wdt_reset();
        
        // Check task health
        if (!sensorTaskAlive) {
            sensorErrors++;
            Serial.println("Sensor task may be stuck!");
            if (sensorErrors >= TASK_RESET_THRESHOLD) {
                Serial.println("Restarting sensor task...");
                if (taskSensorHandle != NULL) {
                    vTaskDelete(taskSensorHandle);
                    xTaskCreatePinnedToCore(TaskReadSensors, "ReadSensors", 4096, NULL, 1, &taskSensorHandle, 0);
                    sensorErrors = 0;
                }
            }
        }
        
        if (!displayTaskAlive) {
            displayErrors++;
            Serial.println("Display task may be stuck!");
            if (displayErrors >= TASK_RESET_THRESHOLD) {
                Serial.println("Restarting display task...");
                if (taskDisplayHandle != NULL) {
                    vTaskDelete(taskDisplayHandle);
                    xTaskCreatePinnedToCore(TaskUpdateDisplay, "UpdateDisplay", 4096, NULL, 1, &taskDisplayHandle, 1);
                    displayErrors = 0;
                }
            }
        }
        
        if (!mqttTaskAlive) {
            mqttErrors++;
            Serial.println("MQTT task may be stuck!");
            if (mqttErrors >= TASK_RESET_THRESHOLD) {
                Serial.println("Restarting MQTT task...");
                if (taskMQTTHandle != NULL) {
                    vTaskDelete(taskMQTTHandle);
                    xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 4096, NULL, 1, &taskMQTTHandle, 1);
                    mqttErrors = 0;
                }
            }
        }
        
        // Check WiFi connection
        if (!checkWiFiConnection()) {
            Serial.println("WiFi connection failed, restarting ESP32...");
            ESP.restart();
        }
        
        // Reset alive flags
        sensorTaskAlive = false;
        displayTaskAlive = false;
        mqttTaskAlive = false;
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Task to read sensors
void TaskReadSensors(void *parameter) {
    esp_task_wdt_add(NULL);
    SensorData data;
    
    while (1) {
        esp_task_wdt_reset();
        sensorTaskAlive = true;
        
        // Read RS485 sensor with timeout and error handling
        sensor.write(byteRequest, 8);
        unsigned long startTime = millis();
        bool rs485Success = false;
        
        while (millis() - startTime < SENSOR_WAITING_TIME) {
            esp_task_wdt_reset();
            
            if (sensor.available() >= SENSOR_FRAME_SIZE) {
                for (int n = 0; n < SENSOR_FRAME_SIZE; n++) {
                    byteResponse[n] = sensor.read();
                }
                
                if (byteResponse[0] == SENSOR_ID && 
                    byteResponse[1] == SENSOR_FUNCTION && 
                    byteResponse[2] == SENSOR_BYTE_RESPONSE) {
                    
                    data.moisture = sensorValue((int)byteResponse[3], (int)byteResponse[4]) * 0.1;
                    data.temperature = sensorValue((int)byteResponse[5], (int)byteResponse[6]) * 0.1;
                    data.ec = sensorValue((int)byteResponse[7], (int)byteResponse[8]);
                    data.ph = sensorValue((int)byteResponse[9], (int)byteResponse[10]) * 0.1;
                    data.nitrogen = sensorValue((int)byteResponse[11], (int)byteResponse[12]);
                    data.phosphorus = sensorValue((int)byteResponse[13], (int)byteResponse[14]);
                    data.potassium = sensorValue((int)byteResponse[15], (int)byteResponse[16]);
                    rs485Success = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (!rs485Success) {
            Serial.println("Failed to read RS485 sensor");
        } else {
            if (xQueueSend(sensorQueue, &data, 0) != pdTRUE) {
                Serial.println("Failed to send data to queue");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Task to update display
void TaskUpdateDisplay(void *parameter) {
    esp_task_wdt_add(NULL);
    SensorData data;
    
    while (1) {
        esp_task_wdt_reset();
        displayTaskAlive = true;
        
        if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                bool displaySuccess = false;
                int retryCount = 0;
                const int maxRetries = 3;
                
                while (!displaySuccess && retryCount < maxRetries) {
                    // First screen
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(SSD1306_WHITE);
                    
                    display.setCursor(0, 0);
                    display.print("Soil Data:");
                    
                    display.setCursor(0, 10);
                    display.print("Temp: " + String(data.temperature) + " C");
                    
                    display.setCursor(0, 20);
                    display.print("Moisture: " + String(data.moisture) + " %");
                    
                    display.setCursor(0, 30);
                    display.print("EC: " + String(data.ec) + " uS/cm");
                    
                    display.setCursor(0, 40);
                    display.print("pH: " + String(data.ph));
                    
                    // Update display
                    display.display();
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    
                    // Second screen
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.print("NPK Values:");
                    
                    display.setCursor(0, 10);
                    display.print("N: " + String(data.nitrogen) + " mg/kg");
                    
                    display.setCursor(0, 20);
                    display.print("P: " + String(data.phosphorus) + " mg/kg");
                    
                    display.setCursor(0, 30);
                    display.print("K: " + String(data.potassium) + " mg/kg");
                    
                    // Update display
                    display.display();
                }
                
                xSemaphoreGive(i2cSemaphore);
                
                if (!displaySuccess) {
                    Serial.println("Display update failed after max retries");
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to handle MQTT communications
void TaskMQTT(void *parameter) {
    esp_task_wdt_add(NULL);
    SensorData data;
    
    while (1) {
        esp_task_wdt_reset();
        mqttTaskAlive = true;
        
        if (!client.connected()) {
            reconnectMQTT();
        }
        
        client.loop();
        
        if (xQueuePeek(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            StaticJsonBuffer<300> JSONbuffer;
            JsonObject &root = JSONbuffer.createObject();
            
            root["m"] = data.moisture;
            root["t"] = data.temperature;
            root["ec"] = data.ec;
            root["ph"] = data.ph;
            root["n"] = data.nitrogen;
            root["p"] = data.phosphorus;
            root["k"] = data.potassium;
            
            char JSONmessageBuffer[200];
            root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
            
            int retryCount = 0;
            const int maxRetries = 3;
            bool publishSuccess = false;
            
            while (!publishSuccess && retryCount < maxRetries) {
                if (client.publish("ESP32Soil/soil_sensors", JSONmessageBuffer)) {
                    publishSuccess = true;
                    Serial.println("Success sending message");
                } else {
                    retryCount++;
                    Serial.println("Error sending message, retrying...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);
    
    // Initialize watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT, true);
    
    // Initialize hardware
    sensor.begin(4800, SERIAL_8N1, RS485RX, RS485TX);
    Wire.begin();
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("Error initializing display");
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    
    // Setup WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    
    // Setup MQTT
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqtt_callback);
    
    // Create queue and semaphore
    sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(SensorData));
    i2cSemaphore = xSemaphoreCreateMutex();
    
    // Create tasks
    xTaskCreatePinnedToCore(
        TaskMonitor,
        "Monitor",
        TASK_STACK_SIZE,
        NULL,
        2,
        &taskMonitorHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        TaskReadSensors,
        "ReadSensors",
        TASK_STACK_SIZE,
        NULL,
        1,
        &taskSensorHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        TaskUpdateDisplay,
        "UpdateDisplay",
        TASK_STACK_SIZE,
        NULL,
        1,
        &taskDisplayHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        TaskMQTT,
        "MQTT",
        TASK_STACK_SIZE,
        NULL,
        1,
        &taskMQTTHandle,
        1
    );
}

void loop() {
    vTaskDelete(NULL);
}