#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <BH1750.h>
#include <AHTxx.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <millisDelay.h>

#define RS485RX 16
#define RS485TX 17

#define sensorFrameSize 19
#define sensorWaitingTime 1000
#define sensorID 0x01
#define sensorFunction 0x03
#define sensorByteResponse 0x0E

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define ORelay_RESET -1

millisDelay rebootDelay;
unsigned long REBOOT_DELAY_MS = 24ul * 60 * 60 * 1000; // 1 day in ms

const char *ssid = "norazlin@unifi";
const char *password = "bkh223811286";
const char *mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char *mqtt_user = "";
const char *mqtt_pass = "";

WiFiClient espClient;
PubSubClient client(espClient);

SoftwareSerial sensor(RS485RX, RS485TX);
BH1750 lightMeter;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); // sensor address, sensor type
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, ORelay_RESET);

unsigned char byteRequest[8] = {0X01, 0X03, 0X00, 0X00, 0X00, 0X07, 0X04, 0X08};
// unsigned char byteRequest[8] = {0X01, 0X03, 0X00, 0X30, 0X00, 0X08, 0X44, 0X03};
unsigned char byteResponse[19] = {};

float moisture, temperature, ph, ec, nitrogen, phosphorus, potassium;
float lux;
float tempv, humidity;
// int humidity;

long now = millis();
long lastMeasure = 0;

int sensorValue(int x, int y);

// mqtt callback
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// mqtt reconnect
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attemping MQTT connection...");
    //    Create random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    //    Attemp to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // subscribe topic
      client.subscribe("ESP32Soil/soil_sensors");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// setup wifi
void setup_wifi()
{
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  rebootDelay.start(REBOOT_DELAY_MS);
#if defined(ARDUINO_ARCH_ESP32)
  enableLoopWDT(); // default appears to be 5sec
#elif defined(ARDUINO_ARCH_ESP8266)
  ESP.wdtEnable(5000);
#else
#error only ESP8266 and ESP32 reboot supported
#endif
  Serial.begin(115200);
  sensor.begin(4800);
  delay(10);
  Wire.begin();
  lightMeter.begin();
  while (aht10.begin() != true) // for ESP-01 use aht10.begin(0, 2);
  {
    Serial.println(F("AHT1x not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free

    delay(5000);
  }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  // subscribe topic
  client.subscribe("ESP32Soil/soil_sensors");
}

void loop()
{
  if (rebootDelay.justFinished())
  {
    while (1)
    {
    } // force watch dog timer reboot
  }
  // Reconnecting.......
  if (!client.connected())
  {
    reconnect();
  }
  if (!client.loop())
  {
    client.connect("ESP32Soil/soil_sensors", mqtt_user, mqtt_pass);
  }

  now = millis();
  if (now - lastMeasure > 10000)
  {
    lastMeasure = now;

    // NPK Process
    sensor.write(byteRequest, 8);

    unsigned long resptime = millis();
    while ((sensor.available() < sensorFrameSize) && ((millis() - resptime) < sensorWaitingTime))
    {
      delay(1);
    }

    while (sensor.available())
    {
      for (int n = 0; n < sensorFrameSize; n++)
      {
        byteResponse[n] = sensor.read();
      }

      if (byteResponse[0] != sensorID && byteResponse[1] != sensorFunction && byteResponse[2] != sensorByteResponse)
      {
        return;
      }
    }

    //    Serial.println();
    //    Serial.println("===== SOIL PARAMETERS");
    //    Serial.print("Byte Response: ");
    //
    //    String responseString;
    //    for (int j = 0; j < 19; j++) {
    //      responseString += byteResponse[j] < 0x10 ? " 0" : " ";
    //      responseString += String(byteResponse[j], HEX);
    //      responseString.toUpperCase();
    //    }
    //    Serial.println(responseString);

    moisture = sensorValue((int)byteResponse[3], (int)byteResponse[4]) * 0.1;
    temperature = sensorValue((int)byteResponse[5], (int)byteResponse[6]) * 0.1;
    // ec = sensorValue((int)byteResponse[7], (int)byteResponse[8]);
    ph = sensorValue((int)byteResponse[9], (int)byteResponse[10]) * 0.1;
    nitrogen = sensorValue((int)byteResponse[11], (int)byteResponse[12]);
    phosphorus = sensorValue((int)byteResponse[13], (int)byteResponse[14]);
    potassium = sensorValue((int)byteResponse[15], (int)byteResponse[16]);
    ec = (nitrogen + phosphorus + potassium) * 0.1;

    lux = lightMeter.readLightLevel();
    tempv = aht10.readTemperature();
    humidity = aht10.readHumidity();

    StaticJsonBuffer<300> JSONbuffer;
    JsonObject &root = JSONbuffer.createObject();

    root["lx"] = lux;
    root["tv"] = tempv;
    root["h"] = humidity;
    root["m"] = moisture;
    root["t"] = temperature;
    root["ec"] = ec;
    root["ph"] = ph;
    root["n"] = nitrogen;
    root["p"] = phosphorus;
    root["k"] = potassium;

    char JSONmessageBuffer[200];
    root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    Serial.println("Sending message to MQTT topic..");
    //  Serial.println(JSONmessageBuffer);
    //    root.printTo(Serial);

    if (client.publish("ESP32Soil/soil_sensors", JSONmessageBuffer) == true)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Success sending message");
    }
    else
    {
      Serial.println("Error sending message");
    }

    Serial.println("Moisture: " + (String)moisture);
    Serial.println("Temperature: " + (String)temperature);
    Serial.println("Ec: " + (String)ec);
    Serial.println("Ph: " + (String)ph);
    Serial.println("Nitrogen: " + (String)nitrogen);
    Serial.println("Phosphorus: " + (String)phosphorus);
    Serial.println("Potassium: " + (String)potassium);
    Serial.println("Lux: " + (String)lux);
    Serial.println("Temperature Env: " + (String)tempv);
    Serial.println("Humidity: " + (String)humidity);

    //    lux = lightMeter.readLightLevel();
    //    temp = dht.readTemperature();
    //    humidity = dht.readHumidity();

    //    Oled display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print("Temperature:" + (String)tempv + " " + (char)247 + "C");

    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print("Humidity: " + (String)humidity + " %");
    //  display.print(humidity);
    //  display.print((char)247);
    //  display.print(" lx");

    display.setCursor(0, 20);
    display.setTextSize(1);
    display.print("Light:" + String(lux) + " lx");

    // soil temp & soil moisture

    display.setCursor(0, 30);
    display.setTextSize(1);
    display.print("Soil Temp:" + (String)temperature + " " + (char)247 + "C");

    display.setCursor(0, 40);
    display.setTextSize(1);
    display.print("Soil Moisture:" + String(moisture) + " %");

    display.display();
    delay(3000);

    display.clearDisplay();
    // display.setCursor(0, 0);
    // display.setTextSize(1);
    // display.setTextColor(WHITE);
    // display.print("Moisture:" + String(moisture) + " %");

    // display.setCursor(0, 10);
    // display.setTextSize(1);
    // display.setTextColor(WHITE);
    // display.print("Temperature:" + String(temperature) + " C");

    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("EC:" + String(ec) + " uS/cm");

    display.setCursor(0, 10);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("PH:" + String(ph));

    display.setCursor(0, 20);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("N:" + String(nitrogen) + " mg/kg");

    display.setCursor(0, 30);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("P:" + String(phosphorus) + " mg/kg");

    display.setCursor(0, 40);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("K:" + String(potassium) + " mg/kg");

    display.display();
  }
}

int sensorValue(int x, int y)
{
  int t = 0;
  t = x * 256;
  t = t + y;

  return t;
}