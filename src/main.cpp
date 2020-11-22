//
//
//

const char compile_date[] = __DATE__ " " __TIME__;

#include <Arduino.h>
#include <ArduinoLog.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <WiFi.h>
#include <SPIFFS.h>
#include <IP5306.h>

#include "esp_camera.h"
#include <WiFiClientSecure.h>

#define PWDN_GPIO_NUM       -1
#define RESET_GPIO_NUM      -1
#define XCLK_GPIO_NUM       32
#define SIOD_GPIO_NUM       13
#define SIOC_GPIO_NUM       12

#define Y9_GPIO_NUM         39
#define Y8_GPIO_NUM         36
#define Y7_GPIO_NUM         23
#define Y6_GPIO_NUM         18
#define Y5_GPIO_NUM         15
#define Y4_GPIO_NUM         4
#define Y3_GPIO_NUM         14
#define Y2_GPIO_NUM         5

#define VSYNC_GPIO_NUM      27
#define HREF_GPIO_NUM       25
#define PCLK_GPIO_NUM       19

// Strings for dynamic config
String Smyname = "ttgocamera",
  Spass = "TheC0l0r0fMag1c",
  Sssid = "AnkhMorpork",
  Smqttserver = "pi3.garf.de",
  Ssite = "Chattenweg5",
  Sroom = "camera",
  Smqttuser = "ttgocamera",
  Smqttpass = "none";
unsigned int Imqttport = 1883;
bool Bindoor = true;

int pirState = LOW;
gpio_num_t pirInput = GPIO_NUM_33; //ttgo pir on pin 33, for testing: button is on 34


WiFiClient espClient;
PubSubClient client;

float bme280TempAdjust = 0.0;

// Camera stuff
bool camera_found = false;
camera_fb_t *fb = NULL;

// Flags for display
#define DISPLAY_OFF 0
#define DISPLAY_TEMPERATURE 1
#define DISPLAY_HUMIDITY 2
#define DISPLAY_AIRPRESSURE 3
#define DISPLAY_LUX 4
#define DISPLAY_STRING 5
#define DISPLAY_DISTANCE 6
#define DISPLAY_TEMPHUM 7
#define DISPLAY_CO2 8

// forward declarations

boolean setup_wifi();
void take_picture();

// Debug functions
void log_config () {

  Log.verbose("Smyname = %s",Smyname.c_str());
  Log.verbose("Ssite = %s",Ssite.c_str());
  Log.verbose("Sroom = %s",Sroom.c_str());
  Log.verbose("Sssid = %s",Sssid.c_str());
  Log.verbose("Spass = %s",Spass.c_str());
  Log.verbose("Smqttuser = %s",Smqttuser.c_str());
  Log.verbose("Smqttpass = %s",Smqttpass.c_str());
  Log.verbose("Imqttport = %d",Imqttport);
}

// Logging helper routines
void printTimestamp(Print* _logOutput) {
  char c[12];
  sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
  _logOutput->print('\n');
}

esp_sleep_wakeup_cause_t print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Log.notice(F("Wakeup caused by external signal using RTC_IO")); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Log.notice(F("Wakeup caused by external signal using RTC_CNTL")); break;
    case ESP_SLEEP_WAKEUP_TIMER : Log.notice(F("Wakeup caused by timer")); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Log.notice(F("Wakeup caused by touchpad")); break;
    case ESP_SLEEP_WAKEUP_ULP : Log.notice(F("Wakeup caused by ULP program")); break;
    default : Log.notice(F("Wakeup was not caused by deep sleep: %d"),wakeup_reason); break;
  }

  return wakeup_reason;
}


void setup_i2c() {
  byte error, address;

  // 0x3c DISPLAY
  // 0x75
  // 0x77 BME280


  Log.notice("Scanning i2c bus");
  Wire.begin(I2CSDA, I2CSCL);
  Wire.setClock(10000);

  for(address = 1; address < 127; address++ ) {
    // Log.verbose(F("Trying %d"),address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Log.trace("I2C device found at address 0x%x",address);
    }
  }
}

void setup_ip5306() {

  IP5306_SetPowerOnLoadEnabled(0);


  Log.verbose(F("IP5306_GetKeyOffEnabled = %d"),IP5306_GetKeyOffEnabled());
  Log.verbose(F("IP5306_GetBoostOutputEnabled = %d"),IP5306_GetBoostOutputEnabled());
  Log.verbose(F("IP5306_GetPowerOnLoadEnabled = %d"),IP5306_GetPowerOnLoadEnabled());
  Log.verbose(F("IP5306_GetChargerEnabled = %d"),IP5306_GetChargerEnabled());
  Log.verbose(F("IP5306_GetBoostEnabled = %d"),IP5306_GetBoostEnabled());
  Log.verbose(F("IP5306_GetLowBatShutdownEnable = %d"),IP5306_GetLowBatShutdownEnable());
  Log.verbose(F("IP5306_GetBoostAfterVin = %d"),IP5306_GetBoostAfterVin());
  Log.verbose(F("IP5306_GetShortPressBoostSwitchEnable = %d"),IP5306_GetShortPressBoostSwitchEnable());
  Log.verbose(F("IP5306_GetFlashlightClicks = %d"),IP5306_GetFlashlightClicks());
  Log.verbose(F("IP5306_GetBoostOffClicks = %d"),IP5306_GetBoostOffClicks());
  Log.verbose(F("IP5306_GetLightLoadShutdownTime = %d"),IP5306_GetLightLoadShutdownTime());
  Log.verbose(F("IP5306_GetLongPressTime = %d"),IP5306_GetLongPressTime());
  Log.verbose(F("IP5306_GetChargingFullStopVoltage = %d"),IP5306_GetChargingFullStopVoltage());
  Log.verbose(F("IP5306_GetChargeUnderVoltageLoop = %d"),IP5306_GetChargeUnderVoltageLoop());
  Log.verbose(F("IP5306_GetEndChargeCurrentDetection = %d"),IP5306_GetEndChargeCurrentDetection());
  Log.verbose(F("IP5306_GetVoltagePressure = %d"),IP5306_GetVoltagePressure());
  Log.verbose(F("IP5306_GetChargeCutoffVoltage = %d"),IP5306_GetChargeCutoffVoltage());
  Log.verbose(F("IP5306_GetChargeCCLoop = %d"),IP5306_GetChargeCCLoop());
  Log.verbose(F("IP5306_GetVinCurrent = %d"),IP5306_GetVinCurrent());
  Log.verbose(F("IP5306_GetShortPressDetected = %d"),IP5306_GetShortPressDetected());
  Log.verbose(F("IP5306_GetLongPressDetected = %d"),IP5306_GetLongPressDetected());
  Log.verbose(F("IP5306_GetDoubleClickDetected = %d"),IP5306_GetDoubleClickDetected());
  Log.verbose(F("IP5306_GetPowerSource = %d"),IP5306_GetPowerSource());
  Log.verbose(F("IP5306_GetBatteryFull = %d"),IP5306_GetBatteryFull());
  Log.verbose(F("IP5306_GetLevelLeds = %d"),IP5306_GetLevelLeds());
  Log.verbose(F("IP5306_GetOutputLoad = %d"),IP5306_GetOutputLoad());

}

void publish_ip5306() {
  
}

void setup_serial() {
  // delay(1000);
  Serial.begin(115200);
}

void setup_pir() {
  pinMode(pirInput, INPUT);
  pirState = digitalRead(pirInput);
}

void setup_camera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
      Log.verbose(F("psram found"));
      // config.frame_size = FRAMESIZE_UXGA;
      config.frame_size = FRAMESIZE_SVGA;

      config.jpeg_quality = 12;
      config.fb_count = 1;
  } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
  };

  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  esp_err_t err = esp_camera_init(&config);

  if (err == ESP_OK) {
    camera_found = true;
    Log.verbose(F("camera ok"));
  } else {
    Log.error(F("esp32 camera error 0x%x"),err);
    return;
  }
}

boolean setup_wifi() {
  WiFi.persistent(false);
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);

  Log.verbose(F("IPv6 enabled = %T"),WiFi.enableIpV6());

  WiFi.begin(Sssid.c_str(), Spass.c_str());
  int count = 10;
  while ((WiFi.status() != WL_CONNECTED) && (count > 0)) {
    delay(1000);
    Log.verbose(F("Connecting..."));
    count--;
  }

  Log.verbose(F("IPv6 enabled = %T"),WiFi.enableIpV6());
  Log.notice(F("Connected. IP = %s, IPv6 = %s"),
    WiFi.localIP().toString().c_str(),
    WiFi.localIPv6().toString().c_str());

  return (WiFi.status() == WL_CONNECTED);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)  {
  String in[10];
  unsigned int wordcounter = 0;

  for (unsigned int i = 0; i < length; i++) {
    if ((char)payload[i] == ' ' && wordcounter < 9) {
      wordcounter++;
    } else {
      in[wordcounter] += String((char)payload[i]);
    }
  }
  Log.verbose("Message arrived[%s]: %d Words",topic,wordcounter);
  for (unsigned int i=0; i <= wordcounter; i++)
    Log.verbose("Word[%d] = %s",i,in[i].c_str());
}

void setup_mqtt() {
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), Imqttport);
  client.setCallback(mqtt_callback);
}

void setup_logging() {
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

void setup() {
  setup_serial();
  setup_logging();
  setup_camera();

  if (print_wakeup_reason() == ESP_SLEEP_WAKEUP_EXT0) {
    take_picture();
  }

  bool wifi_ok = setup_wifi();

  setup_i2c();
  setup_ip5306();
  setup_pir();
  if (wifi_ok) {
    setup_mqtt();
  }
}

boolean mqtt_reconnect() {
  // Loop until we're reconnected
  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/status", Ssite.c_str(), Sroom.c_str());

  if (WiFi.status() != WL_CONNECTED) {
    if (!setup_wifi())
      return false;
  }

  Log.verbose("Attempting MQTT connection...%d...",client.state());

  // Attempt to connect
  if (client.connect(Smyname.c_str(),Smqttuser.c_str(),Smqttpass.c_str(),mytopic,0,0,"stopped")) {
    Log.verbose("MQTT connected");

    client.publish(mytopic, "started");
    delay(10);
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
    // and to my location
    String mylocation = "/" + Ssite + "/cmd";
    client.subscribe(mylocation.c_str());
    String myroom = "/" + Ssite + "/" + Sroom + "/cmd";
    client.subscribe(myroom.c_str());
    delay(10);
  } else {
    Log.error("MQTT connect failed, rc=%d",client.state());
  }
  return client.connected();
}


unsigned long lastReconnectAttempt = 0;
void mqtt_publish(char *topic, char *msg) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (mqtt_reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();


  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/%s", Ssite.c_str(), Sroom.c_str(),topic);
  Log.verbose("MQTT Publish message [%s]:%s",mytopic,msg);

  client.publish(mytopic, msg);
}

void mqtt_publish(char *topic, uint8_t *buf, size_t len) {
  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/%s", Ssite.c_str(), Sroom.c_str(),topic);

  client.publish(mytopic,buf,len);
}

void mqtt_publish(const __FlashStringHelper *topic, const __FlashStringHelper *msg) {
  char a[64],b[64];
  snprintf(a,63,"%s",topic);
  snprintf(b,63,"%s",msg);
  mqtt_publish(a,b);
}

void mqtt_publish(const __FlashStringHelper *topic, char *msg) {
  char a[64];
  snprintf(a,63,"%s",topic);
  mqtt_publish(a,msg);
}

void mqtt_publish(const __FlashStringHelper *topic, const char *msg) {
  char buf[64];
  snprintf(buf,63,"%s",msg);
  mqtt_publish(topic, buf);
}


void mqtt_publish(char *topic, int i) {
  char buf[15];
  snprintf(buf,14,"%d",i);
  mqtt_publish(topic, buf);
}

void mqtt_publish(char *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_publish(topic,buf);
}

void mqtt_publish(const __FlashStringHelper *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_publish(topic,buf);
}

void mqtt_publish(char *topic, float value) {
  char buf[15];
  snprintf(buf,14,"%.3f",value);
  mqtt_publish(topic, buf);
}

void mqtt_publish(const __FlashStringHelper *topic, float value) {
  char buf[15];
  snprintf(buf,14,"%.3f",value);
  mqtt_publish(topic, buf);
}



void mqtt_influx(char *field, char *value) {
  char out[128];

  sprintf(out,"sensor,site=%s,room=%s %s=%s",
    Ssite.c_str(), Sroom.c_str(),field,value);
  mqtt_publish("influx",out);
}

void mqtt_influx(char *field, int i) {
  char buf[15];
  snprintf(buf,14,"%di",i);
  mqtt_influx(field,buf);
}

void mqtt_influx(char *field, float f) {
  char buf[15];
  snprintf(buf,14,"%.3f",f);
  mqtt_influx(field,buf);
}

void mqtt_influx(char *field, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_influx(field,buf);
}

void mqtt_influx(char *field, bool b) {
  if (b)
    mqtt_influx(field,"true");
  else
    mqtt_influx(field,"false");
}

void mqtt_influx(const __FlashStringHelper *topic, const __FlashStringHelper *msg) {
  char a[32],b[32];
  snprintf(a,31,"%s",topic);
  snprintf(b,31,"%s",msg);
  mqtt_influx(a,b);
}

void mqtt_influx(const __FlashStringHelper *topic, char *msg) {
  char a[32];
  snprintf(a,31,"%s",topic);
  mqtt_influx(a,msg);
}

void mqtt_influx(const __FlashStringHelper *topic, const char *msg) {
  char buf[32];
  snprintf(buf,31,"%s",msg);
  mqtt_influx(topic, buf);
}

void mqtt_influx(const __FlashStringHelper *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_influx(topic,buf);
}

void mqtt_influx(const __FlashStringHelper *topic, float f) {
  char buf[32];
  snprintf(buf,31,"%.3f",f);
  mqtt_influx(topic,buf);
}



#define BOUNDARY     "--------------------------133747188241686651551404"
String body(String content , String message)
{
  String data;
  data = "--";
  data += BOUNDARY;
  data += F("\r\n");
  if(content=="imageFile")
  {
    data += F("Content-Disposition: form-data; name=\"imageFile\"; filename=\"picture.jpg\"\r\n");
    data += F("Content-Type: image/jpeg\r\n");
    data += F("\r\n");
  }
  else
  {
    data += "Content-Disposition: form-data; name=\"" + content +"\"\r\n";
    data += "\r\n";
    data += message;
    data += "\r\n";
  }
   return(data);

}

String header(String token,size_t length)
{
  String  data;
      data =  F("POST /receive/upload.php HTTP/1.1\r\n");
      data += F("cache-control: no-cache\r\n");
      data += F("Content-Type: multipart/form-data; boundary=");
      data += BOUNDARY;
      data += "\r\n";
      data += F("User-Agent: PostmanRuntime/6.4.1\r\n");
      data += F("Accept: */*\r\n");
      data += F("Host: ");
      data += F("lancre.garf.de");
      data += F("\r\n");
      data += F("accept-encoding: gzip, deflate\r\n");
      data += F("Connection: keep-alive\r\n");
      data += F("content-length: ");
      data += String(length);
      data += "\r\n";
      data += "\r\n";
    return(data);
}


String sendImage(String token,String message, uint8_t *data_pic,size_t size_pic)
{
  String bodyTxt =  body("uploadBtn",message);
  String bodyPic =  body("imageFile",message);
  String bodyEnd =  String("--")+BOUNDARY+String("--\r\n");
  size_t allLen = bodyTxt.length()+bodyPic.length()+size_pic+bodyEnd.length();
  String headerTxt =  header(token,allLen);
  WiFiClient http;

  if (!http.connect("lancre.garf.de",80))
  {
    return("connection failed");
  }

   http.print(headerTxt+bodyTxt+bodyPic);
   http.write(data_pic,size_pic);
   http.print("\r\n"+bodyEnd);

   delay(20);
   long tOut = millis() + 100000;
   while(http.connected() && tOut > millis())
   {
    if (http.available())
    {
      String serverRes = http.readStringUntil('\r');
        return(serverRes);
    }
   }
   return "Error";
}


void take_picture() {

  sensor_t *s = esp_camera_sensor_get();

  s->set_brightness(s,2);
  s->set_exposure_ctrl(s,1);

  Log.verbose(F("camera framesize: %u"),s->status.framesize);
  fb = esp_camera_fb_get();
  if (!fb) {
    Log.error(F("camera capture failed"));
  } else {
    Log.verbose(F("camera format %d"), fb->format);
    Log.verbose(F("camera buf size %d"), fb->len);
  }
}

void transmit_picture() {
  if (fb) {
    Log.verbose(F("sending Image"));
    Log.verbose(F("sent Image: %s"),sendImage("x1","Upload",fb->buf,fb->len).c_str());

    esp_camera_fb_return(fb);
    fb = NULL;
  } else {
    Log.verbose(F("no image to send"));
  }
}

void enable_pir_sleep() {
  esp_sleep_enable_ext0_wakeup(pirInput,1);
  Log.verbose(F("sleeping"));
  delay(1000);

  esp_deep_sleep_start();
}

// read PIR if it exists
void loop_pir() {
  int pir_old = pirState;
  pirState = digitalRead(pirInput);
  // Low -> Low
  if (pir_old == LOW && pirState == LOW)
    return;
  // Low -> High
  if (pir_old == LOW && pirState == HIGH) {
    Log.verbose(F("motion started l>h"));
  }

  // High -> High
  if (pir_old == HIGH && pirState == HIGH)
    return;
  // High -> Low
  if (pir_old == HIGH && pirState == LOW) {
    Log.verbose(F("motion stopped h>l"));
  }
}

void loop_publish_voltage(){

}

void loop() {
  loop_pir();
  if (pirState == HIGH) {
    // take picture and upload it
    take_picture();
  }

  transmit_picture();

  if (pirState == LOW)
    enable_pir_sleep();

  delay(1000);
}
