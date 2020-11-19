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

bool camera_found = false;

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

void print_wakeup_reason(){
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
}


void setup_i2c() {
  byte error, address;

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

void setup_serial() {
  delay(1000);
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
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
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
  print_wakeup_reason();
  setup_i2c();
  setup_pir();
  if (setup_wifi()) {
    setup_mqtt();
  }
  setup_camera();
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


void loop() {
  loop_pir();
  if (pirState == HIGH) {
    // take picture and upload it
  }
  if (pirState == LOW)
    enable_pir_sleep();
}