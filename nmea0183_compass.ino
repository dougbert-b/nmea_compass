   /* Hardware pinout for DSP32 DEVKIT board

   Magnetometer:
   SDA = 21;
   SCL = 22;

   NMEA0183 out = 17

   */



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include <HardwareSerial.h>
#include <EEPROM.h>



// Load Wi-Fi library
#include <WiFi.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <DNSServer.h>

#include <AsyncElegantOTA.h>

constexpr bool verbose = false;
constexpr bool verbose_bno = false;


bool wiFiActive = true;


double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


tNMEA0183* NMEA0183 = nullptr;


// Replace with your network credentials
const char* ssid     = "Compass";
const char* password = "1234567890";

IPAddress apIp(192, 168, 4, 1);
const char* hostname = "compass";


DNSServer dnsServer;

// Set web server port number to 80
AsyncWebServer server(80);

unsigned long lastConnectionTime = 0;



class PData {
public:
  long id;
  adafruit_bno055_offsets_t calibration_data;
  bool axis_valid;
  Adafruit_BNO055::adafruit_bno055_axis_remap_config_t axis_config;
  Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t axis_sign;

  bool begin() {
    bool ok = EEPROM.begin(sizeof(this)); 
    if (ok) {
      EEPROM.get(0, *this);
    }
    return ok;
  }

  bool commit() {
    EEPROM.put(0, *this);
    return EEPROM.commit();
  }
};

PData pData;



void onWiFiEvent(arduino_event_id_t event) {
  switch (event) {
  case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
    Serial.println("Station connected.");
    lastConnectionTime = millis();
    break;
  case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
    Serial.println("Station disconnected.");
    lastConnectionTime = millis();
    break;
  case ARDUINO_EVENT_WIFI_AP_START:
    if (WiFi.softAPsetHostname(hostname)) {
      Serial.println("AP started, setting hostname");
    }
    break;
  case ARDUINO_EVENT_WIFI_AP_STOP:
    Serial.println("AP stopped.");
    break;
  }

}

sensors_event_t orientationData;
uint8_t system_calib;
uint8_t gyro_calib;
uint8_t accel_calib;
uint8_t mag_calib;
uint8_t fully_calib;

bool found_calib = false;

const char *msgHeader =  "<!DOCTYPE html><html>"
                    "<head><name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                    "<link rel=\"icon\" href=\"data:,\">"
                    // CSS to style the on/off buttons 
                    // Feel free to change the background-color and font-size attributes to fit your preferences
                    "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}"
                    ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;"
                    "text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}"
                    ".button2 {background-color: #555555;}</style>";

                    
            
const char *msgTrailer = "</body></html>";


void handleRoot(AsyncWebServerRequest *request){

  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->addHeader("Server","ESP Async Web Server");
  response->print(msgHeader);

  response->print("<meta http-equiv=\"refresh\" content=\"2\">"
                    "<title>ESP32 Compass Web Server</title>"
                    "</head><body>"
                    "<h1>ESP32 Compass Web Server</h1>");

  response->printf("<br>Build  %s  %s<br>", __DATE__, __TIME__);
  response->printf("<br>Heading: %4.4f<br>Roll (-y): %4.4f<br>Pitch (-z): %4.4f<br>",
                   orientationData.orientation.x, -orientationData.orientation.y, -orientationData.orientation.z);
  response->printf("<br>Calibration:<br>system: %d  gyro: %d  accel: %d  mag: %d<br>", system_calib, gyro_calib, accel_calib, mag_calib);
  response->printf("<br>fully_calib: %d<br>found_calib: %d", fully_calib, found_calib);
  response->printf("<br>axis_config: 0x%02x<br>axis_sign: 0x%02x", pData.axis_config, pData.axis_sign);
  response->print("<br><br><a href=\"update\">Update Firmware</a>");
  response->print("<br><br><a href=\"uncalibrate\">Reset Compass Calibration</a>");
  response->print("<br><br><a href=\"axis_remap\">Remap Axes</a>");
  response->print("<br><br><a href=\"reboot\">Reboot</a>");

  response->print(msgTrailer);

  request->send(response);
  lastConnectionTime = millis();
}

void handleRootText(AsyncWebServerRequest *request){

  AsyncResponseStream *response = request->beginResponseStream("text/plain");
  response->addHeader("Server","ESP Async Web Server");
  response->printf("ESP32 Compass Web Server\n");
  response->printf("Build %s  %s\n", __DATE__, __TIME__);
  response->printf("Heading: %4.4f  Roll (-y): %4.4f  Pitch (-z): %4.4f\n",
                   orientationData.orientation.x, -orientationData.orientation.y, -orientationData.orientation.z);
  response->printf("Calibration: system: %d  gyro: %d  accel: %d  mag: %d\n", system_calib, gyro_calib, accel_calib, mag_calib);
  response->printf("fully_calib: %d  found_calib: %d\n", fully_calib, found_calib);
  response->printf("axis_config: 0x%02x  axis_sign: 0x%02x\n", pData.axis_config, pData.axis_sign);

  request->send(response);
  lastConnectionTime = millis();
}


void handleNotFound(AsyncWebServerRequest *request) {

  String message = "File Not Found\n\n";
  message += "URI: ";
  message += request->url();
  message += "\nMethod: ";
  message += (request->method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += request->args();
  message += "\n";
  for (uint8_t i = 0; i < request->args(); i++) {
    message += " " + request->argName(i) + ": " + request->arg(i) + "\n";
  }
  request->send(404, "text/plain", message);
}

void handleUncalibrate(AsyncWebServerRequest *request) {
  clearCalib();
  String message = "Calibration erased - please power-cycle\n\n";
  request->send(200, "text/plain", message);
}

void handleReboot(AsyncWebServerRequest *request){
  AsyncResponseStream *response = request->beginResponseStream("text/plain");
  response->addHeader("Server","ESP Async Web Server");
  response->printf("ESP32 Compass Web Server\n");
  response->printf("Rebooting...\n");
  request->send(response);

  delay(1000);
  ESP.restart(); 
}

void handleAxisRemap(AsyncWebServerRequest *request) {

  const char *remapMsgHeader =  
                    "<title>ESP32 Compass Axis Remapping</title>"
                    "</head><body>"
                    "<h1>Axis Remapping</h1>";

  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->addHeader("Server","ESP Async Web Server");
  response->print(msgHeader);
  response->print(remapMsgHeader);
  response->printf("<form action=\"/axis_remap\" method=\"post\">");
  response->printf("<label for=\"axis_config\">Axis config:</label><br>");
  response->printf("<input type=\"text\" id=\"axis_config\" name=\"axis_config\" value=\"%02x\"><br>", pData.axis_config);
  response->printf("<label for=\"axis_sign\">Axis sign:</label><br>");
  response->printf("<input type=\"text\" id=\"axis_sign\" name=\"axis_sign\" value=\"%02x\"><br>", pData.axis_sign);
  response->printf("<br><input type=\"submit\" value=\"Submit\">");
  response->printf("</form>");
  response->print("<br><br><a href=\"/\">Home</a>");
  response->print(msgTrailer);

  request->send(response);
}

void handleAxisRemapPost(AsyncWebServerRequest *request){
  int params = request->params();
  for (int i=0;i<params;i++){
    AsyncWebParameter* p = request->getParam(i);
    if (p->isPost()) {
      int val = -1;
      sscanf(p->value().c_str(), "%x", &val);

      if (p->name() == "axis_config" && val >= 0 && val <= 0x3f) {
        pData.axis_config = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)val;
      }
      else if (p->name() == "axis_sign" && val >= 0 && val <= 0x07) {
        pData.axis_sign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)val;
      }
    }
  }
  pData.axis_valid = true;  
  pData.commit();

  const char *remapMsgHeader =  
                    "<title>ESP32 Compass Axis Remapping</title>"
                    "</head><body>"
                    "<h1>Axis Remapping</h1>";

  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->addHeader("Server","ESP Async Web Server");
  response->print(msgHeader);
  response->print(remapMsgHeader);
  response->printf("New values:  config 0x%x  sign 0x%x<br>", pData.axis_config, pData.axis_sign);
  response->print("<br><br><a href=\"/\">Home</a>");
  response->print("<br><br><a href=\"reboot\">Reboot</a>");

  response->print(msgTrailer);

  request->send(response);
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

unsigned long nextUpdate = 0;


void setup() {
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


  // This will use the default I2C Wire pins.
  if (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    //while (1);
  }

  // Note: The ESP32 EEPROM library is used differently than the official Arduino version.
  pData.begin();

  if (!pData.axis_valid) {
    // First-time initialization
    pData.axis_valid = true;

    //pData.axis_config = Adafruit_BNO055::REMAP_CONFIG_P1;
    //pData.axis_sign = Adafruit_BNO055::REMAP_SIGN_P1;

      // X = -Y, Y = Z, Z = -X  For mounting on forward side of vertical bulkhead, cable on right.
    pData.axis_config = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)0x09;
    pData.axis_sign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)0x05;
    pData.commit();
    Serial.println("Initialized axis remap");
  } else {
    bno.setAxisRemap(pData.axis_config);
    bno.setAxisSign(pData.axis_sign);
    Serial.printf("Set axis data to 0x%x  0x%x\n", pData.axis_config, pData.axis_sign);
  }



  /*
  *  Read the sensor's unique ID in the EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */

  sensor_t sensor;
  bno.getSensor(&sensor);

  if (pData.id != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      Serial.printf("\nSensor ID: %lx  Stored ID: %lx\n", sensor.sensor_id, pData.id);

      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");

      displaySensorOffsets(pData.calibration_data);
      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(pData.calibration_data);

      Serial.println("\n\nCalibration data loaded into BNO055");
      found_calib = true;
  }

  delay(1000);

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  // For NMEA output on ESP32
  Serial2.begin(4800, SERIAL_8N1, 16 /*Rx pin*/, 17 /*Tx pin*/, true /*invert*/);


  NMEA0183 = new tNMEA0183(&Serial2);
  NMEA0183->Open();


  if (wiFiActive) {

    WiFi.disconnect(true, true);
    WiFi.softAPsetHostname("compass");
    WiFi.mode(WIFI_AP);

    WiFi.onEvent(onWiFiEvent, ARDUINO_EVENT_WIFI_AP_START);
    WiFi.onEvent(onWiFiEvent, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(onWiFiEvent, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

    Serial.println("Opening AP...");
    WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    WiFi.setTxPower(WIFI_POWER_8_5dBm);  // About 10% of max power

    // Connect to Wi-Fi network with SSID and password
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid, password);
    WiFi.softAPsetHostname("compass");

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    String s = WiFi.softAPSSID();
    Serial.print("AP SSID: ");
    Serial.println(s);
    
    s = WiFi.softAPgetHostname();
    Serial.print("hostname: ");
    Serial.println(s);
    
    // setup after AP start (so IP exists)
    //dnsServer.start(53, "compass", WiFi.softAPIP());

    // you can also put a * to resolve ALL DNS to this IP
    dnsServer.start(53, "*", WiFi.softAPIP());
    
    
    delay(100);
    
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/text", HTTP_GET, handleRootText);
    server.on("/uncalibrate", HTTP_GET, handleUncalibrate);
    server.on("/axis_remap", HTTP_GET, handleAxisRemap);
    server.on("/axis_remap", HTTP_POST, handleAxisRemapPost);
    server.on("/reboot", HTTP_GET, handleReboot);



    server.on("/inline", [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "this works as well");
    });

    // respond to GET requests on URL /heap
    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", String(ESP.getFreeHeap()));
    });


    server.onNotFound(handleNotFound);

    AsyncElegantOTA.begin(&server);
    server.begin();
    Serial.println("HTTP server started");
  }

  lastConnectionTime = millis();
}
  
void light_sleep(unsigned long delay_us) {

  esp_sleep_enable_timer_wakeup(delay_us); // ESP32 wakeup delay in uS.
  
  Serial.flush(); 
  esp_light_sleep_start();

  if (verbose) {
    print_wakeup_reason(); //Print the wakeup reason for ESP32
  }
}


void loop(){

  if (true || nextUpdate < millis()) {
    nextUpdate += 1000;
  

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    if (verbose_bno) {
      
      sensors_event_t magnetometerData;
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

      printEvent(&orientationData);
      printEvent(&magnetometerData);

      uint8_t system_status;
      uint8_t self_test_result;
      uint8_t system_error;

      bno.getSystemStatus(&system_status, &self_test_result, &system_error);
      Serial.print("Status: ");
      Serial.print(system_status);
      Serial.print("  Self-test: ");
      Serial.print(self_test_result);
      Serial.print("  Error: ");
      Serial.println(system_error);

    }

    
    bno.getCalibration(&system_calib, &gyro_calib, &accel_calib, &mag_calib);
    fully_calib = bno.isFullyCalibrated();

    
    
    if (verbose_bno) {
      Serial.print("Calibration:  System: ");
      Serial.print(system_calib);
      Serial.print("  Gyro: ");
      Serial.print(gyro_calib);
      Serial.print("  Accel: ");
      Serial.print(accel_calib);
      Serial.print("  Mag: ");
      Serial.print(mag_calib);

      Serial.print("  Fully calibrated: ");
      Serial.println(fully_calib);

      Serial.print("Heading: ");
      Serial.print(orientationData.orientation.x);
      Serial.print("  Roll (-y): ");
      Serial.print(-orientationData.orientation.y);
      Serial.print("  Pitch (-z): ");
      Serial.println(-orientationData.orientation.z);
    }

    double heading;
    if (mag_calib > 0 && system_calib + mag_calib >= 2) {
      digitalWrite(LED_BUILTIN, HIGH);
      // The NMEA0183 API wants heading in radians
      heading = orientationData.orientation.x * (2*PI/360.0);
    } else {
      heading = NMEA0183DoubleNA;  // Send a blank value
    }

    tNMEA0183Msg NMEA0183Msg;
    if ( NMEA0183SetHDM(NMEA0183Msg, heading, "HC") ) {  // HC means magnetic compass
      NMEA0183->SendMessage(NMEA0183Msg);
    }

    
    
    if (!found_calib && bno.isFullyCalibrated()) {

      Serial.println("\nFully calibrated!");
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");
      
      bno.getSensorOffsets(pData.calibration_data);

      sensor_t sensor;
      bno.getSensor(&sensor);
      pData.id = sensor.sensor_id;

      displaySensorOffsets(pData.calibration_data);
      Serial.println("\n\nStoring calibration data to EEPROM...");
      Serial.printf("\nSensor ID: %lx\n", sensor.sensor_id);
      
      pData.commit();
      Serial.println("Data stored to EEPROM.");
      found_calib = true;

      Serial.println("\n--------------------------------\n");
      delay(500);
    }

  }

  delay(5); //allow LED to blink and the cpu to switch to other tasks
  digitalWrite(LED_BUILTIN, LOW);
  
  if (wiFiActive) {
    dnsServer.processNextRequest();
  }

  if (wiFiActive && (millis() > lastConnectionTime+180000)) {
    // If there has been no web server activity in three minutes, turn off the WiFi
    wiFiActive = false;
    Serial.println("WiFi turned off due to inactivity");
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
  }
  

  if (wiFiActive) {
    delay(500);
  } else {
    light_sleep(500000L);
  }

}

void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print("Event ");
  Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


void clearCalib()
{
  pData.id = 0L;
  pData.commit();
  Serial.println("Stored calibration invalidated.");
}