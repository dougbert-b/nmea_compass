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

/*
    BLE stuff based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


constexpr bool verbose = true;
constexpr bool verbose_bno = false;



double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


tNMEA0183* NMEA0183 = nullptr;

sensors_event_t orientationData;
uint8_t system_calib;
uint8_t gyro_calib;
uint8_t accel_calib;
uint8_t mag_calib;
uint8_t fully_calib;

bool found_calib = false;


// Persistent Data
class PersistentData {
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

PersistentData persistentData;




// Older versions of the BLE library would automatically resume advertising when the
// client disconnected, but now we have to do that ourselves via this callback.
class MyBLEServerCallbacks : public BLEServerCallbacks {
public:
  void onDisconnect(BLEServer *pServer) override;
};

void MyBLEServerCallbacks::onDisconnect(BLEServer *pServer)
{
  pServer->startAdvertising();
}



class My2901Descriptor : public BLEDescriptor {
public:
  My2901Descriptor(const char *description);
  My2901Descriptor() = delete;
};

My2901Descriptor::My2901Descriptor(const char *description)
  : BLEDescriptor(BLEUUID("2901"))
{
  setValue(description);
  setAccessPermissions(ESP_GATT_PERM_READ);
}


// This class is both a Characteristic and a set of callbacks for it.
class MyConfigDataCharacteristic : public BLECharacteristic, public BLECharacteristicCallbacks {
public:
  typedef void(*SetFunc)(uint8_t val);

  MyConfigDataCharacteristic(const char *uuid, const char *descr, SetFunc setter = nullptr) : 
    BLECharacteristic(BLEUUID(uuid), BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE ),
    BLECharacteristicCallbacks(),
    _val(0),
    _setter(setter)
  {
    setValue(getValStr());  // In base class BLECharacteristic

    // Do this after setValue(), to avoid weird loops.
    setCallbacks(this);  // In base class BLECharacteristic

    addDescriptor(new My2901Descriptor(descr));
  }
    
  void onWrite(BLECharacteristic *characteristic) override;
  uint8_t getVal() const { return _val; }
  std::string getValStr() const;
private:
  uint8_t _val;
  SetFunc _setter;
};


std::string
MyConfigDataCharacteristic::getValStr() const
{
  char s[16];
  snprintf(s, sizeof(s), "0x%x", _val);
  return std::string(s);
}

void MyConfigDataCharacteristic::onWrite(BLECharacteristic *characteristic)
{
  if (characteristic == this) {
    sscanf((char*)getData(), "%x", &_val);  // Convert BLE payload from ascii string to char.
    if (_setter) _setter(_val);
  } else {
    log_e("Improper structure of angleCharacteristic!");
  }
}


class MyAngleCharacteristic : public BLECharacteristic {
public:
  MyAngleCharacteristic(BLEService *pService, const char *uuid, const char *descr) : 
    BLECharacteristic(BLEUUID(uuid), BLECharacteristic::PROPERTY_READ |
                                     BLECharacteristic::PROPERTY_NOTIFY )
  {
    setValue("0.0");  // In base class BLECharacteristic
    addDescriptor(new My2901Descriptor(descr));
    pService->addCharacteristic(this);
  }

  void setVal(float val, bool noti=false) { setValue(std::to_string(val)); if (noti) notify(); }
};


class MyStringCharacteristic : public BLECharacteristic {
public:
  MyStringCharacteristic(BLEService *pService, const char *uuid, const char *descr) : 
    BLECharacteristic(BLEUUID(uuid), BLECharacteristic::PROPERTY_READ |
                                     BLECharacteristic::PROPERTY_NOTIFY )
  {
    setValue("");  // In base class BLECharacteristic
    addDescriptor(new My2901Descriptor(descr));
    pService->addCharacteristic(this);
  }

  void setVal(std::string val, bool noti=false) { setValue(val); if (noti) notify(); }
};



// This class is both a Characteristic and a set of callbacks for it.  It is write-only.
class MyResetCharacteristic : public BLECharacteristic, public BLECharacteristicCallbacks {
public:
  typedef void(*ResetFunc)();

  MyResetCharacteristic(BLEUUID uuid, ResetFunc resetter, const char *descr = nullptr) : 
    BLECharacteristic(uuid, BLECharacteristic::PROPERTY_WRITE),
    BLECharacteristicCallbacks(),
    _resetter(resetter)
  {
    setValue("");

    addDescriptor(new My2901Descriptor(descr ? descr : "reset"));

    // Do this after setValue(), to avoid weird loops.
    setCallbacks(this);  // In base class BLECharacteristic
  }
    
  void onWrite(BLECharacteristic *characteristic) override;

private:
  ResetFunc _resetter;
};



void MyResetCharacteristic::onWrite(BLECharacteristic *characteristic)
{

  setValue("");  // Discard whatever value was written

  if (characteristic == this) {
    if (_resetter) _resetter();
  } else {
    log_e("Improper structure of resetCharacteristic!");
  }
}


/* *****************************************************************
    <!-- 180A: org.bluetooth.service.device_information -->
    <service uuid="180A" id="device_information">
        <description>Device Information</description>

        <!-- 2A29: org.bluetooth.characteristic.manufacturer_name_string -->
        <!-- (support for this characteristic is MANDATORY 
              according to the profile spec) -->
        <characteristic uuid="2A29" id="c_manufacturer_name">
            <description>Manufacturer Name</description>
            <properties read="true" const="true" />
            <value>My SuperDuper Company</value>
        </characteristic>

        <!-- 2A24: org.bluetooth.characteristic.model_number_string -->
        <characteristic uuid="2A24" id="c_model_number">
            <description>Model Number</description>
            <properties read="true" const="true" />
            <value>TESTTHERMO-0001</value>
        </characteristic>

    </service>

    ************************************************************************ */


class DeviceInformationService
{
public:
  DeviceInformationService() = delete;
  DeviceInformationService(BLEServer *server);
private:
  BLEService *_service;
  BLECharacteristic *_manufNameCharacteristic;
  BLECharacteristic *_modelNumberCharacteristic;
  BLECharacteristic *_firmwareRevCharacteristic;
};

DeviceInformationService::DeviceInformationService(BLEServer *server)
{
  _service = server->createService(BLEUUID("180a"), 32 /*numHandles*/);

  _manufNameCharacteristic = _service->createCharacteristic("2a29", BLECharacteristic::PROPERTY_READ);
  _manufNameCharacteristic->setValue("dougbraun.com");

  _modelNumberCharacteristic = _service->createCharacteristic("2a24", BLECharacteristic::PROPERTY_READ);
  _modelNumberCharacteristic->setValue("NMEA Compass 1.0");

  _firmwareRevCharacteristic = _service->createCharacteristic("2a26", BLECharacteristic::PROPERTY_READ);
  _firmwareRevCharacteristic->setValue(__DATE__);

  _service->dump();

  _service->start();
  
}



// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

const char *SERVICE_UUID  =      "ac73740d-faf4-4c5c-a109-8abaaff98abc";

const char *HEADING_UUID  =      "cd3fb5aa-c679-4d3e-9eb4-97912c27b298";
const char *ROLL_UUID  =         "cd3fb5aa-c679-4d3e-9eb4-3990fa52213b";
const char *PITCH_UUID  =        "cd3fb5aa-c679-4d3e-9eb4-f765e94d054b";
const char *CALIBRATION_UUID =   "cd3fb5aa-c679-4d3e-9eb4-8ce31b0538c6";


const char *AXIS_CONFIG_UUID =   "cd3fb5aa-c679-4d3e-9eb4-85b6bfc15110";
const char *AXIS_SIGN_UUID =     "cd3fb5aa-c679-4d3e-9eb4-7ae7aed6bf55";


const char *RESET_UUID =          "cd3fb5aa-c679-4d3e-9eb4-c12dcc1b4ccb";



BLEServer *pServer(nullptr);
BLEService *pService(nullptr);
DeviceInformationService *pDeviceInfoService(nullptr);


MyAngleCharacteristic *pHeadingChar(nullptr);
MyAngleCharacteristic *pRollChar(nullptr);
MyAngleCharacteristic *pPitchChar(nullptr);
MyStringCharacteristic *pCalibChar(nullptr);


MyConfigDataCharacteristic *pAxisConfigChar(nullptr);
MyConfigDataCharacteristic *pAxisSignChar(nullptr);

MyResetCharacteristic *pResetChar(nullptr);

MyBLEServerCallbacks serverCallbacks;

void resetEncoders(); // Defined a bit further down


bool haveConnection(false);


void resetEncoders() {
}


void reset()
{
  delay(1000);
  ESP.restart(); 
}



void handleAxisRemapPost(){
  int val = -1;

  if (val >= 0 && val <= 0x3f) {
    persistentData.axis_config = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)val;
  }
  else if (val >= 0 && val <= 0x07) {
    persistentData.axis_sign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)val;
  }
  persistentData.axis_valid = true;  
  persistentData.commit();
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
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    while (1);
  }

  // Note: The ESP32 EEPROM library is used differently than the official Arduino version.
  if (!persistentData.begin()) {
    Serial.println("Cannot init EEPROM");
    while (1);
  }

  if (!persistentData.axis_valid) {
    // First-time initialization
    persistentData.axis_valid = true;

    //persistentData.axis_config = Adafruit_BNO055::REMAP_CONFIG_P1;
    //persistentData.axis_sign = Adafruit_BNO055::REMAP_SIGN_P1;

      // X = -Y, Y = Z, Z = -X  For mounting on forward side of vertical bulkhead, cable on right.
    persistentData.axis_config = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)0x09;
    persistentData.axis_sign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)0x05;
    if (!persistentData.commit()) {
       Serial.println("Cannot commit to EEPROM");
       while (1);
    }
    Serial.println("Initialized axis remap");
  } else {
    bno.setAxisRemap(persistentData.axis_config);
    bno.setAxisSign(persistentData.axis_sign);
    Serial.printf("Set axis data to 0x%x  0x%x\n", persistentData.axis_config, persistentData.axis_sign);
  }



  /*
  *  Read the sensor's unique ID in the EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */

  sensor_t sensor;
  bno.getSensor(&sensor);

  if (persistentData.id != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      Serial.printf("\nSensor ID: %lx  Stored ID: %lx\n", sensor.sensor_id, persistentData.id);

      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");

      displaySensorOffsets(persistentData.calibration_data);
      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(persistentData.calibration_data);

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


  Serial.println("Starting BLE server!");

  // Hardware initialized here

  BLEDevice::init("DougCompass");
  pServer = BLEDevice::createServer();
  // Not needed at the moment  pServer->setCallbacks(&serverCallbacks);
  
  pDeviceInfoService = new DeviceInformationService(pServer);

  pService = pServer->createService(BLEUUID(SERVICE_UUID), 32 /*numHandles*/);

  
  pAxisConfigChar = new MyConfigDataCharacteristic(AXIS_CONFIG_UUID, "axis config");
  pService->addCharacteristic(pAxisConfigChar);

  pAxisSignChar = new MyConfigDataCharacteristic(AXIS_SIGN_UUID, "axis sign");
  pService->addCharacteristic(pAxisSignChar);
 
  Serial.println("resolution done!");

  pHeadingChar = new MyAngleCharacteristic(pService, HEADING_UUID, "heading value");
  pRollChar = new MyAngleCharacteristic(pService, ROLL_UUID, "roll value");
  pPitchChar = new MyAngleCharacteristic(pService, PITCH_UUID, "pitch value");

  pCalibChar = new MyStringCharacteristic(pService, CALIBRATION_UUID, "calibration");


  Serial.println("pitch done!");


  
  pResetChar = new MyResetCharacteristic(BLEUUID(RESET_UUID), resetEncoders, "reset");
  pService->addCharacteristic(pResetChar);
  Serial.println("reset done!");



  pService->dump();

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("Initialization finished.");
  Serial.println(BLEDevice::toString().c_str());

}
  
  
void light_sleep(unsigned long delay_us) {

  esp_sleep_enable_timer_wakeup(delay_us); // ESP32 wakeup delay in uS.
  
  Serial.flush(); 
  esp_light_sleep_start();

  if (verbose) {
    print_wakeup_reason(); //Print the wakeup reason for ESP32
  }
}


void loop() {

  // Detect and process a change in the connection state.
  if (pServer->getConnectedCount() > 0) {
    if (!haveConnection) {
      Serial.printf("Got a connection!\n");
      haveConnection = true;
    }
  } else {
    if (haveConnection) {
      Serial.printf("No longer connected.\n");
      haveConnection = false;
      pServer->startAdvertising();
    }
  }


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

    std::string cal_str = " S"+std::to_string(system_calib)+"  G"+std::to_string(gyro_calib)+"  A"+
                          std::to_string(accel_calib)+"  M"+std::to_string(mag_calib);
    pCalibChar->setVal(cal_str, haveConnection);
        
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
    }

    if (verbose) {
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


    pHeadingChar->setVal(orientationData.orientation.x, haveConnection);
    pRollChar->setVal(-orientationData.orientation.y, haveConnection);
    pPitchChar->setVal(-orientationData.orientation.z, haveConnection);
  
    
    
    if (!found_calib && bno.isFullyCalibrated()) {

      Serial.println("\nFully calibrated!");
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");
      
      bno.getSensorOffsets(persistentData.calibration_data);

      sensor_t sensor;
      bno.getSensor(&sensor);
      persistentData.id = sensor.sensor_id;

      displaySensorOffsets(persistentData.calibration_data);
      Serial.println("\n\nStoring calibration data to EEPROM...");
      Serial.printf("\nSensor ID: %lx\n", sensor.sensor_id);
      
      persistentData.commit();
      Serial.println("Data stored to EEPROM.");
      found_calib = true;

      Serial.println("\n--------------------------------\n");
      delay(500);
    }

  }

  delay(5); //allow LED to blink and the cpu to switch to other tasks
  digitalWrite(LED_BUILTIN, LOW);
  

  if (true) {
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
  persistentData.id = 0L;
  persistentData.commit();
  Serial.println("Stored calibration invalidated.");
}
