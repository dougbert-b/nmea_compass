   /* Hardware pinout for ESP32 DEVKIT board

   Magnetometer:
   SDA = 21;
   SCL = 22;

   NMEA0183 out = 17

   */


#define ESP32_CAN_TX_PIN GPIO_NUM_32  // Set CAN TX port 
#define ESP32_CAN_RX_PIN GPIO_NUM_34   // Set CAN RX port  // BTW, 4 is unavailable on Heltec

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>

#include <NMEA2000_CAN.h>     // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>      // NMEA2000

#include <HardwareSerial.h>
#include <Preferences.h>

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

bool loaded_calib = false;


// Persistent Data

Preferences prefs;

class PersistentData {
public:
  long sensor_id;  // If non-zero and equal to actual sensor ID, calibration_data is valid
  adafruit_bno055_offsets_t calibration_data;
  bool data_valid;
  Adafruit_BNO055::adafruit_bno055_axis_remap_config_t axis_config;
  Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t axis_sign;
  int node_address;  // For NMEA2K, default is 34
  float roll_offset;
  float pitch_offset;
  float temperature_offset;
  bool reload_cal_if_lost;

  void begin() {
    prefs.begin("compass_prefs");  
    prefs.getBytes("compass", this, sizeof(*this));
  }

  void init() {
    // Call commit() after this!

    sensor_id = 0;  // Mark calibration_data as invalid
    data_valid = true;

    //axis_config = Adafruit_BNO055::REMAP_CONFIG_P1;
    //axis_sign = Adafruit_BNO055::REMAP_SIGN_P1;

      // X = -Y, Y = Z, Z = -X  For mounting on forward side of vertical bulkhead, cable on right.
    axis_config = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)0x09;
    axis_sign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)0x05;

    node_address = 34;

    roll_offset = 0.0;
    pitch_offset = 0.0;
    temperature_offset = 0.0;
    reload_cal_if_lost = true;
  }


  void commit() {
    prefs.putBytes("compass", this, sizeof(*this));
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


// This class is both a r/w Characteristic and a set of callbacks for it.
template<typename DT> class MyDataCharacteristic : public BLECharacteristic, public BLECharacteristicCallbacks {
public:
  typedef std::function<void(DT)> SetFunc;
  typedef std::function<DT()> GetFunc;


  MyDataCharacteristic(BLEService *pService, const char *uuid, const char *descr, GetFunc getter, SetFunc setter) : 
    BLECharacteristic(BLEUUID(uuid), BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE),
    BLECharacteristicCallbacks(),
    _getter(getter),
    _setter(setter)
  {
     setValue(formatVal(_getter()));  // setValue() is in base class BLECharacteristic

    // Do this after setValue(), to avoid weird loops.
    setCallbacks(this);  // In base class BLECharacteristic

    addDescriptor(new My2901Descriptor(descr));
    pService->addCharacteristic(this);
  }
    
  void onWrite(BLECharacteristic *characteristic) override;  // Needs specialization
  std::string formatVal(DT val) const;  // Needs specialization
private:
  GetFunc _getter;
  SetFunc _setter;
};


typedef MyDataCharacteristic<uint8_t> MyByteDataCharacteristic;
typedef MyDataCharacteristic<float> MyFloatDataCharacteristic;




template<> std::string
MyByteDataCharacteristic::formatVal(uint8_t val) const
{
  char s[16];
  snprintf(s, sizeof(s), "%x", val);
  return std::string(s);
}

template<> void
MyByteDataCharacteristic::onWrite(BLECharacteristic *characteristic)
{
  if (characteristic == this) {
    uint8_t val;
    sscanf((char*)getData(), "%x", &val);  // Convert BLE payload from ascii hex string to char.
    if (_setter) _setter(val);
    setValue(formatVal(_getter()));
  } else {
    log_e("Improper structure of byteDataCharacteristic!");
  }
}


template<> std::string
MyFloatDataCharacteristic::formatVal(float val) const
{
  char s[16];
  snprintf(s, sizeof(s), "%4.2f", val);
  return std::string(s);
}

template<> void
MyFloatDataCharacteristic::onWrite(BLECharacteristic *characteristic)
{
  if (characteristic == this) {
    float val;
    sscanf((char*)getData(), "%f", &val);  // Convert BLE payload from ascii string to float.
    if (_setter) _setter(val);
    setValue(formatVal(_getter()));
  } else {
    log_e("Improper structure of floatDataCharacteristic!");
  }
}




// This is useable for any floating-point value...
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
class MyActionCharacteristic : public BLECharacteristic, public BLECharacteristicCallbacks {
public:
  typedef void(*ActionFunc)();

  MyActionCharacteristic(BLEService *pService, BLEUUID uuid, ActionFunc actor, const char *descr = nullptr) : 
    BLECharacteristic(uuid, BLECharacteristic::PROPERTY_WRITE),
    BLECharacteristicCallbacks(),
    _actor(actor)
  {
    setValue("");

    addDescriptor(new My2901Descriptor(descr ? descr : "action"));

    // Do this after setValue(), to avoid weird loops.
    setCallbacks(this);  // In base class BLECharacteristic
    pService->addCharacteristic(this);
  }
    
  void onWrite(BLECharacteristic *characteristic) override;

private:
  ActionFunc _actor;
};


void MyActionCharacteristic::onWrite(BLECharacteristic *characteristic)
{

  setValue("");  // Discard whatever value was written

  if (characteristic == this) {
    if (_actor) _actor();
  } else {
    log_e("Improper structure of actionCharacteristic!");
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
const char *TEMPERATURE_UUID  =  "cd3fb5aa-c679-4d3e-9eb4-bbe77c760f16";

const char *CALIBRATION_UUID =   "cd3fb5aa-c679-4d3e-9eb4-8ce31b0538c6";

const char *AXIS_CONFIG_UUID =   "cd3fb5aa-c679-4d3e-9eb4-85b6bfc15110";
const char *AXIS_SIGN_UUID =     "cd3fb5aa-c679-4d3e-9eb4-7ae7aed6bf55";

const char *ROLL_OFFSET_UUID =   "cd3fb5aa-c679-4d3e-9eb4-a0b507178d86";
const char *PITCH_OFFSET_UUID =  "cd3fb5aa-c679-4d3e-9eb4-361609541a10";
const char *TEMPERATURE_OFFSET_UUID =  "cd3fb5aa-c679-4d3e-9eb4-929b992324e2";

const char *CLEAR_CALIB_UUID =   "cd3fb5aa-c679-4d3e-9eb4-c12dcc1b4ccb";
const char *RESET_UUID =         "cd3fb5aa-c679-4d3e-9eb4-23587c136d2a";
const char *RELOAD_CAL_UUID =    "cd3fb5aa-c679-4d3e-9eb4-0251d9c95b48";


BLEServer *pServer(nullptr);
BLEService *pService(nullptr);
DeviceInformationService *pDeviceInfoService(nullptr);

MyAngleCharacteristic *pHeadingChar(nullptr);
MyAngleCharacteristic *pRollChar(nullptr);
MyAngleCharacteristic *pPitchChar(nullptr);
MyAngleCharacteristic *pTemperatureChar(nullptr);
MyStringCharacteristic *pCalibChar(nullptr);

MyFloatDataCharacteristic *pRollOffsetChar(nullptr);
MyFloatDataCharacteristic *pPitchOffsetChar(nullptr);
MyFloatDataCharacteristic *pTemperatureOffsetChar(nullptr);

MyByteDataCharacteristic *pAxisConfigChar(nullptr);
MyByteDataCharacteristic *pAxisSignChar(nullptr);
MyByteDataCharacteristic *pReloadCalChar(nullptr);


MyActionCharacteristic *pClearCalibChar(nullptr);
MyActionCharacteristic *pResetChar(nullptr);


MyBLEServerCallbacks serverCallbacks;


bool haveConnection(false);



void resetSystem()
{
  delay(1000);
  ESP.restart(); 
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



bool goodCalibration() {
    return mag_calib > 0;
}



// Set the information for other bus devices, which N2K messages we support: Heading and Attitude
const unsigned long transmitMessages[] PROGMEM = {127250L, 127571L, 0};

unsigned long loadCalibrationTime = 0;

bool loadCalibration() {

  sensor_t sensor;
  bno.getSensor(&sensor);

  if (persistentData.sensor_id != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    Serial.printf("\nSensor ID: %lx  Stored ID: %lx\n", sensor.sensor_id, persistentData.sensor_id);

    loaded_calib = false;
    return false;
  }
  
  Serial.println("\nFound Calibration for this sensor in EEPROM.");

  displaySensorOffsets(persistentData.calibration_data);
  Serial.println("\n\nRestoring Calibration data to the BNO055...");
  bno.setSensorOffsets(persistentData.calibration_data);
  Serial.println("\n\nCalibration data loaded into BNO055");

  loadCalibrationTime = millis();
  loaded_calib = true;
  return true;

}



void setup() {


  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);



  // This will use the default I2C Wire pins.
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("No BNO055 detected");
    while (1);
  }

  // Note: The ESP32 EEPROM library is used differently than the official Arduino version.
  persistentData.begin();


  if (!persistentData.data_valid) {
    // First-time initialization of persistent data
    persistentData.init();
    persistentData.commit();  
    Serial.println("Initialized persistent data");
  }

  bno.setAxisRemap(persistentData.axis_config);
  bno.setAxisSign(persistentData.axis_sign);
  Serial.printf("Set sensor axis data to 0x%x  0x%x\n", persistentData.axis_config, persistentData.axis_sign);
  

  /*
  *  Read the sensor's unique ID in the EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */

  loadCalibration();
  
  delay(1000);

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  // For NMEA0183 output on ESP32
  Serial2.begin(4800, SERIAL_8N1, 16 /*Rx pin*/, 17 /*Tx pin*/, true /*invert*/);

  NMEA0183 = new tNMEA0183(&Serial2);
  NMEA0183->Open();

  // Reserve enough buffer for sending all messages.
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  // Generate unique number from chip id
  uint8_t chipid[6];
  esp_efuse_mac_get_default(chipid);

  uint32_t id = 0;
  for (int i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  // Set product information
  NMEA2000.SetProductInformation("001",                // Manufacturer's Model serial code
                                  1,                   // Manufacturer's product code
                                 "Doug Compass",       // Manufacturer's Model ID
                                 __DATE__,             // Manufacturer's Software version code
                                 "1.0",                // Manufacturer's Model version
                                 1                     // Load Equivalency  (units of 50mA)
                                 );

   // Set device information
  NMEA2000.SetDeviceInformation(id,   // Unique number. Use e.g. Serial number.
                                140,  // Device function=Ownship Attitude. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                60,   // Device class=Navigation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2006  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                );


  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, persistentData.node_address);   // Read stored last NodeAddress, default 34
  NMEA2000.ExtendTransmitMessages(transmitMessages);
  Serial.print("setup 2\n");

  if (!NMEA2000.Open()) {
    Serial.println("NMEA2000.Open failed");
  }


  Serial.println("Starting BLE server!");

  // Hardware initialized here

  BLEDevice::init("DougCompass");
  pServer = BLEDevice::createServer();
  // Not needed at the moment  pServer->setCallbacks(&serverCallbacks);
  
  pDeviceInfoService = new DeviceInformationService(pServer);

  pService = pServer->createService(BLEUUID(SERVICE_UUID), 48 /*numHandles*/);

  
 
  Serial.println("resolution done!");

  pHeadingChar = new MyAngleCharacteristic(pService, HEADING_UUID, "heading value");
  pRollChar = new MyAngleCharacteristic(pService, ROLL_UUID, "roll value");
  pPitchChar = new MyAngleCharacteristic(pService, PITCH_UUID, "pitch value");
  pTemperatureChar = new MyAngleCharacteristic(pService, TEMPERATURE_UUID, "temperature value");

  pCalibChar = new MyStringCharacteristic(pService, CALIBRATION_UUID, "calibration");

  
  pClearCalibChar = new MyActionCharacteristic(pService, BLEUUID(CLEAR_CALIB_UUID), clearCalib, "clear calibration");
  pResetChar = new MyActionCharacteristic(pService, BLEUUID(RESET_UUID), resetSystem, "reset system");

 
  pAxisConfigChar = new MyByteDataCharacteristic(pService, AXIS_CONFIG_UUID, "axis config", 
                             []{ return (uint8_t)persistentData.axis_config; },
                             [](uint8_t val){ persistentData.axis_config = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)val; persistentData.commit();}
                          );


  pAxisSignChar = new MyByteDataCharacteristic(pService, AXIS_SIGN_UUID, "axis sign", 
                             []{ return (uint8_t)persistentData.axis_sign; },
                             [](uint8_t val){ persistentData.axis_sign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)val; persistentData.commit();}
                          );

  pRollOffsetChar = new MyFloatDataCharacteristic(pService, ROLL_OFFSET_UUID, "roll offset", 
                             []{ return persistentData.roll_offset; },
                             [](float val){ persistentData.roll_offset = val; persistentData.commit();}
                          );

  pPitchOffsetChar = new MyFloatDataCharacteristic(pService, PITCH_OFFSET_UUID, "pitch offset", 
                             []{ return persistentData.pitch_offset; },
                             [](float val){ persistentData.pitch_offset = val; persistentData.commit();}
                          );

  pTemperatureOffsetChar = new MyFloatDataCharacteristic(pService, TEMPERATURE_OFFSET_UUID, "temperature offset", 
                             []{ return persistentData.temperature_offset; },
                             [](float val){ persistentData.temperature_offset = val; persistentData.commit();}
                          );

  pReloadCalChar = new MyByteDataCharacteristic(pService, RELOAD_CAL_UUID, "reload cal if lost", 
                             []{ return (uint8_t)persistentData.reload_cal_if_lost; },
                             [](uint8_t val){ persistentData.reload_cal_if_lost = val; persistentData.commit();}
                          );


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


unsigned long nextUpdate = 0;

void loop() {

  // Detect and process a change in the BLE connection state.
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

  unsigned long curTime = millis();
  if (curTime > nextUpdate) {
    nextUpdate = curTime + 500;
  

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // In degrees
    float rawHeading = orientationData.orientation.x;
    float rawRoll = -orientationData.orientation.y;
    float rawPitch = -orientationData.orientation.z;

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
                          std::to_string(accel_calib)+"  M"+std::to_string(mag_calib)+
                          "  "+(goodCalibration() ? "OK" : "NG")+"  E"+std::to_string(persistentData.sensor_id);
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

      Serial.print("  NMEA Valid output: ");
      Serial.println(goodCalibration());
    }

 

    if (verbose) {
      Serial.print("Heading: ");
      Serial.print(rawHeading);
      Serial.print("  Roll (-y): ");
      Serial.print(rawRoll);
      Serial.print("  Pitch (-z): ");
      Serial.println(rawPitch);
    }

    double heading;
    if (goodCalibration()) {
      digitalWrite(LED_BUILTIN, HIGH);  // Indicate NMEA data transmission
      // The NMEA0183 API wants heading in radians
      heading = DegToRad(rawHeading);
    } else {
      heading = NMEA0183DoubleNA;  // Send a blank value  (N2KDoubleNA is the same value)

      if (loaded_calib && persistentData.reload_cal_if_lost && (curTime > loadCalibrationTime + 60000)) {
        Serial.println("\nCalibration lost, reloading saved calibration.");
        //resetSystem();
        loadCalibration();
        delay(1000);
        /* Crystal must be configured AFTER loading calibration data into BNO055. */
        bno.setExtCrystalUse(true);
      }
    }

    tNMEA0183Msg NMEA0183Msg;
    if ( NMEA0183SetHDM(NMEA0183Msg, heading, "HC") ) {  // HC means magnetic compass
      NMEA0183->SendMessage(NMEA0183Msg);
    }

    tN2kMsg N2kMsg;
    SetN2kMagneticHeading(N2kMsg, 0, heading);
    //Serial.println("sending CAN");
    NMEA2000.SendMsg(N2kMsg);

    // We send the actual roll/patch values even if the sensor is not calibrated.
    SetN2kAttitude(N2kMsg, 0, 0.0, DegToRad(rawPitch + persistentData.pitch_offset), DegToRad(rawRoll + persistentData.roll_offset));
    NMEA2000.SendMsg(N2kMsg);

    double rawTemperature = bno.getTemp();
    SetN2kTemperatureExt(N2kMsg, 0, 1 /*TempInstance*/, N2kts_InsideTemperature, CToKelvin(rawTemperature + persistentData.temperature_offset));
    NMEA2000.SendMsg(N2kMsg);

    // Check if SourceAddress has changed (due to address conflict on bus)

    if (NMEA2000.ReadResetAddressChanged()) {
      // Save potentially changed Source Address to NVS memory
      persistentData.node_address = NMEA2000.GetN2kSource();
      persistentData.commit();
    }

    // These BLE Characteristics are raw uncorrected sensor values
    pHeadingChar->setVal(rawHeading, haveConnection);
    pRollChar->setVal(rawRoll, haveConnection);
    pPitchChar->setVal(rawPitch, haveConnection);

    pTemperatureChar->setVal(rawTemperature, haveConnection);

        
    if (!loaded_calib && bno.isFullyCalibrated()) {

      Serial.println("\nFully calibrated!");
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");
      
      bno.getSensorOffsets(persistentData.calibration_data);

      sensor_t sensor;
      bno.getSensor(&sensor);
      persistentData.sensor_id = sensor.sensor_id;

      displaySensorOffsets(persistentData.calibration_data);
      Serial.println("\n\nStoring calibration data to EEPROM...");
      Serial.printf("\nSensor ID: %lx\n", sensor.sensor_id);
      
      persistentData.commit();
      Serial.println("Data stored to EEPROM.");
      loaded_calib = true;

      Serial.println("\n--------------------------------\n");
    }

    delay(5); //allow LED to blink and the cpu to switch to other tasks
    digitalWrite(LED_BUILTIN, LOW);

  }

  NMEA2000.ParseMessages();

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
  // Preserve all the other persistent parameters
  persistentData.sensor_id = 0L;
  persistentData.commit();
  Serial.println("Stored calibration invalidated.");
  resetSystem();
}
