
   /* Hardware pinout for ESP32 DEVKIT board

   Magnetometer:
   SDA = 21;
   SCL = 22;

   NMEA0183 out = 17

   */


#define ESP32_CAN_TX_PIN GPIO_NUM_32  // Set CAN TX port 
#define ESP32_CAN_RX_PIN GPIO_NUM_34   // Set CAN RX port  // BTW, 4 is unavailable on Heltec

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h" 

#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>

#include <NMEA2000_CAN.h>     // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>      // NMEA2000

#include <HardwareSerial.h>
#include <Preferences.h>

#include "esp_mac.h"

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



//BNO085 outputs radians and NMEA2000 takes radians, but messages and BLE ought to use degrees
constexpr double RAD_2_DEG = 180.0 / PI;
constexpr double DEG_2_RAD = 1 / RAD_2_DEG; 

BNO08x myIMU;
// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define BNO08X_ADDR 0x4A // Default for Adafruit BNO085 board - SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B


tNMEA0183* NMEA0183 = nullptr;


// Persistent Data

Preferences prefs;

class PersistentData {
public:
  bool data_valid;
  int reorientationIdx;
  int node_address;  // For NMEA2K, default is 34
  float roll_offset;
  float pitch_offset;
  float temperature_offset;

  void begin() {
    prefs.begin("compass_prefs");  
    prefs.getBytes("compass", this, sizeof(*this));
  }

  void init() {
    // Call commit() after this!

    data_valid = true;

    //reorientationIdx = 0x00;

    // Old arrangement:
    // For mounting on forward side of vertical bulkhead, cable runs to port.
    // Z points forward, X points up
    // reorientationIdx = 0x08;
    
    // New 11/2024 arrangement:
    // For mounting on rear side of vertical bulkhead, cable runs to port.
    // Z points aft, X points down
    reorientationIdx = 0x0e;

    node_address = 34;

    roll_offset = 0.0;
    pitch_offset = 0.0;
    temperature_offset = 0.0;
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
     setValue(formatVal(_getter()).c_str());  // setValue() is in base class BLECharacteristic

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
    setValue(formatVal(_getter()).c_str());
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
    setValue(formatVal(_getter()).c_str());
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

  void setVal(float val, bool noti=false) { setValue(std::to_string(val).c_str()); if (noti) notify(); }
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

  void setVal(std::string val, bool noti=false) { setValue(val.c_str()); if (noti) notify(); }
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
  _modelNumberCharacteristic->setValue("NMEA Compass 2.0");

  _firmwareRevCharacteristic = _service->createCharacteristic("2a26", BLECharacteristic::PROPERTY_READ);
  _firmwareRevCharacteristic->setValue(__DATE__);

  _service->dump();

  _service->start();
  
}



// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

const char *SERVICE_UUID  =      "4bd81659-41d3-46d2-9e84-bd7e9cae18ef";

const char *HEADING_UUID  =      "cd3fb5aa-c679-4d3e-9eb4-97912c27b298";
const char *ROLL_UUID  =         "cd3fb5aa-c679-4d3e-9eb4-3990fa52213b";
const char *PITCH_UUID  =        "cd3fb5aa-c679-4d3e-9eb4-f765e94d054b";
const char *TEMPERATURE_UUID  =  "cd3fb5aa-c679-4d3e-9eb4-bbe77c760f16";

const char *CALIBRATION_UUID =   "cd3fb5aa-c679-4d3e-9eb4-8ce31b0538c6";

const char *REORIENTATION_UUID = "cd3fb5aa-c679-4d3e-9eb4-85b6bfc15120";

const char *ROLL_OFFSET_UUID =   "cd3fb5aa-c679-4d3e-9eb4-a0b507178d86";
const char *PITCH_OFFSET_UUID =  "cd3fb5aa-c679-4d3e-9eb4-361609541a10";
const char *TEMPERATURE_OFFSET_UUID =  "cd3fb5aa-c679-4d3e-9eb4-929b992324e2";

const char *SAVE_CALIB_UUID =   "cd3fb5aa-c679-4d3e-9eb4-4181053fa198";
const char *CLEAR_CALIB_UUID =   "cd3fb5aa-c679-4d3e-9eb4-c12dcc1b4ccb";
const char *RESET_UUID =         "cd3fb5aa-c679-4d3e-9eb4-23587c136d2a";


BLEServer *pServer(nullptr);
BLEService *pService(nullptr);
DeviceInformationService *pDeviceInfoService(nullptr);

MyAngleCharacteristic *pHeadingChar(nullptr);
MyAngleCharacteristic *pRollChar(nullptr);
MyAngleCharacteristic *pPitchChar(nullptr);
MyAngleCharacteristic *pTemperatureChar(nullptr);
MyAngleCharacteristic *pCalibChar(nullptr);

MyFloatDataCharacteristic *pRollOffsetChar(nullptr);
MyFloatDataCharacteristic *pPitchOffsetChar(nullptr);
MyFloatDataCharacteristic *pTemperatureOffsetChar(nullptr);

MyByteDataCharacteristic *pReorientationChar(nullptr);

MyActionCharacteristic *pSaveCalibChar(nullptr);
MyActionCharacteristic *pClearCalibChar(nullptr);
MyActionCharacteristic *pResetChar(nullptr);


MyBLEServerCallbacks serverCallbacks;


bool haveConnection(false);



void resetSystem()
{
  delay(1000);
  ESP.restart(); 
}

  

void saveCalib()
{
  // The calibration is stored in the BNO085 chip's flash, not in the ESP32 flash.
  sh2_saveDcdNow();
  Serial.println("Current calibrarion stored.");
  resetSystem();
}


void clearCalib()
{
  // The calibration is stored in the BNO085 chip's flash, not in the ESP32 flash.
  sh2_clearDcdAndReset();
  Serial.println("Stored calibration invalidated.");
  resetSystem();
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


// Set the information for other bus devices, which N2K messages we support: Heading and Attitude
const unsigned long transmitMessages[] PROGMEM = {127250L, 127571L, 0};



int setReorientation(double w, double x, double y, double z) {
    
  sh2_Quaternion_t q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  Serial.printf("Orientation: %f  %f  %f  %f\n", w, x, y, z);

  return sh2_setReorientation(&q);
}

constexpr double S22 = sqrt(2)/2;


double orientationQuaternions[24][4] = {

  // Qw    Qx    Qy    Qz        Fwd   Up
  {   1,    0,    0,    0},  //    X    Z   (default)  
  { S22,    0,    0,  S22},  //   -Y    Z   Y points back, Z points up
  {   0,    0,    0,    1},  //   -X    Z
  { S22,    0,    0, -S22},  //    Y    Z

  {   0,    0,   -1,    0},  //    X   -Z   X points fwd, Z points down
  {   0, -S22, -S22,    0},  //    Y   -Z
  {   0,   -1,    0,    0},  //   -X   -Z
  {   0, -S22,  S22,    0},  //   -Y   -Z

  {   0,    0, -S22,  S22},  //    Z    X  Z points fwd, X points up
  { 0.5, -0.5, -0.5,  0.5},  //    Z    Y
  { S22, -S22,    0,    0},  //    Z   -X
  { 0.5, -0.5,  0.5, -0.5},  //    Z   -Y

  {-S22, -S22,    0,    0},  //   -Z    X  
  {-0.5, -0.5, -0.5, -0.5},  //   -Z   -Y   
  {   0,    0, -S22, -S22},  //   -Z   -X
  { 0.5,  0.5, -0.5, -0.5},  //   -Z    Y

  {-0.5, -0.5, -0.5,  0.5},  //    Y    X  
  {   0, -S22,    0,  S22},  //   -X    Y
  { 0.5, -0.5,  0.5,  0.5},  //   -Y   -X
  {-S22,    0, -S22,    0},  //    X   -Y

  { 0.5,  0.5, -0.5,  0.5},  //   -Y    X
  {   0, -S22,    0, -S22},  //   -X   -Y
  { 0.5, -0.5, -0.5, -0.5},  //    Y   -X
  { S22,    0, -S22,    0}   //    X    Y
};


int setReorientation(int idx) {
  if (idx < 0 || idx > 23) {
    Serial.printf("Bad reorientation idx %d!\n", idx);
    return setReorientation(0);
  }

  double *q = &(orientationQuaternions[idx][0]);
  return setReorientation(q[0], q[1], q[2], q[3]);
}


// Here is where you define the sensor outputs you want to receive
void configSensor(void) {
  
  int ret = setReorientation(persistentData.reorientationIdx);
  Serial.printf("setReorientation returns %x\n", ret);

  Serial.println("Setting desired reports");
  if (myIMU.enableGeomagneticRotationVector() == true) {
  //if (myIMU.enableRotationVector(60) == true) {
    Serial.println(F("Geomagnetic Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable geomagnetic rotation vector");
  }

}




void setup() {

  delay(100); // Allow BNO085 to startup

  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // This will use the default I2C Wire pins.
  Wire.begin();
  //Wire.setClockStretchLimit(4000);

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected. Retrying...");
    delay(100);
    Wire.begin();
    myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST);
  }
  Serial.println("BNO08x found!");

  
  // Note: The ESP32 EEPROM library is used differently than the official Arduino version.
  persistentData.begin();


  if (!persistentData.data_valid) {
    // First-time initialization of persistent data
    persistentData.init();
    persistentData.commit();  
    Serial.println("Initialized persistent data");
  }

  
  Serial.printf("Set sensor reorientation to 0x%x\n", persistentData.reorientationIdx);
  
  configSensor();
  
  
  delay(1000);

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
                                 "2.0",                // Manufacturer's Model version
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

  pCalibChar = new MyAngleCharacteristic(pService, CALIBRATION_UUID, "calibration accuracy");

  pSaveCalibChar = new MyActionCharacteristic(pService, BLEUUID(SAVE_CALIB_UUID), saveCalib, "save calibration");
  pClearCalibChar = new MyActionCharacteristic(pService, BLEUUID(CLEAR_CALIB_UUID), clearCalib, "clear calibration");
  pResetChar = new MyActionCharacteristic(pService, BLEUUID(RESET_UUID), resetSystem, "reset system");

 
  pReorientationChar = new MyByteDataCharacteristic(pService, REORIENTATION_UUID, "reorientation", 
                             []{ return (uint8_t)persistentData.reorientationIdx; },
                             [](uint8_t val){ persistentData.reorientationIdx = val; persistentData.commit();}
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

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    configSensor();
  }

  // Try to pull an event off the queue
  bool haveEvent = myIMU.getSensorEvent();
  
  // Has a new event come in on the Sensor Hub Bus?
  // This will almost always be true - probably several events have come in.
  if (haveEvent) {
    // These are in radians
    float roll = myIMU.getRoll();   // Positive roll is to the right, around the device X axis
    float pitch = -myIMU.getPitch(); // Pitch is around the device Y axis. Negate so that positive pitch is upwards
    float yaw = myIMU.getYaw();  // Zero yaw is (magnetic) east, positive yaw is to the left!
    uint8_t accuracy = myIMU.getMagAccuracy();
    
    // is it the correct sensor data we want?
    bool eventOK = (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR ||
                    myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR);


    unsigned long curTime = millis();
    if (curTime > nextUpdate) {
      nextUpdate = curTime + 500;
    
      // Map yaw to a proper nonnegative compass heading.
      float heading = -yaw + (PI/2);
      if (heading < 0.0) {
        heading += 2*PI;
      }

      // Get values in degrees for messages
      float degRoll = RadToDeg(roll);
      float degPitch = RadToDeg(pitch);
      float degYaw = RadToDeg(yaw);
      float degHeading = RadToDeg(heading);

      pCalibChar->setVal(accuracy, haveConnection);
          
      
      if (verbose) {
        Serial.print("Yaw: ");
        Serial.print(degYaw);
        Serial.print("  Heading: ");
        Serial.print(degHeading);
        Serial.print("  Roll: ");
        Serial.print(degRoll);
        Serial.print("  Pitch: ");
        Serial.print(degPitch);
        Serial.print("  Calibration accuracy: ");
        Serial.println(accuracy);
      }

      if (accuracy > 1) {
        digitalWrite(LED_BUILTIN, HIGH);  // Indicate NMEA data transmission
      } else {
        heading = NMEA0183DoubleNA;  // Send a blank value  (N2KDoubleNA is the same value)
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
      SetN2kAttitude(N2kMsg, 0, 0.0, pitch + DegToRad(persistentData.pitch_offset), roll + DegToRad(persistentData.roll_offset));
      NMEA2000.SendMsg(N2kMsg);

      double rawTemperature = 0.0;   // Available from BNO085?
      SetN2kTemperatureExt(N2kMsg, 0, 1 /*TempInstance*/, N2kts_InsideTemperature, CToKelvin(rawTemperature + persistentData.temperature_offset));
      NMEA2000.SendMsg(N2kMsg);

      // Check if SourceAddress has changed (due to address conflict on bus)
      if (NMEA2000.ReadResetAddressChanged()) {
        // Save potentially changed Source Address to NVS memory
        persistentData.node_address = NMEA2000.GetN2kSource();
        persistentData.commit();
      }

      // Unlike the NMEA values, these BLE Characteristics are un-offset sensor values
      pHeadingChar->setVal(degHeading, haveConnection);
      pRollChar->setVal(degRoll, haveConnection);
      pPitchChar->setVal(degPitch, haveConnection);

      pTemperatureChar->setVal(rawTemperature, haveConnection);

          
      delay(5); //allow LED to blink and the cpu to switch to other tasks
      digitalWrite(LED_BUILTIN, LOW);

    }
  }

  NMEA2000.ParseMessages();

}



