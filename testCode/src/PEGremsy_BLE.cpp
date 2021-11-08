#include "Arduino.h"
#include "PEGremsy_BLE.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include <EEPROM.h>
#include "gimbalParam.h"

/*
 * Battery Service
 * */
#define BLE_UUID_BATTERY_SERVICE				"180F"
#define BLE_UUID_BATTERY_LEVEL_CHAR				"2A19"

/*
 * jigTest Service
 */
#define BLE_UUID_JIGTEST_SERVICE				"0A630100-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_JIG_STATUS_CHAR				"0A630101-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_PRODUCT_ON_JIG_STATUS_CHAR     "0A630102-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_JIG_QC_MODE_CHAR               "0A630103-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_JIG_CONTROL_CHAR               "0A630104-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_PRODUCT_PROFILE_CHAR           "0A630105-47B5-4779-8699-054D0879FC69"

/*
 * heartbeat Service
 */
#define BLE_UUID_MAVLINK_SERVICE				"0A630201-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_HEARTBEAT_CHAR					"0A630202-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_PARAMVALUE_CHAR				"0A630203-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_RAWIMU_CHAR					"0A630204-47B5-4779-8699-054D0879FC69"

/*
 * Gimbal Service
 */


/**
 * Information Service
 */
#define BLE_UUID_DEVICE_INFORMATION_SERVICE     "180A"
#define BLE_UUID_MANUFACTURER_NAME_STRING_CHAR  "2A29"
#define BLE_UUID_MODEL_NUMBER_STRING_CHAR       "2A24"
#define BLE_UUID_SERIAL_NUMBER_STRING_CHAR      "2A25"
#define BLE_UUID_FIRMWARE_REVISION_STRING_CHAR  "2A26"
#define BLE_UUID_HARDWARE_REVISION_STRING_CHAR  "2A27"
#define BLE_UUID_PNP_ID_CHAR                    "2A50"
/*
 * Base service
 */
#define BLE_BASE_UUID                            "0A630000-47B5-4779-8699-054D0879FC69"//"62610000-f5ea-490e-8548-e6821e3e7792"
#define BLE_ADV_UUID                             "0A63CB01-47B5-4779-8699-054D0879FC69"//"6261CB01-f5ea-490e-8548-e6821e3e7792"

#define MANUFACTURER            "GREMSY"
#define MODEL_NUMBER            "JIGT3-VN"
#define SERIAL_NUMBER           "JT3-7E5-A-001"
#define FIRMWARE_VERSION        "0.1.21110101"
#define HARDWARE_VERSION        "0.1.21110102"

#define BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG  0x01
#define BLE_DIS_VENDOR_ID_LSB                0x63
#define BLE_DIS_VENDOR_ID_MSB                0x0A
#define BLE_DIS_PRODUCT_ID_LSB               0x01
#define BLE_DIS_PRODUCT_ID_MSB               0x00
#define BLE_DIS_PRODUCT_VERSION_LSB          0x02
#define BLE_DIS_PRODUCT_VERSION_MSB          0x00

#define DEVICE_GROUP      0x01
#define DEVICE_MODEL      0x03

#define FLASH_MEMORY_SIZE 10

#define SIMULATION_PROFILE  1

typedef struct 
{
    /* data */
    uint8_t state;
    JigTestStatus_t si_JigStatus;
    ProductStatus_t si_ProductStatus;
    ProductOnJigTestStatus_t si_ProductOnJigTestStatus;
    JigTestQcMode_t si_QcMode;
    JigTestQcModeStatus_t si_QcModeStatus;
    jigControl_t si_JigControl;
    bool deviceDisconnected;

}bleProfileSimulation_t;


/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
static const char *TAG = "BLE";

uint8_t PnPData[7] = {
BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG,
BLE_DIS_VENDOR_ID_LSB,
BLE_DIS_VENDOR_ID_MSB,
BLE_DIS_PRODUCT_ID_LSB,
BLE_DIS_PRODUCT_ID_MSB,
BLE_DIS_PRODUCT_VERSION_LSB,
BLE_DIS_PRODUCT_VERSION_MSB
};

char advertising_data[4] = {
BLE_DIS_VENDOR_ID_LSB,
BLE_DIS_VENDOR_ID_MSB,
DEVICE_GROUP,
DEVICE_MODEL
};

bool isConnect;
bool oldDeviceConnected;

size_t BLE_address[2];
char deviceName[100];

extern mavlinkHandle_t mavlink;
extern taskManagement_t management;
extern mavlinkHandle_t mavlink;

bleProfileSimulation_t simulation_ble;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
/** @group PEGremsy_BLE_CONFIGURATION
    @{
*/#ifndef PEGremsy_BLE_CONFIGURATION
#define PEGremsy_BLE_CONFIGURATION

/** @brief get buffer reciever
    @return
*/
void PEGremsy_BLE_getRecieverBuffer(uint8_t length, uint8_t maxLengthPacket, uint8_t *bufferReciever, std::string value, uint8_t count)
{
    if(length == maxLengthPacket)
    {
        bufferReciever[count] = value[count];
    }
}

/** @brief CallbackConnect
    @return 
*/
class CallbackConnect: public BLEServerCallbacks {

    /*
     * This function will be called when device disconnect
     */
    void onDisconnect(BLEServer *pBLEServer) {
        ESP_LOGI(TAG,"Disconnected");

        BLEDevice::startAdvertising();

        Serial1.println("[     BLE     ] : Disconnected");

        isConnect = false;
    }

    /*
     * This function will be called when device connect
     */
    void onConnect(BLEServer *pBLEServer) {
        char address[6];

        ESP_LOGI(TAG,"Connected");
        ESP_LOGI(TAG,"Connected Id %d - Count %d \n", pBLEServer->getConnId(), pBLEServer->getConnectedCount());

        BLEAddress addr = BLEDevice::getAddress();

        Serial.printf("[     BLE     ] : Connected Id : %s | Count : %d\n", addr.toString().c_str(), pBLEServer->getConnectedCount());

        if (pBLEServer->getConnectedCount() < 1) {
            BLEDevice::startAdvertising();
        }
        BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_CONN_HDL0);

        isConnect = true;
    }
};

/** @brief writeCallbacks
    @return
*/
class writeCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string value = pCharacteristic->getValue();

//      memset(bufferRecieverHeartBeat, 0, 10);

      if (value.length() > 0)
      {
        Serial.println("*********");
        

        BLEUUID jigCharUuId = BLEUUID(BLE_UUID_JIG_CONTROL_CHAR);
        BLEUUID productProfile = BLEUUID(BLE_UUID_PRODUCT_PROFILE_CHAR);
        BLEUUID UuIdGet = pCharacteristic->getUUID();

        if(UuIdGet.equals(jigCharUuId) == true)
        {
            Serial.print("BLE_UUID_JIG_CONTROL_CHAR - ");
            Serial.print("New value: ");
            for (int i = 0; i < value.length(); i++)
            {
                PEGremsy_BLE_getRecieverBuffer(value.length(), 1, management.JigControlBuffer, value, i);
                
                Serial.print(management.JigControlBuffer[i], HEX);
                // Serial.println("");
                // Serial.print(value[i], HEX);
            }
        }
        else if(UuIdGet.equals(productProfile) == true)
        {
            Serial.print("BLE_UUID_PRODUCT_PROFILE_CHAR - ");
            Serial.print("New value: ");
            for (int i = 0; i < value.length(); i++)
            {
                PEGremsy_BLE_getRecieverBuffer(value.length(), 1, management.productProfileBuffer, value, i);
                
                Serial.print(management.productProfileBuffer[i], HEX);
            }
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};

/** @brief AdvertisedDeviceCallbacks
    @return
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      memcpy(deviceName, advertisedDevice.toString().c_str(), advertisedDevice.toString().length());
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

/** @brief
    @return
*/
PEGremsy_BLE::PEGremsy_BLE(void)
{

}

/** @brief
    @return
*/
PEGremsy_BLE::~PEGremsy_BLE(void)
{

}

void printDeviceAddress() 
{
 
  const uint8_t* point = esp_bt_dev_get_address();
 
  for (int i = 0; i < 6; i++) 
  {
    char str[3];

    if(i == 4)
    {
        BLE_address[0] = point[4];
    }

    if(i == 5)
    {
        BLE_address[1] = point[5];
    }
 
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
 
    if (i < 5)
    {
      Serial.print(":");
    }
  }

  Serial.println("[BLE_address] = " + String(BLE_address[0]) + String(BLE_address[1]));

    EEPROM.write(0, BLE_address[0]);
    EEPROM.write(1, BLE_address[1]);
    EEPROM.commit();
}

/** @brief  CharacteristicsJigStatus_Initialize
    @return none
*/
void PEGremsy_BLE::CharacteristicsJigStatus_Initialize(BLEServer* pServer)
{
    /// Create service
    BLEService *pJIGTESTService = pServer->createService(BLE_UUID_JIGTEST_SERVICE);

    /// create Characteristics
    pCharacteristicsJigStatus           = pJIGTESTService->createCharacteristic(BLE_UUID_JIG_STATUS_CHAR, BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristicsProductOnJigStatus  = pJIGTESTService->createCharacteristic(BLE_UUID_PRODUCT_ON_JIG_STATUS_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicsJigQcMode           = pJIGTESTService->createCharacteristic(BLE_UUID_JIG_QC_MODE_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicsJigControl          = pJIGTESTService->createCharacteristic(BLE_UUID_JIG_CONTROL_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    /// set write callBack
    pCharacteristicsJigControl->setCallbacks(new writeCallbacks());

    /// create product profile Characteristis
    pCharacteristicsProductProfile      = pJIGTESTService->createCharacteristic(BLE_UUID_PRODUCT_PROFILE_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    /// set write callBack
    pCharacteristicsProductProfile->setCallbacks(new writeCallbacks());

    /// start jigtest service
    pJIGTESTService->start();
}

/** @brief  CharacteristicsMavlink_Initialize
    @return none
*/
void PEGremsy_BLE::CharacteristicsMavlink_Initialize(BLEServer* pServer)
{
    BLEService *pServiceMavlink = pServer->createService(BLE_UUID_MAVLINK_SERVICE);

    pCharacteristicsHeartbeat = pServiceMavlink->createCharacteristic(BLE_UUID_HEARTBEAT_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicsHeartbeat->setCallbacks(new writeCallbacks());

    pCharacteristicsParamValue = pServiceMavlink->createCharacteristic(BLE_UUID_PARAMVALUE_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicsParamValue->setCallbacks(new writeCallbacks());

    pCharacteristicsRawImu = pServiceMavlink->createCharacteristic(BLE_UUID_RAWIMU_CHAR, BLECharacteristic::PROPERTY_NOTIFY);

    pServiceMavlink->start();
}

/** @brief  CharacteristicsDeviceInfo_Initialize
    @return none
*/
void PEGremsy_BLE::CharacteristicsDeviceInfo_Initialize(BLEServer* pServer)
{
    BLEService *pInfoDeviceService = pServer->createService(BLEUUID(BLE_UUID_DEVICE_INFORMATION_SERVICE));

    /// create manufacture Characteristics
    pCharacterNameManufacture = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_MANUFACTURER_NAME_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    /// create model number Characteristics
    pCharacterModelNumber = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_MODEL_NUMBER_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    /// create serial number Characteristics
    pCharacterSerialNumber = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_SERIAL_NUMBER_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    /// create firmware version Characteristics
    pCharacterFirmwareVerison = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_FIRMWARE_REVISION_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    /// create hardware version Characteristics
    pCharacterHardwareVerison = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_HARDWARE_REVISION_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    // /// create PnpId Characteristics
    // pCharacterPnPId = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_PNP_ID_CHAR), BLECharacteristic::PROPERTY_READ);

    /// start device info service
    pInfoDeviceService->start();

    /// set value mane Manufacture
    pCharacterNameManufacture->setValue(MANUFACTURER);

    /// set value mane module number
    pCharacterModelNumber->setValue(MODEL_NUMBER);

    /// set value mane serial number
    pCharacterSerialNumber->setValue(SERIAL_NUMBER);

    /// set value mane firmware version
    pCharacterFirmwareVerison->setValue(FIRMWARE_VERSION);

    /// set value mane hardware version
    pCharacterHardwareVerison->setValue(HARDWARE_VERSION);

    // /// set value mane PnPId
    // pCharacterPnPId->setValue(PnPData, 7);
}

/** @brief  scan BLE Device
    @return none
*/
void PEGremsy_BLE::enableScanDevice(void)
{
  BLEScan* pBLEScan;

  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value   

  BLEScanResults foundDevices = pBLEScan->start(5, false); // 5s
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
}

/** @brief  initialize
    @return none
*/
void PEGremsy_BLE::initialize(void)
{
    String BLE_name = "JIG ";

    ESP_LOGI(TAG," Init BLE");

    EEPROM.begin(FLASH_MEMORY_SIZE);

    /// read ble address from eeprom
    BLE_address[0] = EEPROM.read(0);
    BLE_address[1] = EEPROM.read(1);

    if(BLE_address[0] != 255 && BLE_address[1] != 255)
    {

        Serial.println("[BLE_address] = " + String(BLE_address[0]) + String(BLE_address[1]));

        String temp = BLE_name + String(DecToHex(BLE_address[0])) + String(DecToHex(BLE_address[1]));

        Serial.println("[BLE_name] = " + String(DecToHex(BLE_address[0])) + String(DecToHex(BLE_address[1])) + " | " + temp);

        /// init BLE
        BLEDevice::init(temp.c_str());
    }
    else
    {
        /// init BLE
        BLEDevice::init(BLE_name.c_str());

        printDeviceAddress();

        delay(1000);

        ESP.restart();
    }

    // enableScanDevice();

    /// set power
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);

    /// create ble server
    BLEServer* pServer = BLEDevice::createServer();

    /// set MTU
    BLEDevice::setMTU(512);

    /// set connect Callback
    pServer->setCallbacks(new CallbackConnect());

    /*
     * Create Battery Service and Characteristics
     */
    // BLEService *pBatteryService = pServer->createService(BLEUUID(BLE_UUID_BATTERY_SERVICE));

    // /// create battery Characteristics
    // pCharacterBatteryLevel = pBatteryService->createCharacteristic(BLEUUID(BLE_UUID_BATTERY_LEVEL_CHAR), BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

    // pDescriptorBatteryLevel = new BLEDescriptor(BLEUUID("2901"));
    // pCharacterBatteryLevel->addDescriptor(pDescriptorBatteryLevel);

    // /// start battery Service
    // pBatteryService->start();

    // pDescriptorBatteryLevel->setValue("value");

    /*
     * Create JIGTEST Service and Characteristics
     */
    CharacteristicsJigStatus_Initialize(pServer);

    /*
     * Create mavlink Service and Characteristics
     */
    // CharacteristicsMavlink_Initialize(pServer);
    /*
     * Create Info Device Service and Characteristics
     */
    CharacteristicsDeviceInfo_Initialize(pServer);

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    BLEAdvertisementData dataAdvertisement;
    dataAdvertisement.setManufacturerData(std::string(advertising_data, 4));
    pAdvertising->addServiceUUID(BLE_BASE_UUID);

    pAdvertising->setAdvertisementData(dataAdvertisement);

    BLEDevice::startAdvertising();

    Serial.println("");
    Serial.println("start advertising");

    pinMode(22, OUTPUT);

}

/** @brief  initialize
    @return none
*/
void PEGremsy_BLE::deinit(void)
{
    BLEDevice::deinit(true);
}

/** @brief  heartBeatHandle
    @return none
*/
bool PEGremsy_BLE::heartBeatHandle(void)
{
    bool ret = false;

//     heartBeat.flag_heartbeat = bufferRecieverHeartBeat[9];

//     if(heartBeat.flag_heartbeat == true)
//     {
//         heartBeat.custom_mode       = bufferRecieverHeartBeat[3] << 24 | bufferRecieverHeartBeat[2] << 16 | bufferRecieverHeartBeat[1] << 8 | bufferRecieverHeartBeat[0];
//         heartBeat.type              = bufferRecieverHeartBeat[4];
//         heartBeat.autopilot         = bufferRecieverHeartBeat[5];
//         heartBeat.base_mode         = bufferRecieverHeartBeat[6];
//         heartBeat.system_status     = bufferRecieverHeartBeat[7];
//         heartBeat.mavlink_version   = bufferRecieverHeartBeat[8];

// //      memset(bufferRecieverHeartBeat, 0, 10);
//         heartBeat.flag_heartbeat = false;

//         char buff[300];
//         sprintf(buff, "custom_mode : %10d | type : %3d | autopilot : %3d | base_mode : %3d | system_status : %3d | mavlink_version : %3d"
//                 , heartBeat.custom_mode, heartBeat.type, heartBeat.autopilot, heartBeat.base_mode, heartBeat.system_status, heartBeat.mavlink_version );
//         Serial.println("[recieverServiceHearBeat] : " + String(buff));

//         ret = true;
//     }

    return ret;
}

/** @brief  send_jigStatus
    @return none
*/
void PEGremsy_BLE::send_jigStatus(bool Notify)
{
    // static uint32_t timeDebug = 0;

    #if (SIMULATION_PROFILE == 1)

        uint8_t jigStatusBuffer[2] = {(uint8_t)simulation_ble.si_JigStatus, (uint8_t)simulation_ble.si_ProductStatus};

    #else

        JigTestStatus_t jigStatus = management.getJigStatus();
        ProductStatus_t productStatus = management.getProductStatus();

        uint8_t jigStatusBuffer[2] = {(uint8_t)jigStatus, (uint8_t)productStatus};

    #endif

    if(flagSendJigStatus == true)
    {
        flagSendJigStatus = false;

        pCharacteristicsJigStatus->setValue(jigStatusBuffer, 2);
        pCharacteristicsJigStatus->notify(true);
    }
}

/** @brief  send_ProductOnJigStatus
    @return none
*/
void PEGremsy_BLE::send_ProductOnJigStatus(bool Notify)
{
    #if (SIMULATION_PROFILE == 1)

        uint8_t productOnStatusBuffer[1] = {(uint8_t)simulation_ble.si_ProductOnJigTestStatus};

    #else
        ProductOnJigTestStatus_t productOnStatus = management.getProductOnJigTestStatus();
        uint8_t productOnStatusBuffer[1] = {(uint8_t)productOnStatus};
    #endif

    if(flagSendProductOnJigStatus == true)
    {
        flagSendProductOnJigStatus = false;

        pCharacteristicsProductOnJigStatus->setValue(productOnStatusBuffer, 1);
        pCharacteristicsProductOnJigStatus->notify(true);
    }
}

/** @brief  send_jigQcMode
    @return none
*/
void PEGremsy_BLE::send_jigQcMode(bool Notify)
{
    #if (SIMULATION_PROFILE == 1)

    uint8_t qcModeBuffer[2] = {(uint8_t)simulation_ble.si_QcMode, (uint8_t)simulation_ble.si_QcModeStatus};

    #else

        JigTestQcMode_t qcMode = management.getQcMode();
        JigTestQcModeStatus_t qcModeStatus = management.getQcModeStatus();
        uint8_t qcModeBuffer[2] = {(uint8_t)qcMode, (uint8_t)qcModeStatus};

    #endif

    if(flagSendJigQcMode == true)
    {
        flagSendJigQcMode = false;

        pCharacteristicsJigQcMode->setValue(qcModeBuffer, 2);
        pCharacteristicsJigQcMode->notify(true);
    }
}

/** @brief  send_jigControl
    @return none
*/
void PEGremsy_BLE::send_jigControl(void)
{
    jigControl_t control = management.getJigControl();
    uint8_t controlBuffer[1] = {(uint8_t)control};

    if(flagSendJigControl == true)
    {
        flagSendJigControl = false;

        pCharacteristicsJigControl->setValue(controlBuffer, 1);
    }  

    // pCharacteristicsJigControl->setValue(controlBuffer, 1); 
}

/** @brief  send_jigControl
    @return none
*/
void PEGremsy_BLE::send_productProfile(void)
{
    uint8_t value = (uint8_t)gimbalParam_valueTestPixy[management.productProfileBuffer[0]].value;
    uint8_t productProfileBuffer[2] = {1, value};

    if(flagSendProductProfile == true)
    {
        flagSendProductProfile = false;

        /// delete reciver buffer
        management.productProfileBuffer[0] = 0;

        Serial.printf("[send_productProfile] value : %d", value);

        pCharacteristicsProductProfile->setValue(productProfileBuffer, 2);
    }
}
/** @brief  process
    @return none
*/
void PEGremsy_BLE::process(void)
{
    static uint32_t timeSequence;
    static bool state = false;
    static uint16_t countLedBlink = 0;

    if(isConnect == true)
    {
        heartBeatHandle();

        #if ( SIMULATION_PROFILE == 1 )

            static uint32_t timeSimulation = 0;
            static uint8_t simulationCount = 0;
            static bool firstRun = false;
            static bool modeError = false;

            switch (simulation_ble.state)
            {
                case 0/* constant-expression */:
                    /* code */
                {
                    static bool sendFirstJigStatus = false;

                    if(simulation_ble.deviceDisconnected == false)
                    {
                        if(millis() - timeSimulation > 1000)
                        {
                            timeSimulation = millis();

                            simulationCount++;

                            sendFirstJigStatus = false;

                            Serial.printf("count : %d\n", simulationCount);
                        }

                        if(simulationCount >= 4)
                        {
                            simulation_ble.si_JigStatus = JIG_STATUS_STANBY;
                            simulation_ble.si_ProductStatus = JIG_STATUS_A_PRODUCT_ATTACHED;

                            /// next state
                            simulation_ble.state = 1;
                            timeSimulation = 0;
                            simulationCount = 0;

                            if(firstRun == false)
                            {
                                simulation_ble.si_ProductOnJigTestStatus = PRODUCT_WAIT_FOR_QC;
                            }
                            

                            /// set flag jig status
                            flagSendJigStatus = true;

                            /// set flag product on status
                            flagSendProductOnJigStatus = true;

                            /// reset variables send first
                            sendFirstJigStatus = false;

                            Serial.printf("state : %d | jigstatus : %d | productStatus : %d | ProductOnStatus : %d | qcMode : %d | qcModeState : %d | controlStatus : %d\n",
                            simulation_ble.state,
                            simulation_ble.si_JigStatus,
                            simulation_ble.si_ProductStatus,
                            simulation_ble.si_ProductOnJigTestStatus,
                            simulation_ble.si_QcMode,
                            simulation_ble.si_QcModeStatus,
                            simulation_ble.si_JigControl
                            );
                        }
                        else
                        {
                            if(sendFirstJigStatus == false)
                            {
                                sendFirstJigStatus = true;

                                simulation_ble.si_JigStatus = JIG_STATUS_ERROR;
                                simulation_ble.si_ProductStatus = JIG_STATUS_NO_PRODUCT_ATTACHED;

                                /// set flag jig status
                                flagSendJigStatus = true;

                                /// set flag product on status
                                flagSendProductOnJigStatus = true;

                                /// set flag send jig qcMode
                                flagSendJigQcMode = true;
                            }
                        }
                    }
                    else
                    {
                        simulation_ble.deviceDisconnected = false;

                        simulation_ble.si_JigStatus = JIG_STATUS_STANBY;
                        simulation_ble.si_ProductStatus = JIG_STATUS_A_PRODUCT_ATTACHED;

                        delay(2000);

                        /// next state
                        simulation_ble.state = 1;
                        timeSimulation = 0;
                        simulationCount = 0;

                        if(firstRun == false)
                        {
                            simulation_ble.si_ProductOnJigTestStatus = PRODUCT_WAIT_FOR_QC;
                        }
                        
                        /// set flag jig status
                        flagSendJigStatus = true;

                        /// set flag product on status
                        flagSendProductOnJigStatus = true;

                        /// reset variables send first
                        sendFirstJigStatus = false;

                        Serial.printf("reConnect state : %d | jigstatus : %d | productStatus : %d | ProductOnStatus : %d | qcMode : %d | qcModeState : %d | controlStatus : %d\n",
                        simulation_ble.state,
                        simulation_ble.si_JigStatus,
                        simulation_ble.si_ProductStatus,
                        simulation_ble.si_ProductOnJigTestStatus,
                        simulation_ble.si_QcMode,
                        simulation_ble.si_QcModeStatus,
                        simulation_ble.si_JigControl
                        );
                    }
                }break;
                case 1 :
                {
                    static uint8_t simulationQcMode = 0;
                    static bool randomResult = false;
                    

                    /// enable send product Profile
                    if((uint8_t)simulation_ble.si_ProductOnJigTestStatus > 1)
                    {
                        switch (management.productProfileBuffer[0])
                        {
                            case 1:
                            {
                                flagSendProductProfile = true;

                                // uint8_t value = (uint8_t)gimbal_param_setting[management.productProfileBuffer[0]].value;
                                // uint8_t productProfileBuffer[2] = {1, value};

                                // Serial.printf("[send_productProfile] value : %d", value);

                                // pCharacteristicsProductProfile->setValue(productProfileBuffer, 2);

                                // /// delete reciver buffer
                                // management.productProfileBuffer[0] = 0;
                            }break;
                            case 2:
                            {
                                flagSendProductProfile = true;

                                // uint8_t value = (uint8_t)gimbal_param_setting[management.productProfileBuffer[0]].value;
                                // uint8_t productProfileBuffer[2] = {1, value};

                                // Serial.printf("[send_productProfile] value : %d", value);

                                // pCharacteristicsProductProfile->setValue(productProfileBuffer, 2);

                                // /// delete reciver buffer
                                // management.productProfileBuffer[0] = 0;
                            }break;
                            
                            default:
                                break;
                        }
                    }

                    simulation_ble.si_JigControl = management.getJigControl();

                    if(simulation_ble.si_JigControl == JIG_CONTROL_STOP)
                    {
                        if(simulation_ble.si_QcMode > 0)
                        {
                            /// next state
                            simulation_ble.state = 0;

                            memset(&simulation_ble, 0, sizeof(bleProfileSimulation_t));

                            /// send jig Control
                            flagSendJigControl = true;

                            /// send jig Control
                            // send_jigControl();
                            // delay(5);
                        }

                        timeSimulation = 0;
                        simulationCount = 0;
                        simulationQcMode = 0;
                    }
                    else if(simulation_ble.si_JigControl == JIG_CONTROL_START || simulation_ble.si_JigControl == JIG_CONTROL_RESUME)
                    {
                        static bool sendFirstJigStatusRunning = false;
                        

                        simulation_ble.si_ProductOnJigTestStatus = PRODUCT_RUNNING;
                        simulation_ble.si_JigStatus = JIG_STATUS_RUNNING;

                        if(sendFirstJigStatusRunning == false)
                        {
                            sendFirstJigStatusRunning = true;

                            /// set flag jig status
                            flagSendJigStatus = true;

                            /// set flag product on status
                            flagSendProductOnJigStatus = true;

                            /// set flag send jig control
                            flagSendJigControl = true;
                        }

                        if(millis() - timeSimulation > 1000)
                        {
                            timeSimulation = millis();

                            simulationCount++;

                            Serial.printf("state : %d | jigstatus : %d | productStatus : %d | ProductOnStatus : %d | qcMode : %d | qcModeState : %d | controlStatus : %d\n",
                            simulation_ble.state,
                            simulation_ble.si_JigStatus,
                            simulation_ble.si_ProductStatus,
                            simulation_ble.si_ProductOnJigTestStatus,
                            simulation_ble.si_QcMode,
                            simulation_ble.si_QcModeStatus,
                            simulation_ble.si_JigControl
                            );
                        }

                        if(simulationCount == 0)
                        {
                            if(simulation_ble.si_QcMode == 0 ||simulation_ble.si_QcMode == 8)
                            {

                            }
                            else
                            {
                                simulation_ble.si_QcModeStatus = JIG_QC_MODE_STATUS_IDLE;
                            }
                        
                            /// set flag send qcMode
                            flagSendJigQcMode = true;

                            simulationCount ++;
                        }
                        else if(simulationCount == 2)
                        {
                            if(simulation_ble.si_QcMode == 0 ||simulation_ble.si_QcMode == 8)
                            {

                            }
                            else
                            {
                                simulation_ble.si_QcModeStatus = JIG_QC_MODE_STATUS_RUNNING;
                            }
                            

                            /// set flag send qcMode
                            flagSendJigQcMode = true;

                            simulationCount ++;
                        }
                        else if(simulationCount == 7)
                        {
                            if(randomResult == false)
                            {
                                uint8_t value = random(0, 9);

                                randomResult = true;

                                if(value % 2 == 0)
                                {
                                    if(simulation_ble.si_QcMode == 0 ||simulation_ble.si_QcMode == 8)
                                    {

                                    }
                                    else
                                    {
                                        simulation_ble.si_QcModeStatus = JIG_QC_MODE_STATUS_PASSED;
                                    }
                                    
                                    /// set flag send qcMode
                                    flagSendJigQcMode = true;
                                }
                                else
                                {
                                    if(simulation_ble.si_QcMode == 0 ||simulation_ble.si_QcMode == 8)
                                    {

                                    }
                                    else
                                    {
                                        simulation_ble.si_QcModeStatus = JIG_QC_MODE_STATUS_FAILED;

                                        modeError = true;
                                    }
                                    
                                    /// set flag send qcMode
                                    flagSendJigQcMode = true;
                                }
                            }                          
                        }
                        else if(simulationCount >= 9)
                        {
                            simulationCount = 0;
                            randomResult = false;
                            simulationQcMode ++;

                            // simulation_ble.si_JigStatus = JIG_STATUS_STANBY;

                            simulation_ble.si_QcMode = (JigTestQcMode_t)simulationQcMode;

                             /// set flag send qcMode
                            flagSendJigQcMode = true;

                            sendFirstJigStatusRunning = false;
                        }

                        if(simulationQcMode >= 8)
                        {
                            /// next state
                            simulation_ble.state = 2;

                            simulationQcMode = 0;
                            timeSimulation = 0;
                            simulationCount = 0;

                            if(modeError == true)
                            {
                                simulation_ble.si_ProductOnJigTestStatus = PRODUCT_FAIL;
                            }
                            else
                            {
                                simulation_ble.si_ProductOnJigTestStatus = PRODUCT_COMPLETE;
                            }
                            

                            /// set flag send product On status
                            flagSendProductOnJigStatus = true;
                        }
                    }
                    else if(simulation_ble.si_JigControl == JIG_CONTROL_PAUSE)
                    {
                        static uint32_t timePause = 0;

                        if(millis() - timePause > 1000)
                        {
                            timePause = millis();

                            Serial.println("JIG_CONTROL_PAUSE");

                            /// set flag send control
                            flagSendJigControl = true;

                            /// send jig Control
                            // send_jigControl();
                            // delay(5);
                        }                       
                    }
                }break;
                case 2 :
                {

                    static uint32_t timePause = 0;
                    static uint8_t countAutoBackjigStatus_standby = 0;

                    if(simulation_ble.deviceDisconnected == true)
                    {
                        simulation_ble.deviceDisconnected = false;

                        timePause = 0;
                        countAutoBackjigStatus_standby = 0;
                    }

                    if(millis() - timePause > 1000)
                    {
                        timePause = millis();
                        if(++countAutoBackjigStatus_standby > 3)
                        {
                            countAutoBackjigStatus_standby = 0;
                            Serial.println("jigStatus Back to standby");

                            /// next state
                            simulation_ble.state = 0;

                            memset(&simulation_ble, 0, sizeof(bleProfileSimulation_t));

                            timeSimulation = 0;
                            simulationCount = 0;

                            simulation_ble.si_JigStatus = JIG_STATUS_STANBY;

                            /// set flag send jig Status
                            flagSendJigStatus = true;

                            management.JigControlBuffer[0] = 0x00;

                            firstRun = true;
                        }

                        Serial.printf("state : %d | countBackToStandby : %d\n", simulation_ble.state, countAutoBackjigStatus_standby);
                    } 

                    simulation_ble.si_JigControl = management.getJigControl();

                    if(simulation_ble.si_JigControl == JIG_CONTROL_STOP)
                    {
                        /// next state
                        simulation_ble.state = 0;

                        memset(&simulation_ble, 0, sizeof(bleProfileSimulation_t));

                        if(modeError == true)
                        {
                            simulation_ble.si_ProductOnJigTestStatus = PRODUCT_FAIL;
                        }
                        else
                        {
                            simulation_ble.si_ProductOnJigTestStatus = PRODUCT_COMPLETE;
                        }
                        
                        /// set flag send control
                        flagSendJigControl = true;

                        timeSimulation = 0;
                        simulationCount = 0;
                        
                    }


                }break;
                
                default:
                    break;
            }
        #endif

        if(millis() - timeSequence > 50)
        {
            timeSequence = millis();
            
            if(++countLedBlink > 10)
            {
                countLedBlink = 0;

                digitalWrite(22, state);

                state = !state;
            }
        
            /// send Characteristics jigStatus
            send_jigStatus(true);

            /// send Characteristics productOnStatus
            send_ProductOnJigStatus(true);

            /// send Characteristics qcMode
            send_jigQcMode(true);

            /// send Characteristics jigControl
            send_jigControl();

            /// send product profile
            send_productProfile();
        }
    }

    // disconnecting
    if (!isConnect && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        BLEDevice::startAdvertising(); // restart advertising
        Serial.println("restart advertising");
        oldDeviceConnected = isConnect;

        /// set flag device disconnect
        simulation_ble.deviceDisconnected = true;

        /// delete all jig result & process
        // memset(&simulation_ble, 0, sizeof(bleProfileSimulation_t));
        simulation_ble.si_JigControl = JIG_CONTROL_STOP;
        simulation_ble.si_JigStatus = JIG_STATUS_STANBY;
        simulation_ble.si_ProductOnJigTestStatus = PRODUCT_WAIT_FOR_QC;
        simulation_ble.si_ProductStatus = JIG_STATUS_NO_PRODUCT_ATTACHED;
        simulation_ble.si_QcMode = JIG_QC_MODE_IDLE;
        simulation_ble.si_QcModeStatus = JIG_QC_MODE_STATUS_IDLE;
        simulation_ble.state = 0;

        /// xoa buffer control
        management.JigControlBuffer[0] = 0x00;
    }
    // connecting
    if (isConnect && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = isConnect;
    }
}


#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/