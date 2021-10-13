#include "Arduino.h"
#include "PEGremsy_BLE.h"


/*
 * Battery Service
 * */
#define BLE_UUID_BATTERY_SERVICE				"180F"
#define BLE_UUID_BATTERY_LEVEL_CHAR				"2A19"

/*
 * jigTest Service
 */
#define BLE_UUID_JIGTEST_SERVICE				"0A630101-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_JIGTEST_CHAR					"0A630102-47B5-4779-8699-054D0879FC69"

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

#define MANUFACTURER            "GREMSY J.S.C."
#define MODEL_NUMBER            "JIG-01/VN"
#define SERIAL_NUMBER           "123456"
#define FIRMWARE_VERSION        "1.0"
#define HARDWARE_VERSION        "1.0"

#define BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG  0x01
#define BLE_DIS_VENDOR_ID_LSB                0x63
#define BLE_DIS_VENDOR_ID_MSB                0x0A
#define BLE_DIS_PRODUCT_ID_LSB               0x01
#define BLE_DIS_PRODUCT_ID_MSB               0x00
#define BLE_DIS_PRODUCT_VERSION_LSB          0x02
#define BLE_DIS_PRODUCT_VERSION_MSB          0x00

#define TYPE_DEVICE      0x02
#define VERSION_DEVICE   0x02

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
TYPE_DEVICE,
VERSION_DEVICE
};

bool isConnect;
uint8_t bufferRecieverHeartBeat[10];
uint8_t bufferRecieverParamValue[26];

extern mavlinkHandle_t mavlink;

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
        ESP_LOGI(TAG,"Connected");
        ESP_LOGI(TAG,"Connected Id %d - Count %d \n", pBLEServer->getConnId(), pBLEServer->getConnectedCount());

        Serial1.println("[     BLE     ] : Connected Id :" + String(pBLEServer->getConnId()) + " - Count :" + String(pBLEServer->getConnectedCount()));

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
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
        {
          Serial.print(value[i]);

          PEGremsy_BLE_getRecieverBuffer(value.length(), 10, bufferRecieverHeartBeat, value, i);
          PEGremsy_BLE_getRecieverBuffer(value.length(), 26, bufferRecieverParamValue, value, i);
        }
        Serial.println();
        Serial.println("*********");
      }
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

/** @brief  initialize
    @return none
*/
void PEGremsy_BLE::initialize(void)
{
    ESP_LOGI(TAG," Init BLE");

    /// init BLE
    BLEDevice::init("PEGremsy_BLE !");

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
    BLEService *pBatteryService = pServer->createService(BLEUUID(BLE_UUID_BATTERY_SERVICE));

    /// create battery Characteristics
    pCharacterBatteryLevel = pBatteryService->createCharacteristic(BLEUUID(BLE_UUID_BATTERY_LEVEL_CHAR), BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

    pDescriptorBatteryLevel = new BLEDescriptor(BLEUUID("2901"));
    pCharacterBatteryLevel->addDescriptor(pDescriptorBatteryLevel);

    /// start battery Service
    pBatteryService->start();

    pDescriptorBatteryLevel->setValue("value");

    /*
     * Create JIGTEST Service and Characteristics
     */
    BLEService *pJIGTESTService = pServer->createService(BLE_UUID_JIGTEST_SERVICE);

    pCharacteristicsJIGTEST = pJIGTESTService->createCharacteristic(BLE_UUID_JIGTEST_CHAR,
            BLECharacteristic::PROPERTY_READ   |
            BLECharacteristic::PROPERTY_WRITE  |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    pCharacteristicsJIGTEST->setCallbacks(new writeCallbacks());

    pJigTestDescriptor = new BLEDescriptor(BLE_UUID_JIGTEST_CHAR);
    pCharacteristicsJIGTEST->addDescriptor(pJigTestDescriptor);

    /// start jigtest service
    pJIGTESTService->start();

//  std::string value = "jigTest";
    uint8_t value = 10;

    pJigTestDescriptor->setValue(&value, 1);

    /*
     * Create mavlink Service and Characteristics
     */
    BLEService *pServiceMavlink = pServer->createService(BLE_UUID_MAVLINK_SERVICE);

    pCharacteristicsHeartbeat = pServiceMavlink->createCharacteristic(BLE_UUID_HEARTBEAT_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicsHeartbeat->setCallbacks(new writeCallbacks());

    pCharacteristicsParamValue = pServiceMavlink->createCharacteristic(BLE_UUID_PARAMVALUE_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicsParamValue->setCallbacks(new writeCallbacks());

    pCharacteristicsRawImu = pServiceMavlink->createCharacteristic(BLE_UUID_RAWIMU_CHAR, BLECharacteristic::PROPERTY_NOTIFY);

    pServiceMavlink->start();
    /*
     * Create Info Device Service and Characteristics
     */
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

    /// create PnpId Characteristics
    pCharacterPnPId = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_PNP_ID_CHAR), BLECharacteristic::PROPERTY_READ);

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

    /// set value mane PnPId
    pCharacterPnPId->setValue(PnPData, 7);

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    BLEAdvertisementData dataAdvertisement;
    dataAdvertisement.setManufacturerData(std::string(advertising_data, 4));
    pAdvertising->addServiceUUID(BLE_BASE_UUID);

    pAdvertising->setAdvertisementData(dataAdvertisement);

    BLEDevice::startAdvertising();

    Serial.println("start advertising");

    pinMode(2, OUTPUT);

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

    heartBeat.flag_heartbeat = bufferRecieverHeartBeat[9];

    if(heartBeat.flag_heartbeat == true)
    {
        heartBeat.custom_mode       = bufferRecieverHeartBeat[3] << 24 | bufferRecieverHeartBeat[2] << 16 | bufferRecieverHeartBeat[1] << 8 | bufferRecieverHeartBeat[0];
        heartBeat.type              = bufferRecieverHeartBeat[4];
        heartBeat.autopilot         = bufferRecieverHeartBeat[5];
        heartBeat.base_mode         = bufferRecieverHeartBeat[6];
        heartBeat.system_status     = bufferRecieverHeartBeat[7];
        heartBeat.mavlink_version   = bufferRecieverHeartBeat[8];

//      memset(bufferRecieverHeartBeat, 0, 10);
        heartBeat.flag_heartbeat = false;

        char buff[300];
        sprintf(buff, "custom_mode : %10d | type : %3d | autopilot : %3d | base_mode : %3d | system_status : %3d | mavlink_version : %3d"
                , heartBeat.custom_mode, heartBeat.type, heartBeat.autopilot, heartBeat.base_mode, heartBeat.system_status, heartBeat.mavlink_version );
        Serial.println("[recieverServiceHearBeat] : " + String(buff));

        ret = true;
    }

    return ret;
}

/** @brief  process
    @return none
*/
void PEGremsy_BLE::process(void)
{
    static uint8_t count[10];
    static uint8_t hex_array[4];
    uint8_t votlta;
    static uint32_t test = 50;
    static float testFloat = 1;
    static uint32_t timeSequence;
    static bool settingGimbalParam = false;

    // mavlink.process(NULL);
    
    if(isConnect == true)
    {
        heartBeatHandle();

        if(millis() - timeSequence > 1000)
        {
            timeSequence = millis();

            static bool state = false;
            digitalWrite(2, state);

            state = !state;

            testFloat+=0.1;

            ConvertFloat2Hex(testFloat, hex_array);

//          Serial.println("[hextoFloat] value : " + String(ConvertHex2ToFloat(hex_array)));

            votlta = random(90, 100);
            for(uint8_t i = 0; i < 10; i++) count[i]+=i;
//          Serial.println("Battery Level : " + String(votlta));

            pCharacterBatteryLevel->setValue(&votlta, 1);
            pCharacterBatteryLevel->notify(true);


//          Serial.println("[testFloat] value : " + String(testFloat));
//          pCharacteristicsJIGTEST->setValue(hex_array, 4);
            pCharacteristicsJIGTEST->setValue(testFloat);
            pCharacteristicsJIGTEST->notify(true);

            mavlink_msg_heartbeat_t heartbeat;
            heartbeat. custom_mode      = test++;
            heartbeat. type             = 6;
            heartbeat. autopilot        = 2;
            heartbeat. base_mode        = 4;
            heartbeat. system_status    = 5;
            heartbeat. mavlink_version  = 3;
            heartbeat. flag_heartbeat   = true;

            send_heartBeatService(&heartbeat, true);

            mavlink_msg_param_value_t param_value;
            param_value.param_value = testFloat;
            param_value.param_count = test;
            param_value.param_index = test - 50;
            memcpy(param_value.param_id, "nguyenvankinh123", 16);
            param_value.param_type = 0;

            send_paramValueCharastics(&param_value, true);

            send_rawImuCharastics(&mavlink.mavlinkSerial2.rawImu, true);

//          char buff[300];
//          sprintf(buff, "[raw_imu] xacc : %6d, yacc : %6d, zacc : %6d\n "
//                  "xgyro : %6d, ygyro : %6d, zgyro : %6d\n "
//                  "xmag : %6d, ymag : %6d, zmag : %6d\n "
//                  "id : %2d, temperature : %2d"
//              , mavlink.rawImu.xacc
//              , mavlink.rawImu.yacc
//              , mavlink.rawImu.zacc
//              , mavlink.rawImu.xgyro
//              , mavlink.rawImu.ygyro
//              , mavlink.rawImu.zgyro
//              , mavlink.rawImu.xmag
//              , mavlink.rawImu.ymag
//              , mavlink.rawImu.zmag
//              , mavlink.rawImu.id
//              , mavlink.rawImu.temperature
//          );
//
//          Serial.println(buff);

        }
    }
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
