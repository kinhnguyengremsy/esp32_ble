/**
  ******************************************************************************
  * @file    .cpp
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2011 Gremsy.
  * All rights reserved.Firmware coding style V1.0.beta
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/
/* Includes------------------------------------------------------------------------------*/
#include "PEGremsy_BLE.h"
#include "Arduino.h"

#include <EEPROM.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "myMath.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/*
 * jigTest Service
 */
#define BLE_UUID_JIGTEST_SERVICE				"0A630101-47B5-4779-8699-054D0879FC69"
#define BLE_UUID_JIGTEST_CHAR					"0A630102-47B5-4779-8699-054D0879FC69"

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

#define FLASH_MEMORY_SIZE 10
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
BLEServer* pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

char advertising_data[4] = {
BLE_DIS_VENDOR_ID_LSB,
BLE_DIS_VENDOR_ID_MSB,
TYPE_DEVICE,
VERSION_DEVICE
};

size_t BLE_address[2];
extern char BLE_characteristisJigBuffer[50];
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
PEGremsy_BLE_t::PEGremsy_BLE_t(/* args */)
{
}

PEGremsy_BLE_t::~PEGremsy_BLE_t()
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

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
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

        BLEUUID jigCharUuId = BLEUUID(BLE_UUID_JIGTEST_CHAR);
        BLEUUID UuIdGet = pCharacteristic->getUUID();

        if(UuIdGet.equals(jigCharUuId) == true)
        {
            for (int i = 0; i < value.length(); i++)
            {
                // PEGremsy_BLE_getRecieverBuffer(value.length(), 2, BLE_characteristisJigBuffer, value, i);
                BLE_characteristisJigBuffer[i] = value[i];
                
                Serial.print(BLE_characteristisJigBuffer[i], HEX);
            }
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};

/** @group BLE_INITIALIZE
    @{
*/#ifndef BLE_INITIALIZE
#define BLE_INITIALIZE
/** @brief initialize
    @return none
*/
void PEGremsy_BLE_t::initialize(void)
{
    String BLE_name = "PEGremsy_BLE ";

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
    // BLEDevice::init("PEGremsy_BLE 990A");
    /// set power
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);

    /// create ble server
    BLEServer* pServer = BLEDevice::createServer();

    /// set MTU
    BLEDevice::setMTU(512);

    pServer->setCallbacks(new MyServerCallbacks());

    // // // Create the BLE Service
    // // BLEService *pService = pServer->createService(BLE_BASE_UUID);
  // Create the BLE Service
  BLEService *pService = pServer->createService(BLE_BASE_UUID);

  // Create a BLE Characteristic
  pCharacteristicsJIGTEST = pService->createCharacteristic(
                      BLE_UUID_JIGTEST_CHAR,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristicsJIGTEST->addDescriptor(new BLE2902());

  pCharacteristicsJIGTEST->setCallbacks(new writeCallbacks());

  // Start the service
  pService->start();

  // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    BLEAdvertisementData dataAdvertisement;
    dataAdvertisement.setManufacturerData(std::string(advertising_data, 4));
    pAdvertising->setAdvertisementData(dataAdvertisement);
    pAdvertising->addServiceUUID(BLE_BASE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection ...");
    // /*
    //  * Create JIGTEST Service and Characteristics
    //  */
    // BLEService *pJIGTESTService = pServer->createService(BLE_UUID_JIGTEST_SERVICE);

    // pCharacteristicsJIGTEST = pJIGTESTService->createCharacteristic(BLE_UUID_JIGTEST_CHAR,
    //         BLECharacteristic::PROPERTY_READ   |
    //         BLECharacteristic::PROPERTY_WRITE  |
    //         BLECharacteristic::PROPERTY_NOTIFY |
    //         BLECharacteristic::PROPERTY_INDICATE);

    // pCharacteristicsJIGTEST->setCallbacks(new writeCallbacks());

    // // pJigTestDescriptor = new BLEDescriptor(BLE_UUID_JIGTEST_CHAR);
    // // pCharacteristicsJIGTEST->addDescriptor(pJigTestDescriptor);

    // /// start jigtest service
    // pJIGTESTService->start();

    // // /*
    // //  * Create Info Device Service and Characteristics
    // //  */
    // // BLEService *pInfoDeviceService = pServer->createService(BLEUUID(BLE_UUID_DEVICE_INFORMATION_SERVICE));

    // // /// create manufacture Characteristics
    // // pCharacterNameManufacture = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_MANUFACTURER_NAME_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    // // /// create model number Characteristics
    // // pCharacterModelNumber = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_MODEL_NUMBER_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    // // /// create serial number Characteristics
    // // pCharacterSerialNumber = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_SERIAL_NUMBER_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    // // /// create firmware version Characteristics
    // // pCharacterFirmwareVerison = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_FIRMWARE_REVISION_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    // // /// create hardware version Characteristics
    // // pCharacterHardwareVerison = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_HARDWARE_REVISION_STRING_CHAR), BLECharacteristic::PROPERTY_READ);

    // // /// create PnpId Characteristics
    // // pCharacterPnPId = pInfoDeviceService->createCharacteristic(BLEUUID(BLE_UUID_PNP_ID_CHAR), BLECharacteristic::PROPERTY_READ);

    // // /// start device info service
    // // pInfoDeviceService->start();

    // // /// set value mane Manufacture
    // // pCharacterNameManufacture->setValue(MANUFACTURER);

    // // /// set value mane module number
    // // pCharacterModelNumber->setValue(MODEL_NUMBER);

    // // /// set value mane serial number
    // // pCharacterSerialNumber->setValue(SERIAL_NUMBER);

    // // /// set value mane firmware version
    // // pCharacterFirmwareVerison->setValue(FIRMWARE_VERSION);

    // // /// set value mane hardware version
    // // pCharacterHardwareVerison->setValue(HARDWARE_VERSION);

    // // /// set value mane PnPId
    // // pCharacterPnPId->setValue(PnPData, 7);

    // // Start advertising
    // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    // // BLEAdvertisementData dataAdvertisement;
    // // dataAdvertisement.setManufacturerData(std::string(advertising_data, 4));
    // // pAdvertising->setAdvertisementData(dataAdvertisement);
    // pAdvertising->addServiceUUID(BLE_BASE_UUID);
    // pAdvertising->setScanResponse(false);
    // pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter

    // BLEDevice::startAdvertising();

    // Serial.println("");
    // Serial.println("start advertising");
}

#endif
/**
    @}
*/

/** @group BLE_PROCESS
    @{
*/#ifndef BLE_PROCESS
#define BLE_PROCESS
/** @brief initialize
    @return none
*/
void PEGremsy_BLE_t::process(void)
{
    static uint32_t timeCount = 0;
    static uint8_t count = 0;

    // notify changed value
    if (deviceConnected) {

        static bool status = false;

        if(millis() - timeCount > 100)
        {
            timeCount = millis();

            digitalWrite(22, status);
            status = !status;

            // pCharacteristic->setValue((uint8_t*)&value, 4);
            // pCharacteristic->notify();
            // value++;
        }
        // delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


