/**
  ******************************************************************************
  * @file .c
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  * https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=eziya76&logNo=220988141146
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2016 Gremsy.
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
#include "mavlinkProtocol.h"
#include "../mavlink_v2/mavlink_helpers.h"

//#include "FastSerial.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Baud-rates available: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200, 256000, 512000, 962100
 *
 *  Protocols available:
 * SERIAL_5N1   5-bit No parity 1 stop bit
 * SERIAL_6N1   6-bit No parity 1 stop bit
 * SERIAL_7N1   7-bit No parity 1 stop bit
 * SERIAL_8N1 (the default) 8-bit No parity 1 stop bit
 * SERIAL_5N2   5-bit No parity 2 stop bits
 * SERIAL_6N2   6-bit No parity 2 stop bits
 * SERIAL_7N2   7-bit No parity 2 stop bits
 * SERIAL_8N2   8-bit No parity 2 stop bits
 * SERIAL_5E1   5-bit Even parity 1 stop bit
 * SERIAL_6E1   6-bit Even parity 1 stop bit
 * SERIAL_7E1   7-bit Even parity 1 stop bit
 * SERIAL_8E1   8-bit Even parity 1 stop bit
 * SERIAL_5E2   5-bit Even parity 2 stop bit
 * SERIAL_6E2   6-bit Even parity 2 stop bit
 * SERIAL_7E2   7-bit Even parity 2 stop bit
 * SERIAL_8E2   8-bit Even parity 2 stop bit
 * SERIAL_5O1   5-bit Odd  parity 1 stop bit
 * SERIAL_6O1   6-bit Odd  parity 1 stop bit
 * SERIAL_7O1   7-bit Odd  parity 1 stop bit
 * SERIAL_8O1   8-bit Odd  parity 1 stop bit
 * SERIAL_5O2   5-bit Odd  parity 2 stop bit
 * SERIAL_6O2   6-bit Odd  parity 2 stop bit
 * SERIAL_7O2   7-bit Odd  parity 2 stop bit
 * SERIAL_8O2   8-bit Odd  parity 2 stop bit
*/

#define MAVLINK_SERIAL2_BAUDRATE     115200
#define MAVLINK_SERIAL2_DATA_SIZE    SERIAL_8N1
#define MAVLINK_SERIAL2_RX_PIN       16
#define MAVLINK_SERIAL2_TX_PIN       17

#define MAVLINK_SERIAL1_BAUDRATE     115200
#define MAVLINK_SERIAL1_DATA_SIZE    SERIAL_8N1
#define MAVLINK_SERIAL1_RX_PIN       9
#define MAVLINK_SERIAL1_TX_PIN       10

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
/*variables interface */
mavlink_system_t    mavlink_system = {MAVLINK_COMM_0, MAV_COMP_ID_SYSTEM_CONTROL};
SoftwareSerial swSerial;
//FastSerialPort0(Serial2);
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group MAVLINKPROTOCOL_INITIALIZE
    @{
*/#ifndef MAVLINKPROTOCOL_INITIALIZE
#define MAVLINKPROTOCOL_INITIALIZE

mavlinkProtocol::mavlinkProtocol()
{

}

mavlinkProtocol::~mavlinkProtocol()
{

}

/** @brief 
    @return 
*/
void mavlinkProtocol::initialize(void)
{
	Serial2.begin(MAVLINK_SERIAL2_BAUDRATE);

    /// printf to console
    Serial.println("[mavlinkProtocol] Serial 2 Txd is on pin: "+String(MAVLINK_SERIAL2_TX_PIN));
    Serial.println("[mavlinkProtocol] Serial 2 Rxd is on pin: "+String(MAVLINK_SERIAL2_RX_PIN));

    #if (USE_SOFTWARE_SERIAL == 1)

        swSerial.begin(MAVLINK_SERIAL1_BAUDRATE, SWSERIAL_8N1, 18, 19, false, 256);

        Serial.println("[mavlinkProtocol] swSerial Txd is on pin: "+String(18));
        Serial.println("[mavlinkProtocol] swSerial Rxd is on pin: "+String(19));

        if (!swSerial) 
        { 
            // If the object did not initialize, then its configuration is invalid
            Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
            while (1) 
            { // Don't continue with invalid configuration
                delay (1000);
            }
        }

    #else

        Serial1.begin(MAVLINK_SERIAL1_BAUDRATE);

        /// printf to console
        Serial.println("[mavlinkProtocol] Serial 1 Txd is on pin: "+String(MAVLINK_SERIAL1_TX_PIN));
        Serial.println("[mavlinkProtocol] Serial 1 Rxd is on pin: "+String(MAVLINK_SERIAL1_RX_PIN));

    #endif
}


#endif
/**
    @}
*/

/** @group MAVLINK_PROTOCOL_READ_WRITE_DATA
    @{
*/#ifndef MAVLINK_PROTOCOL_READ_WRITE_DATA
#define MAVLINK_PROTOCOL_READ_WRITE_DATA

/** @brief serial_readData
    @return none
*/
bool mavlinkProtocol::serial_readData(HardwareSerial *serial, mavlink_channel_t channel, mav_state_t *mav)
{
    bool ret = false;

    if(serial->available() > 0)
    {
        uint8_t c = serial->read();

        if(mavlink_parse_char(channel, c, &mav->rxmsg, &mav->status))
        {
            ret = true;
        }
    }

    return ret;
}

/** @brief swSerial_readData
    @return none
*/
bool mavlinkProtocol::swSerial_readData(mavlink_channel_t channel, mav_state_t *mav)
{
    bool ret = false;

    if(swSerial.available() > 0)
    {
        uint8_t c = swSerial.read();

        // Serial.println("swSerial :");
        // Serial.println(c, HEX);

        if(mavlink_parse_char(channel, c, &mav->rxmsg, &mav->status))
        {
            ret = true;
        }
    }

    return ret;
}

/** @brief send a buffer out a MAVLink channel
    @param[in] chan
    @param[in] buf
    @param[in] len
    @return none
*/
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    static bool useDebugComm_0 = false;
    static bool useDebugComm_1 = false;

    /// kiem tra channel co ton tai
    if(!valid_channel(chan))
    {
        return;
    }

    /// send data to serial with mavlink channel
    if(chan == MAVLINK_COMM_0)
    {
        if(useDebugComm_0 == false)
        {
            Serial2.write(buf, len);
        }
        else
        {
            uint8_t length = Serial2.write(buf, len);

            if(length == len)
            {
                for(uint8_t i = 0; i < length; i++)
                Serial.print(buf[i], HEX);

                Serial.println("");
                Serial.println("[Serial 2 send mavlink packet] : " + String(len) + "byte");
            }
        }
    }
    else if(chan == MAVLINK_COMM_1)
    {
        if(useDebugComm_1 == false)
        {
            #if(USE_SOFTWARE_SERIAL == 1)
                swSerial.write(buf, len);
            #else
                Serial1.write(buf, len);
            #endif
        }
        else
        {
            #if(USE_SOFTWARE_SERIAL == 1)
                uint8_t length = swSerial.write(buf, len);

                if(length == len)
                {
                    for(uint8_t i = 0; i < length; i++)
                    Serial.print(buf[i], HEX);

                    Serial.println("");
                    Serial.println("[Serial 1 send mavlink packet] : " + String(len) + "byte");
                }
            #else
                uint8_t length = Serial1.write(buf, len);

                if(length == len)
                {
                    for(uint8_t i = 0; i < length; i++)
                    Serial.print(buf[i], HEX);

                    Serial.println("");
                    Serial.println("[Serial 1 send mavlink packet] : " + String(len) + "byte");
                }
            #endif

        }
    }
}


#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
