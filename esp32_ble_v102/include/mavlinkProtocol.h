/** 
  ******************************************************************************
  * @file    mavlinkProtocol.h
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2021 Gremsy. All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __MAVLINKPROTOCOL_H
#define __MAVLINKPROTOCOL_H

#include "Arduino.h"

/* Includes ------------------------------------------------------------------*/
#include "../mavlink_v2/mavlink_avoid_errors.h"
// we have separate helpers disabled to make it possible
#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */

#include "../mavlink_v2/ardupilotmega/version.h"

#define MAVLINK_MAX_PAYLOAD_LEN 255

#include "../mavlink_v2/mavlink_types.h"

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Sanity check MAVLink channel
///
/// @param chan		Channel to send to
static inline bool valid_channel(mavlink_channel_t chan)
{
//#pragma clang diagnostic push
//#pragma clang diagnostic ignored "-Wtautological-constant-out-of-range-compare"
    return chan < MAVLINK_COMM_NUM_BUFFERS;
//#pragma clang diagnostic pop
}

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "../mavlink_v2/ardupilotmega/mavlink.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct _mav_state
{
    mavlink_message_t   rxmsg;
    mavlink_status_t    status;
} mav_state_t;

class mavlinkProtocol
{
    public :
        void initialize();
        bool serial_readData(HardwareSerial *serial, mavlink_channel_t channel, mav_state_t* mav);

    mavlinkProtocol();
    virtual ~mavlinkProtocol();
};
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/** @brief send a buffer out a MAVLink channel
    @param[in] chan
    @param[in] buf
    @param[in] len
    @return none
*/
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

#endif /* __MAVLINKPROTOCOL_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
