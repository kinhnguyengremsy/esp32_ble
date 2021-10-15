/** 
  ******************************************************************************
  * @file    mavlinkHandle.h
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

#ifndef __MAVLINKHANDLE_H
#define __MAVLINKHANDLE_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mavlinkProtocol.h"
/* Exported define ------------------------------------------------------------*/
#define JIG_ID	0x01
/* Exported types ------------------------------------------------------------*/

/**
 * @brief Gimbal rotation mode, specifies control style.
 */
typedef enum {
    GIMBAL_ROTATION_MODE_RELATIVE_ANGLE = 0, /*!< Relative angle rotation mode, represents rotating gimbal specified angles based on current angles. */
    GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE = 1, /*!< Absolute angle rotation mode, represents rotating gimbal to specified angles in the ground coordinate. */
    GIMBAL_ROTATION_MODE_SPEED = 2, /*!< Speed rotation mode, specifies rotation speed of gimbal in the ground coordinate. */
} E_GimbalRotationMode;

/**
 * @brief control_mode_t
 * Command control gimbal mode lock/follow
 */
typedef enum _control_gimbal_mode_t
{
    LOCK_MODE   = 1,
    FOLLOW_MODE = 2,
    MAPPING_MODE = 3,
} control_gimbal_mode_t;

/**
 * @brief control_motor_t
 * Command control motor is on/off
 */
typedef enum _control_gimbal_motor_t
{
    TURN_OFF    = 0,
    TURN_ON     = 1
} control_gimbal_motor_t;

/**
 * @brief gimbal_state_t
 * State of gimbal
 */
typedef enum _gimbal_state
{
    GIMBAL_STATE_OFF            = 0x00,     /*< Gimbal is off*/
    GIMBAL_STATE_INIT           = 0x01,     /*< Gimbal is initializing*/
    GIMBAL_STATE_ON             = 0x02,     /*< Gimbal is on */
    GIMBAL_STATE_MAPPING_MODE   = 0x03,
    GIMBAL_STATE_LOCK_MODE      = 0x04,
    GIMBAL_STATE_FOLLOW_MODE    = 0x08,
    GIMBAL_STATE_SEARCH_HOME    = 0x10,
    GIMBAL_STATE_SET_HOME       = 0x20,
    GIMBAL_STATE_ERROR          = 0x40,
    GIMBAL_STATE_SENSOR_CALIB  = 0x400,

} gimbal_state_t;

/**
 * @brief gimbal_state_t
 * State of gimbal's sensor
 */
typedef enum _sensor_state
{
    SENSOR_OK                   = 0x00,     /* Gimbal's sensor is healthy */
    SENSOR_IMU_ERROR            = 0x01,     /* IMU error*/
    SENSOR_EN_TILT              = 0x02,     /* Encoder sensor is error at tilt axis*/
    SENSOR_EN_ROLL              = 0x03,     /* Encoder sensor is error at roll axis*/
    SENSOR_EN_PAN               = 0x04,     /* Encoder sensor is error at pan axis*/
}sensor_state_;

/**
 * @brief remote_control_gimbal_t
 * Command remote control gimbal
 */
typedef enum _remote_control_gimbal_t
{
    MAVLINK_CONTROL_MODE       = 2,
    REMOTE_CONTROL_MODE        = 3 //MAV_MOUNT_MODE_RC_TARGETING
} remote_control_gimbal_t;

/**
 * @brief remote_control_gimbal_t
 * Setting mode remote control gimbal
 */
typedef enum _modeRC_control_gimbal_t
{
	GIMBAL_RC_MODE_SBUS = 1,
	GIMBAL_RC_MODE_PPM = 6,
	GIMBAL_RC_MODE_CAN = 11,
	GIMBAL_RC_MODE_MAVLINK = 15,

}modeRC_control_gimbal_t;

/**
 * @brief controlJigState
 * controlJigState
 */
typedef enum _controlJigState_t
{
	CONTROL_JIG_STATE_IDLE = 0,
	CONTROL_JIG_STATE_START,
	CONTROL_JIG_STATE_RUNNING,
	CONTROL_JIG_STATE_DONE,
	CONTROL_JIG_STATE_RESET,

}controlJigState_t;

/**
 * @brief controlJigMode
 * controlJigMode
 */
typedef enum _controlJigMode_t
{
	CONTROL_JIG_MODE_SBUS = 1,
	CONTROL_JIG_MODE_PPM,
	CONTROL_JIG_MODE_CAN,
	CONTROL_JIG_MODE_COM2,
	CONTROL_JIG_MODE_COM4,
	CONTROL_JIG_MODE_AUX,
	CONTROL_JIG_MODE_VIRATE,
	CONTROL_JIG_MODE_DONE,
}controlJigMode_t;

/**
 * @brief commandStartStop
 * commandStartStop
 */
typedef enum _commandStartStop_t	
{
	COMMAND_START = 1,
	COMMAND_STOP,
	COMMAND_RESET,

}commandStartStop_t;

/**
 * @brief commandStatus
 * commandStatus
 */
typedef enum _commandStatus_t
{
	COMMAND_STATUS_STANDBY = 1,
	COMMAND_STATUS_RUNNING,
	COMMAND_STATUS_DONE,
	COMMAND_STATUS_RESET,
}commandStatus_t;

typedef struct _mavlink_msg_heartbeat_t
{
    bool flag_heartbeat;

    uint8_t vehicle_system_id;      // gimbal system
    uint8_t vehicle_component_id;   // Gimbal component

    uint16_t count;

    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t type; /*<  Type of the MAV (quadrotor, helicopter, etc.)*/
    uint8_t autopilot; /*<  Autopilot type / class.*/
    uint8_t base_mode; /*<  System mode bitmap.*/
    uint8_t system_status; /*<  System status flag.*/
    uint8_t system_status_send; /*<  System status flag.*/
    uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

}mavlink_msg_heartbeat_t;

typedef struct _mavlink_msg_param_value_t
{
	float param_value; /*<  Onboard parameter value*/
	uint16_t param_count; /*<  Total number of onboard parameters*/
	uint16_t param_index; /*<  Index of this onboard parameter*/
	char param_id[16]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
	uint8_t param_type; /*<  Onboard parameter type.*/
}mavlink_msg_param_value_t;

typedef struct _mavlink_msg_raw_imu_t
{
	uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
	int16_t xacc; /*<  X acceleration (raw)*/
	int16_t yacc; /*<  Y acceleration (raw)*/
	int16_t zacc; /*<  Z acceleration (raw)*/
	int16_t xgyro; /*<  Angular speed around X axis (raw)*/
	int16_t ygyro; /*<  Angular speed around Y axis (raw)*/
	int16_t zgyro; /*<  Angular speed around Z axis (raw)*/
	int16_t xmag; /*<  X Magnetic field (raw)*/
	int16_t ymag; /*<  Y Magnetic field (raw)*/
	int16_t zmag; /*<  Z Magnetic field (raw)*/
	uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
	int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/
}mavlink_msg_raw_imu_t;

typedef struct _mavlink_msg_mount_orientation_t
{
	uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
	float roll; /*< [deg] Roll in global frame (set to NaN for invalid).*/
	float pitch; /*< [deg] Pitch in global frame (set to NaN for invalid).*/
	float yaw; /*< [deg] Yaw relative to vehicle(set to NaN for invalid).*/
	float yaw_absolute; /*< [deg] Yaw in absolute frame, North is 0 (set to NaN for invalid).*/
}mavlink_msg_mount_orientation_t;

typedef struct _mavlink_msg_command_ack_t
{
	uint16_t command; /*<  Command ID (of acknowledged command).*/
	uint8_t result; /*<  Result of command.*/
	uint8_t progress; /*<  WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.*/
	int32_t result_param2; /*<  WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.*/
	uint8_t target_system; /*<  WIP: System which requested the command to be executed*/
	uint8_t target_component; /*<  WIP: Component which requested the command to be executed*/
}mavlink_msg_command_ack_t;

/**
 * @brief gimbal_state_t
 * State of gimbal's sensor
 */
typedef struct _gimbal_status_t
{
    uint16_t    load; /*< [ms] Maximum usage the mainloop time. Values: [0-1000] - should always be below 1000*/
    uint16_t    voltage_battery; /*< [V] Battery voltage*/
    uint8_t     sensor; /*< Specific sensor occur error (encorder, imu) refer sensor_state_*/
    uint16_t    state;  /* System state of gimbal. Refer gimbal_state_t*/
    uint8_t     mode;   /*< Gimbal mode is running*/
    uint32_t    seq;
    uint8_t     startUpCalib_running;
    bool        startUpCalib_Done;
    uint8_t     first_calib_motor;
} gimbal_status_t;

typedef struct 
{
	/* data */
	mavlink_msg_heartbeat_t 		heartBeat;
	mavlink_msg_raw_imu_t 			rawImu;
	mavlink_msg_mount_orientation_t mountOrientation;
	mavlink_msg_command_ack_t 		ackCommand;
	gimbal_status_t					gimbalStatus;
	mavlink_msg_param_value_t		paramValue;

}mavlinkMsg_t;


class mavlinkHandle_t
{
	public:
		mavlinkMsg_t mavlinkSerial2;
		mavlinkMsg_t mavlinkSerial1;

		mavlink_msg_heartbeat_t control;

		void initialize(void);
		void sendData(void);
		void recieverData(void);
		void process( void *pvParameters );
		
		void controlGimbal(int16_t tilt, int16_t roll, int16_t pan, control_gimbal_mode_t mode);
		bool settingParamGimbal(void);
		void controlJig(void);

	private:
		/* data */

		/* function */
		void mavlink_set_gimbal_move(mavlink_channel_t channel, int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode);
		void mavlink_set_gimbal_home(mavlink_channel_t channel);
		void mavlink_set_gimbal_mode(mavlink_channel_t channel, control_gimbal_mode_t mode);
		void mavlink_control_motor(mavlink_channel_t channel, control_gimbal_motor_t type);
		bool findGimbalAxis(uint8_t ID);
		void mavlink_set_param_gimbal(mavlink_channel_t channel, float param_value, char *param_id);
		void mavlink_param_request_read(mavlink_channel_t channel, int16_t param_index, char* param_id);
		bool requestParamGimbal(void);
		void mavlink_remoteControl(mavlink_channel_t channel, remote_control_gimbal_t command);
		bool requestGimbalModeRC(modeRC_control_gimbal_t modeRC);
		void sendheartbeat(mavlink_channel_t channel);
		bool getGimbalReturnHome(void);
		bool applyControlGimbalWithRC(modeRC_control_gimbal_t modeRC, bool RcOrMavlink);
		bool applyControlJig(modeRC_control_gimbal_t modeControl, controlJigMode_t *modeInput, controlJigMode_t modeOutput);

};
/* Exported constants --------------------------------------------------------*/
extern mavlinkHandle_t mavlink;
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


#endif /* __MAVLINKHANDLE_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
