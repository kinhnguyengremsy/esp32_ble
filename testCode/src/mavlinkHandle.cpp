/**
  ******************************************************************************
  * @file mavlinkHandle.cpp
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
#include "Arduino.h"
#include "mavlinkHandle.h"
#include "gimbalParam.h"
#include "taskManagement.h"
/* Private typedef------------------------------------------------------------------------------*/
typedef enum
{
	CONTROL_ANGLE_STATE_IDLE,
	CONTROL_ANGLE_STATE_SET_MODE,
	CONTROL_ANGLE_STATE_MOVE_CW,
	CONTROL_ANGLE_STATE_MOVE_CCW,
	CONTROL_ANGLE_STATE_DONE,
}controlAngle_state;
/* Private define------------------------------------------------------------------------------*/
#define DEBUG_STATE 0
#define DEBUG_MAVLINK_HANDLE_HEARTBEAT_SERIAL2				0
#define DEBUG_MAVLINK_HANDLE_HEARTBEAT_SERIAL1				0
#define DEBUG_MAVLINK_HANDLE_MOUNTORIENTATION_SERIAL2		0
#define DEBUG_MAVLINK_HANDLE_MOUNTORIENTATION_SERIAL1		0
#define DEBUG_MAVLINK_HANDLE_PARAMVALUE_SERIAL2				0
#define DEBUG_MAVLINK_HANDLE_PARAMVALUE_SERIAL1				0
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
mavlinkProtocol protocol;
mav_state_t mavlinkStateSerial2, mavlinkStateSerial1;
extern taskManagement_t management;
/* Private function prototypes------------------------------------------------------------------------------*/

/* Private functions------------------------------------------------------------------------------*/

/** @group MAVLINK_HANDLE_INITIALIZE
    @{
*/#ifndef MAVLINK_HANDLE_INITIALIZE
#define MAVLINK_HANDLE_INITIALIZE

/** @brief initialize
    @return none
*/
void mavlinkHandle_t::initialize(void)
{
	protocol.initialize();	
}

#endif
/**
    @}
*/

/** @group MAVLINK_MESSAGE_HANDLE
    @{
*/#ifndef MAVLINK_MESSAGE_HANDLE
#define MAVLINK_MESSAGE_HANDLE

/** @brief mavlinkMessageHandle
    @return none
*/
void mavlinkMessageHandle(mavlink_channel_t channel, mavlinkMsg_t* msg, mav_state_t *mavlink)
{
	switch (mavlink->rxmsg.msgid)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
		{
			mavlink_heartbeat_t heartbeat;
			mavlink_msg_heartbeat_decode(&mavlink->rxmsg, &heartbeat);

			msg->heartBeat.flag_heartbeat = true;
			msg->heartBeat.vehicle_system_id = mavlink->rxmsg.sysid;

			// uint8_t len = sizeof(mavlink_heartbeat_t);
			// memcpy(&msg->heartBeat, &heartbeat, len);
			msg->heartBeat.autopilot 		= heartbeat.autopilot;
			msg->heartBeat.base_mode 		= heartbeat.base_mode;
			msg->heartBeat.custom_mode 		= heartbeat.custom_mode;
			msg->heartBeat.mavlink_version 	= heartbeat.mavlink_version;
			msg->heartBeat.system_status 	= heartbeat.system_status;
			msg->heartBeat.type 			= heartbeat.type;

			#if(DEBUG_MAVLINK_HANDLE_HEARTBEAT_SERIAL2 == 1)

				char buffHeartbeatSr1[300];

				if(channel == MAVLINK_COMM_0)
				{
					sprintf(buffHeartbeatSr1, "[heartbeat Serial2] autopilot : %3d, base_mode : %3d, custom_mode : %3d, mavlink_version : %3d, system_status : %3d, type : %3d"
						,msg->heartBeat.autopilot
						,msg->heartBeat.base_mode
						,msg->heartBeat.custom_mode
						,msg->heartBeat.mavlink_version
						,msg->heartBeat.system_status
						,msg->heartBeat.type
					);

					Serial.println(buffHeartbeatSr1);
				}

			#endif

			#if(DEBUG_MAVLINK_HANDLE_HEARTBEAT_SERIAL1 == 1)

				if(channel == MAVLINK_COMM_1)
				{
					Serial.printf("[heartbeat Serial1] autopilot : %3d, base_mode : %3d, custom_mode : %3d, mavlink_version : %3d, system_status : %3d, type : %3d\n"
						,msg->heartBeat.autopilot
						,msg->heartBeat.base_mode
						,msg->heartBeat.custom_mode
						,msg->heartBeat.mavlink_version
						,msg->heartBeat.system_status
						,msg->heartBeat.type
					);
				}

			#endif		            
		}break;
		case MAVLINK_MSG_ID_SYS_STATUS:
		{
			mavlink_sys_status_t              packet;
			mavlink_msg_sys_status_decode(&mavlink->rxmsg, &packet);

			/* Get voltage battery*/
			msg->gimbalStatus.voltage_battery   = packet.voltage_battery;

			msg->gimbalStatus.load              = packet.load;

			/* Check gimbal's motor */
			if(packet.errors_count1 & 0x10)
			{
				msg->gimbalStatus.state = GIMBAL_STATE_ON;

				/* Check gimbal is follow mode*/
				if(packet.errors_count1 & 0x01)
				{
					msg->gimbalStatus.mode = GIMBAL_STATE_FOLLOW_MODE;
				}
				else
				{
					msg->gimbalStatus.mode = GIMBAL_STATE_LOCK_MODE;
				}
			}
			/* Check gimbal is initializing*/
			else if(packet.errors_count1 & 0x20)
			{
				msg->gimbalStatus.state = GIMBAL_STATE_INIT;
			}
			else if(packet.errors_count1 & 0x04)
			{
				/* Check gimbal is error state*/
				msg->gimbalStatus.state = GIMBAL_STATE_ERROR;
			}
			else if(!(packet.errors_count1 & 0x00))
			{
				msg->gimbalStatus.state    = GIMBAL_STATE_OFF;
				msg->gimbalStatus.mode     = GIMBAL_STATE_OFF;
			}
			else if((packet.errors_count1 & GIMBAL_STATE_MAPPING_MODE) == GIMBAL_STATE_MAPPING_MODE)
			{
				msg->gimbalStatus.state    = GIMBAL_STATE_MAPPING_MODE;
			}

			/* Check gimbal's sensor status */
			if(packet.errors_count2 & 0x01)
			{
				msg->gimbalStatus.sensor |= SENSOR_IMU_ERROR;
			}
			if(packet.errors_count2 & 0x02)
			{
				msg->gimbalStatus.sensor |= SENSOR_EN_TILT;
			}
			if(packet.errors_count2 & 0x04)
			{
				msg->gimbalStatus.sensor |= SENSOR_EN_ROLL;
			}
			if(packet.errors_count2 & 0x08)
			{
				msg->gimbalStatus.sensor |= SENSOR_EN_PAN;
			}
			else
			{
				msg->gimbalStatus.sensor = SENSOR_OK;
			}

			/// kiem tra gimbal StartUp calib

			if((packet.errors_count1 & GIMBAL_STATE_SENSOR_CALIB) == GIMBAL_STATE_SENSOR_CALIB)
			{
				msg->gimbalStatus.startUpCalib_running = 1;
			}
			else
			{
				msg->gimbalStatus.startUpCalib_running = 2;
			}

			if((packet.errors_count1 & 0x20) == 0x20)
			{
				msg->gimbalStatus.first_calib_motor = 1;
			}
			else
			{
				msg->gimbalStatus.first_calib_motor = 2;
			}

		}break;
		case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
		{
			mavlink_mount_orientation_t mount_orientation;
			mavlink_msg_mount_orientation_decode(&mavlink->rxmsg, &mount_orientation);

			uint8_t len = sizeof(mavlink_mount_orientation_t);
			memcpy(&msg->mountOrientation, &mount_orientation, len);


			#if(DEBUG_MAVLINK_HANDLE_MOUNTORIENTATION_SERIAL2 == 1)

				if(channel == MAVLINK_COMM_0)
				{
					Serial.printf("[mountOrientation Serial2] pitch : %f, roll : %f, yaw : %f, time_boot_ms : %7d, yaw_absolute : %f\n"
						,msg->mountOrientation.pitch
						,msg->mountOrientation.roll
						,msg->mountOrientation.yaw
						,msg->mountOrientation.time_boot_ms
						,msg->mountOrientation.yaw_absolute
					);
				}
				

			#endif

			#if(DEBUG_MAVLINK_HANDLE_MOUNTORIENTATION_SERIAL1 == 1)

				char buffMountOrienSr1[100];

				if(channel == MAVLINK_COMM_1)
				{
					sprintf(buffMountOrienSr1, "[mountOrientation Serial1] pitch : %f, roll : %f, yaw : %f, time_boot_ms : %7d, yaw_absolute : %f"
						,msg->mountOrientation.pitch
						,msg->mountOrientation.roll
						,msg->mountOrientation.yaw
						,msg->mountOrientation.time_boot_ms
						,msg->mountOrientation.yaw_absolute
					);

					Serial.println(buffMountOrienSr1);
				}

			#endif
		}break;
		case MAVLINK_MSG_ID_RAW_IMU:
		{
			mavlink_raw_imu_t raw_imu;
			mavlink_msg_raw_imu_decode(&mavlink->rxmsg, &raw_imu);


			uint8_t len = sizeof(mavlink_raw_imu_t);
			memcpy(&msg->rawImu, &raw_imu, len);
//		            Serial.println("[raw_imu] len : " + String(len));

//		            char buff[300];
//		            sprintf(buff, "[raw_imu] xacc : %6d | %6d, yacc : %6d | %6d, zacc : %6d | %6d\n "
//		            		"xgyro : %6d | %6d, ygyro : %6d | %6d, zgyro : %6d | %6d\n "
//		            		"xmag : %6d | %6d, ymag : %6d | %6d, zmag : %6d | %6d\n "
//		            		"id : %2d | %2d, temperature : %2d | %2d"
//		                ,raw_imu.xacc, rawImu.xacc
//		                ,raw_imu.yacc, rawImu.yacc
//		                ,raw_imu.zacc, rawImu.zacc
//		                ,raw_imu.xgyro, rawImu.xgyro
//		                ,raw_imu.ygyro, rawImu.ygyro
//		                ,raw_imu.zgyro, rawImu.zgyro
//		                ,raw_imu.xmag, rawImu.xmag
//		                ,raw_imu.ymag, rawImu.ymag
//		                ,raw_imu.zmag, rawImu.zmag
//						,raw_imu.id, rawImu.id
//						,raw_imu.temperature, rawImu.temperature
//		            );
//
//		            Serial.println(buff);
		}break;
		case MAVLINK_MSG_ID_COMMAND_ACK:
		{
			mavlink_command_ack_t ack;
			mavlink_msg_command_ack_decode(&mavlink->rxmsg, &ack);


			// uint8_t len = sizeof(mavlink_command_ack_t);
			// memcpy(&msg->ackCommand, &ack, len);
			msg->ackCommand.command = ack.command;
			msg->ackCommand.result = ack.result;

//		            Serial.println("[command ack rec] : command" + String(ackCommand.command) + "| result : " + String(ackCommand.result));
		}break;
		case MAVLINK_MSG_ID_PARAM_VALUE:
		{
			mavlink_param_value_t param_value;
			mavlink_msg_param_value_decode(&mavlink->rxmsg, &param_value);

			uint8_t len = sizeof(mavlink_param_value_t);
			memcpy(&msg->paramValue, &param_value, len);

			#if (DEBUG_MAVLINK_HANDLE_PARAMVALUE_SERIAL2 == 1)
				if(channel == MAVLINK_COMM_0)
				Serial.println("MAVLINK_MSG_ID_PARAM_VALUE : param_index : " + String(msg->paramValue.param_index) + " | param_value : " + String(msg->paramValue.param_value));
			#endif

			# if(DEBUG_MAVLINK_HANDLE_PARAMVALUE_SERIAL1 == 1)
				if(channel == MAVLINK_COMM_1)
				Serial.println("MAVLINK_MSG_ID_PARAM_VALUE : param_index : " + String(msg->paramValue.param_index) + " | param_value : " + String(msg->paramValue.param_value));
			#endif
		}break;

		default:
			break;
	}
}

/** @brief sendheartbeat
    @return none
*/
void mavlinkHandle_t::sendheartbeat(mavlink_channel_t channel)
{
	mavlink_message_t       msg;
	mavlink_heartbeat_t     heartbeat;
	uint16_t                len = 0;

	if(channel == MAVLINK_COMM_0)
	{
		// static uint8_t commandStatus = 0;

		// if(mavlinkSerial2.heartBeat.type == COMMAND_START)
		// {
		// 	commandStatus = COMMAND_STATUS_RUNNING;
		// }
		// else if(mavlinkSerial2.heartBeat.type == COMMAND_STOP)
		// {
		// 	// commandStatus = COMMAND_STATUS_;
		// }
		// else
		// {

		// }

		// heartbeat.type          = mavlinkSerial2.heartBeat.type;//MAV_TYPE_ONBOARD_TESTER;
		// heartbeat.autopilot     = mavlinkSerial2.heartBeat.autopilot;//MAV_AUTOPILOT_INVALID;
		// heartbeat.base_mode     = mavlinkSerial2.heartBeat.base_mode;//0;
		// heartbeat.custom_mode   = mavlinkSerial2.heartBeat.custom_mode;//0;
		// heartbeat.system_status = commandStatus;//MAV_STATE_ACTIVE;

		heartbeat.type          = MAV_TYPE_ONBOARD_TESTER;
		heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
		heartbeat.base_mode     = 0;
		heartbeat.custom_mode   = 0;
		heartbeat.system_status = MAV_STATE_ACTIVE;
	}
	else if(channel == MAVLINK_COMM_1)
	{
		heartbeat.type          = control.type;
		heartbeat.autopilot     = 0;
		heartbeat.base_mode     = control.base_mode;
		heartbeat.custom_mode   = 0;
		heartbeat.system_status = 0;

		Serial.println("[sendHeartBeat to STM32] type : " + String(control.type) + " | base_mode : " + String(control.base_mode));

		// heartbeat.type          = MAV_TYPE_ONBOARD_TESTER;
		// heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
		// heartbeat.base_mode     = 0;
		// heartbeat.custom_mode   = 0;
		// heartbeat.system_status = MAV_STATE_ACTIVE;
	}

	/*
	save and restore sequence number for chan, as it is used by
	generated encode functions
	*/

	mavlink_status_t    *chan_status = mavlink_get_channel_status((mavlink_channel_t)channel);
	uint8_t saved_seq = chan_status->current_tx_seq;

	mavlink_msg_heartbeat_encode_chan(  0x01,
										MAV_COMP_ID_SYSTEM_CONTROL,
										(mavlink_channel_t)channel,
										&msg,
										&heartbeat);

	chan_status->current_tx_seq = saved_seq;

	uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];

	len = mavlink_msg_to_send_buffer(msgbuf, &msg);

	if(len > 0)
	{
		_mavlink_send_uart((mavlink_channel_t)channel,(const char*) msgbuf, len);
	}
}

/** @brief mavlink_set_gimbal_move
    @return none
*/
void mavlinkHandle_t::mavlink_set_gimbal_move(mavlink_channel_t channel, int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;

    memset(&msg, 0, sizeof(mavlink_message_t));
    memset(&command_long, 0, sizeof(mavlink_command_long_t));
   
    command_long.command            = MAV_CMD_DO_MOUNT_CONTROL;
    command_long.param1             = tilt;
    command_long.param2             = roll;
    command_long.param3             = pan;
    command_long.param4             = 0;
    command_long.param5             = 0;
    command_long.param6             = rotationMode;
    command_long.param7             = MAV_MOUNT_MODE_MAVLINK_TARGETING;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(   JIG_ID,
                                            MAV_COMP_ID_SYSTEM_CONTROL,
                                            channel,
                                            &msg,
                                            &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];

    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }    
}

/** @brief mavlink_gimbal_home
    @return none
*/
void mavlinkHandle_t::mavlink_set_gimbal_home(mavlink_channel_t channel)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    command_long.command            = MAV_CMD_USER_2;
    command_long.param6             = 3;

    command_long.param7             = 0x04;

    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    command_long.target_system      = MAV_COMP_ID_SYSTEM_CONTROL;
    
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(   JIG_ID,
                                            MAV_COMP_ID_SYSTEM_CONTROL,
                                            channel,
                                            &msg,
                                            &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];

    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_mode
    @return none
*/
void mavlinkHandle_t::mavlink_set_gimbal_mode(mavlink_channel_t channel, control_gimbal_mode_t mode)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;

    command_long.command            = MAV_CMD_USER_2;
    command_long.param7             = mode;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */

    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;

    mavlink_msg_command_long_encode_chan(	JIG_ID,
											MAV_COMP_ID_SYSTEM_CONTROL,
											channel,
											&msg,
											&command_long);

    chan_status->current_tx_seq = saved_seq;

    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];

    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_control_motor
    @return none
*/
void mavlinkHandle_t::mavlink_control_motor(mavlink_channel_t channel, control_gimbal_motor_t type)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;

    command_long.command            = MAV_CMD_USER_1;
    command_long.param7             = type;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */

    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;

    mavlink_msg_command_long_encode_chan(   JIG_ID,
                                            MAV_COMP_ID_SYSTEM_CONTROL,
                                            channel,
                                            &msg,
                                            &command_long);

    chan_status->current_tx_seq = saved_seq;

    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];

    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_set_param
    @return none
*/
void mavlinkHandle_t::mavlink_set_param_gimbal(mavlink_channel_t channel, float param_value, char *param_id)
{
    uint16_t len;
    mavlink_message_t msg;

    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;

    mavlink_msg_param_set_pack(   JIG_ID
								, MAV_COMP_ID_SYSTEM_CONTROL
								, &msg
								, JIG_ID
								, MAV_COMP_ID_GIMBAL
								, param_id
								, (float)param_value
								, MAVLINK_TYPE_UINT16_T);

    chan_status->current_tx_seq = saved_seq;

    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_home
    @return none
*/
bool mavlinkHandle_t::findGimbalAxis(uint8_t ID)
{
	bool ret = false;

	if(ID == 0x7A || ID == 0x54)
	{
		ret = true;
	}

	return ret;
}

/** @brief controlGimbal
    @return none
*/
void mavlinkHandle_t::controlGimbal(int16_t tilt, int16_t roll, int16_t pan, control_gimbal_mode_t mode, bool controlLoop)
{
	static controlAngle_state state;
	static uint32_t timeState = 0;
	static uint32_t timeSetHomeGimbal = 0;

	if(millis() - timeState > 1000 || timeState == 0)
	{
		timeState = millis();

		Serial.println("[controlGimbalState] state : " + String(state) + "mode : " + String(mavlinkSerial2.gimbalStatus.mode));
		Serial.println("[command ack control] : command" + String(mavlinkSerial2.ackCommand.command) + "| result : " + String(mavlinkSerial2.ackCommand.result));
	}

	switch(state)
	{
		case CONTROL_ANGLE_STATE_IDLE:
		{
			if(controlLoop == true)
			{
				if(mavlinkSerial2.ackCommand.command == 0)
				{	
					if(millis() - timeSetHomeGimbal > 1000 || timeSetHomeGimbal == 0)
					{
						timeSetHomeGimbal = millis();

						mavlink_set_gimbal_home(MAVLINK_COMM_0);
					}
				}
				else
				{
					static uint32_t timeSequence = 0;
					static uint8_t countTimeOut = 0;

					if(mavlinkSerial2.ackCommand.command == MAV_CMD_USER_2)
					{
						if(mavlinkSerial2.ackCommand.result == MAV_RESULT_ACCEPTED)
						{
							uint16_t deltaPan = abs((int16_t)mavlinkSerial2.mountOrientation.yaw - 0);
							uint16_t deltaRoll = abs((int16_t)mavlinkSerial2.mountOrientation.roll - 0);
							uint16_t deltaTilt = abs((int16_t)mavlinkSerial2.mountOrientation.pitch - 0);

							Serial.println("[set angle cw gimbal] delta roll :" + String(deltaRoll) + "delta tilt :" + String(deltaTilt));

							if(deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5 )
							{
								Serial.println("[set home gimbal] : DONE");

								/// next state
								state = CONTROL_ANGLE_STATE_SET_MODE;

								timeSequence = 0;
								countTimeOut = 0;
								mavlinkSerial2.ackCommand.command = 0;
								mavlinkSerial2.ackCommand.result = 0;

								timeSetHomeGimbal = 0;

								uint8_t len = sizeof(mavlink_command_ack_t);
								memset(&mavlinkSerial2.ackCommand, 0, len);
							}
						}
					}
					else
					{
						if(millis() - timeSequence > 500 || timeSequence == 0)
						{
							timeSequence = millis();

							if(countTimeOut++ > 5)
							{
								countTimeOut = 0;

								uint8_t len = sizeof(mavlink_command_ack_t);
								memset(&mavlinkSerial2.ackCommand, 0, len);

								Serial.println("[set home gimba] : CMD timeOut");
							}
							else
							{
								Serial.println("[set home gimba] : RUNNING");
							}
						}
					}
				}
			}
			else
			{
				/// next state
				state = CONTROL_ANGLE_STATE_SET_MODE;

				timeSetHomeGimbal = 0;

				uint8_t len = sizeof(mavlink_command_ack_t);
				memset(&mavlinkSerial2.ackCommand, 0, len);
			}
		}break;
		case CONTROL_ANGLE_STATE_SET_MODE:
		{
			if(mavlinkSerial2.ackCommand.command == 0)
			{
				mavlink_set_gimbal_mode(MAVLINK_COMM_0, mode);
			}
			{
				if(mavlinkSerial2.ackCommand.command == MAV_CMD_USER_2)
				{
					uint8_t gState = 0;

					if(mode == LOCK_MODE) gState = (uint8_t)GIMBAL_STATE_LOCK_MODE;
					else if(mode == FOLLOW_MODE) gState = (uint8_t)GIMBAL_STATE_FOLLOW_MODE;

					if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
					{
						if(mavlinkSerial2.gimbalStatus.mode == gState)
						{
							Serial.println("[set mode gimbal] : DONE");

							/// next state
							state = CONTROL_ANGLE_STATE_MOVE_CW;
						}
					}
					else if(mavlinkSerial2.ackCommand.result == MAV_RESULT_ACCEPTED)
					{
						if(mavlinkSerial2.gimbalStatus.mode == gState)
						{
							Serial.println("[set mode gimbal] : DONE");

							/// next state
							state = CONTROL_ANGLE_STATE_MOVE_CW;
						}
					}

				}
			}
		}break;
		case CONTROL_ANGLE_STATE_MOVE_CW:
		{
			static uint32_t timeSequence = 0;
			static uint32_t timeComapreAngle = 0;
			static bool controlRetry = true;

			int16_t panSetpoint = 170;
			int16_t rollSetpoint = 0;
			int16_t tiltSetpoint = 45;

			if(mavlinkSerial2.ackCommand.command == MAV_CMD_DO_MOUNT_CONTROL)
			{
				uint16_t deltaPan  = abs((int16_t)mavlinkSerial2.mountOrientation.yaw - panSetpoint);
				uint16_t deltaRoll = abs((int16_t)mavlinkSerial2.mountOrientation.roll - rollSetpoint);
				uint16_t deltaTilt = abs((int16_t)mavlinkSerial2.mountOrientation.pitch - tiltSetpoint);

				if(millis() - timeComapreAngle > 5000 || timeComapreAngle == 0)
				{
					timeComapreAngle = millis();

					Serial.println("[mountOrientation value] roll :" + String(mavlinkSerial2.mountOrientation.roll) + "tilt :" + String(mavlinkSerial2.mountOrientation.pitch) + "pan :" + String(mavlinkSerial2.mountOrientation.yaw));
					char buff[200];
					sprintf(buff, "[set angle cw gimbal] delta roll : %3d | delta tilt : %3d | delta pan : %3d", deltaRoll, deltaTilt, deltaPan);
					Serial.println(buff);

					if(findGimbalAxis(mavlinkSerial2.heartBeat.vehicle_system_id) == true)
					{
						if(deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[set angle cw gimbal] : DONE");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							{
								if(controlLoop == true)
								{
									state = CONTROL_ANGLE_STATE_MOVE_CCW;
								}
								else
								{
									state = CONTROL_ANGLE_STATE_DONE;

									timeSequence = 0;
									timeComapreAngle = 0;

									uint32_t result = 1;
									uint32_t resultTemp = (result << (CONTROL_JIG_MODE_COM2 - 1));

									mavlinkSerial1.heartBeat.custom_mode |= resultTemp;

									Serial.println("[controlGimbal] jig test COM2 DONE | " + String(mavlinkSerial1.heartBeat.custom_mode));
								}
							}
							

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
						else
						{
							//// timeOut control
							controlRetry = true;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
					else
					{
						if( deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[set angle cw gimbal] : DONE");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							{
								if(controlLoop == true)
								{
									state = CONTROL_ANGLE_STATE_MOVE_CCW;
								}
								else
								{
									state = CONTROL_ANGLE_STATE_DONE;

									timeSequence = 0;
									timeComapreAngle = 0;

									uint32_t result = 1;
									uint32_t resultTemp = (result << (CONTROL_JIG_MODE_COM2 - 1));

									mavlinkSerial1.heartBeat.custom_mode |= resultTemp;

									Serial.println("[controlGimbal] jig test COM2 DONE | " + String(mavlinkSerial1.heartBeat.custom_mode));
								}
							}
							

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
						else
						{
							//// timeOut control
							controlRetry = true;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
				}
				else
				{
					if(findGimbalAxis(mavlinkSerial2.heartBeat.vehicle_system_id) == true)
					{
						if( deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[mountOrientation value] roll :" + String(mavlinkSerial2.mountOrientation.roll) + "tilt :" + String(mavlinkSerial2.mountOrientation.pitch) + "pan :" + String(mavlinkSerial2.mountOrientation.yaw));
							char buff[200];
							sprintf(buff, "[set angle cw gimbal] delta roll : %3d | delta tilt : %3d | delta pan : %3d", deltaRoll, deltaTilt, deltaPan);
							Serial.println(buff);
							Serial.println("[set angle cw gimbal] : DONE-soon");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							{
								if(controlLoop == true)
								{
									state = CONTROL_ANGLE_STATE_MOVE_CCW;
								}
								else
								{
									state = CONTROL_ANGLE_STATE_DONE;

									timeSequence = 0;
									timeComapreAngle = 0;

									uint32_t result = 1;
									uint32_t resultTemp = (result << (CONTROL_JIG_MODE_COM2 - 1));

									mavlinkSerial1.heartBeat.custom_mode |= resultTemp;

									Serial.println("[controlGimbal] jig test COM2 DONE | " + String(mavlinkSerial1.heartBeat.custom_mode));
								}
							}
							

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
					else
					{
						if( deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[mountOrientation value] roll :" + String(mavlinkSerial2.mountOrientation.roll) + "tilt :" + String(mavlinkSerial2.mountOrientation.pitch) + "pan :" + String(mavlinkSerial2.mountOrientation.yaw));
							char buff[200];
							sprintf(buff, "[set angle cw gimbal] delta roll : %3d | delta tilt : %3d | delta pan : %3d", deltaRoll, deltaTilt, deltaPan);
							Serial.println(buff);
							Serial.println("[set angle cw gimbal] : DONE-soon");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							{
								if(controlLoop == true)
								{
									state = CONTROL_ANGLE_STATE_MOVE_CCW;
								}
								else
								{
									state = CONTROL_ANGLE_STATE_DONE;

									timeSequence = 0;
									timeComapreAngle = 0;

									uint32_t result = 1;
									uint32_t resultTemp = (result << (CONTROL_JIG_MODE_COM2 - 1));

									mavlinkSerial1.heartBeat.custom_mode |= resultTemp;

									Serial.println("[controlGimbal] jig test COM2 DONE | " + String(mavlinkSerial1.heartBeat.custom_mode));
								}
							}
							

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
				}
			}
			else
			{
				if(millis() - timeSequence > 500 || timeSequence == 0 || controlRetry == true)
				{
					timeSequence = millis();

					controlRetry = false;

//					mavlink_set_gimbal_mode(MAVLINK_COMM_0, mode);
					mavlink_set_gimbal_move(MAVLINK_COMM_0, tiltSetpoint, rollSetpoint, panSetpoint, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
				}
			}
		}break;
		case CONTROL_ANGLE_STATE_MOVE_CCW:
		{
			static uint32_t timeSequence = 0;
			static uint32_t timeComapreAngle = 0;
			static bool controlRetry = true;

			int16_t panSetpoint = -170;
			int16_t rollSetpoint = 0;
			int16_t tiltSetpoint = -45;

			if(mavlinkSerial2.ackCommand.command == MAV_CMD_DO_MOUNT_CONTROL)
			{
				uint16_t deltaPan  = abs((int16_t)mavlinkSerial2.mountOrientation.yaw - panSetpoint);
				uint16_t deltaRoll = abs((int16_t)mavlinkSerial2.mountOrientation.roll - rollSetpoint);
				uint16_t deltaTilt = abs((int16_t)mavlinkSerial2.mountOrientation.pitch - tiltSetpoint);

				if(millis() - timeComapreAngle > 5000 || timeComapreAngle == 0)
				{
					timeComapreAngle = millis();

					Serial.println("[mountOrientation value] roll :" + String(mavlinkSerial2.mountOrientation.roll) + "tilt :" + String(mavlinkSerial2.mountOrientation.pitch) + "pan :" + String(mavlinkSerial2.mountOrientation.yaw));
					char buff[200];
					sprintf(buff, "[set angle cw gimbal] delta roll : %3d | delta tilt : %3d | delta pan : %3d", deltaRoll, deltaTilt, deltaPan);
					Serial.println(buff);

					if(findGimbalAxis(mavlinkSerial2.heartBeat.vehicle_system_id) == true)
					{
						if( deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[set angle cw gimbal] : DONE");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							state = CONTROL_ANGLE_STATE_MOVE_CW;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
						else
						{
							//// timeOut control
							controlRetry = true;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
					else
					{
						if( deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[set angle cw gimbal] : DONE");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							state = CONTROL_ANGLE_STATE_MOVE_CW;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
						else
						{
							//// timeOut control
							controlRetry = true;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
				}
				else
				{
					if(findGimbalAxis(mavlinkSerial2.heartBeat.vehicle_system_id) == true)
					{
						if( deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[mountOrientation value] roll :" + String(mavlinkSerial2.mountOrientation.roll) + "tilt :" + String(mavlinkSerial2.mountOrientation.pitch) + "pan :" + String(mavlinkSerial2.mountOrientation.yaw));
							char buff[200];
							sprintf(buff, "[set angle cw gimbal] delta roll : %3d | delta tilt : %3d | delta pan : %3d", deltaRoll, deltaTilt, deltaPan);
							Serial.println(buff);
							Serial.println("[set angle cw gimbal] : DONE-soon");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							state = CONTROL_ANGLE_STATE_MOVE_CW;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
					else
					{
						if( deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[mountOrientation value] roll :" + String(mavlinkSerial2.mountOrientation.roll) + "tilt :" + String(mavlinkSerial2.mountOrientation.pitch) + "pan :" + String(mavlinkSerial2.mountOrientation.yaw));
							char buff[200];
							sprintf(buff, "[set angle cw gimbal] delta roll : %3d | delta tilt : %3d | delta pan : %3d", deltaRoll, deltaTilt, deltaPan);
							Serial.println(buff);
							Serial.println("[set angle cw gimbal] : DONE-soon");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							state = CONTROL_ANGLE_STATE_MOVE_CW;

							uint8_t len = sizeof(mavlink_command_ack_t);
							memset(&mavlinkSerial2.ackCommand, 0, len);
						}
					}
				}
			}
			else
			{
				if(millis() - timeSequence > 500 || timeSequence == 0 || controlRetry == true)
				{
					timeSequence = millis();

					if(controlRetry == true)
					{
						Serial.println("[set angle cw gimbal] : retry running");
					}

					controlRetry = false;

//					mavlink_set_gimbal_mode(MAVLINK_COMM_0, mode);
					mavlink_set_gimbal_move(MAVLINK_COMM_0, tiltSetpoint, rollSetpoint, panSetpoint, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
				}
			}
		}break;
		case CONTROL_ANGLE_STATE_DONE:
		{
			state = CONTROL_ANGLE_STATE_IDLE;
			timeState = 0;
			timeSetHomeGimbal = 0;
		}break;
	}
}

/** @brief mavlink_param_request_read
    @return none
*/
void mavlinkHandle_t::mavlink_param_request_read(mavlink_channel_t channel, int16_t param_index, char* param_id)
{
    mavlink_message_t msg;
    mavlink_param_request_read_t request_read;
    uint16_t len = 0;
    
    mav_array_memcpy(request_read.param_id, param_id, sizeof(char) * 16);
    request_read.param_index = param_index;
    request_read.target_component = MAV_COMP_ID_GIMBAL;
    request_read.target_system = mavlinkSerial2.heartBeat.vehicle_system_id;
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_param_request_read_encode_chan( JIG_ID,
                                                MAV_COMP_ID_SYSTEM_CONTROL,
                                                channel,
                                                &msg,
                                                &request_read);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief getHeartBeatReady
    @return none
*/
bool mavlinkHandle_t::getSerial1HeartBeatReady(uint32_t time)
{
	bool ret = false;
	static uint32_t timeOut_heartBeat = 0;
	static bool heartBeatReady = false;

	if(millis() - timeOut_heartBeat > time || timeOut_heartBeat == 0)
	{
		timeOut_heartBeat = millis();

		if(mavlinkSerial1.heartBeat.flag_heartbeat == true)
		{
			mavlinkSerial1.heartBeat.flag_heartbeat = false;

			if(heartBeatReady == false)
			Serial.println("[getHeartBeatReady] heartBeat Jig ready");

			heartBeatReady = true;

			ret = true;
		}
		else
		{
			heartBeatReady = false;

			Serial.println("[getHeartBeatReady] heartBeat Jig not ready");
		}
	}

	return ret;
}

/** @brief getHeartBeatReady
    @return none
*/
bool mavlinkHandle_t::getSerial2HeartBeatReady(uint32_t time)
{
	bool ret = false;
	static uint32_t timeOut_heartBeat = 0;
	static bool heartBeatReady = false;

	if(millis() - timeOut_heartBeat > time || timeOut_heartBeat == 0)
	{
		timeOut_heartBeat = millis();

		if(mavlinkSerial2.heartBeat.flag_heartbeat == true)
		{
			mavlinkSerial2.heartBeat.flag_heartbeat = false;

			if(heartBeatReady == false)
			Serial.println("[getHeartBeatReady] heartBeat Gimbal ready");

			heartBeatReady = true;

			ret = true;
		}
		else
		{
			heartBeatReady = false;

			Serial.println("[getHeartBeatReady] heartBeat Gimbal not ready");
		}
	}

	return ret;
}

/** @brief requestParamGimbal
    @return none
*/
bool mavlinkHandle_t::requestParamGimbal(void)
{
	static bool ret = false;
	static uint16_t paramIndex_temp = 0;
	static uint8_t countParamRead_true = 1;
	static uint32_t timeOutRequestParamGimbal = 0;
	static uint32_t timeSendRequestParam = 0;
	static uint8_t countTimeOutRequestParam = 0;

	paramIndex_temp = mavlinkSerial2.paramValue.param_index;

	if(paramIndex_temp == gimbal_param_setting[countParamRead_true - 1].index)
	{
		if(millis() - timeSendRequestParam > 100)
		{
			timeSendRequestParam = millis();

			mavlink_param_request_read(MAVLINK_COMM_0
			, gimbal_param_setting[countParamRead_true].index
			, (char *)gimbal_param_setting[countParamRead_true].param_id);

			Serial.println("[requestParamGimbal] paramId :" \
			+ String(gimbal_param_setting[countParamRead_true].param_id) + " ---> send index :" \
			+ String(gimbal_param_setting[countParamRead_true].index));
		}
	}
	else
	{
		uint16_t paramIndexReciever = mavlinkSerial2.paramValue.param_index;
		uint16_t paramIndexCompare = gimbal_param_setting[countParamRead_true].index;

		/// comapre param index reciever with param index set to gimbal
		if(paramIndexReciever == paramIndexCompare)
		{
			Serial.println("[requestParamGimbal]  paramId:" + String(gimbal_param_setting[countParamRead_true].param_id) + " ---> valueReciever : " \
			+ String(mavlinkSerial2.paramValue.param_value) + "|" \
			+ String(gimbal_param_setting[countParamRead_true].value) \
			+ " | countParamRead_true :" + String(countParamRead_true)
			);

			/// true -> count Up variable countParamRead_true
			countParamRead_true ++;

			/// check number of param reciever
			if(countParamRead_true == NUMBER_OF_GIMBAL_PARAM_TEST)
			{
				paramIndex_temp = 0;
				countParamRead_true = 0;
				timeOutRequestParamGimbal = 0;
				timeSendRequestParam = 0;
				countTimeOutRequestParam = 0;

				ret = true;
			}
		}
	}

	if(millis() - timeOutRequestParamGimbal > 1000)
	{
		timeOutRequestParamGimbal = millis();

		if(++countTimeOutRequestParam > 15)
		{
			Serial.println("[requestParamGimbal] timeOut request param gimbal .... try again !!!");

			mavlink_set_gimbal_reboot(MAVLINK_COMM_0);

			/// reset paramValue receiver from gimbal
			uint8_t len = sizeof(mavlink_msg_param_value_t);
			uint8_t d = 0;
			memcpy(&mavlinkSerial2.paramValue, &d, len);

			paramIndex_temp = 0;
			countParamRead_true = 0;
		}
	}

	return ret;
}

/** @brief settingParamGimbal
    @return none
*/
bool mavlinkHandle_t::settingParamGimbal(void)
{
	bool ret = false;
	static bool settingFlag = true;

	if(settingFlag == true)
	{
		settingFlag = false;

		for(uint8_t i = 0; i < NUMBER_OF_GIMBAL_PARAM_TEST; i++)
		{
			gimbal_param_setting[i].value = gimbalParam_valueTestPixy[i].value;

			float param_value = gimbal_param_setting[i].value;

			/// set param to gimbal
			mavlink_set_param_gimbal(MAVLINK_COMM_0, param_value, (char *)gimbal_param_setting[i].param_id);

			Serial.println("[settingParamGimbal] param_id : " + String(gimbal_param_setting[i].param_id) + "---> param_value : " + String(param_value));
		}

		/// reset paramValue receiver from gimbal
		uint8_t len = sizeof(mavlink_msg_param_value_t);
		uint8_t d = 0;
		memcpy(&mavlinkSerial2.paramValue, &d, len);
	}
	else
	{
		if(requestParamGimbal() == true)
		{
			settingFlag = true;

			ret = true;
		}
	}

	return ret;
}

/** @brief mavlink_remoteControl
    @return none
*/
void mavlinkHandle_t::mavlink_remoteControl(mavlink_channel_t channel, remote_control_gimbal_t command)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    
    command_long.command            = MAV_CMD_DO_MOUNT_CONFIGURE;
    command_long.param1             = command;

    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(	JIG_ID,
											MAV_COMP_ID_SYSTEM_CONTROL,
											channel,
											&msg,
											&command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    
    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief requestGimbalModeRC
    @return none
*/
void mavlinkHandle_t::mavlink_set_gimbal_reboot(mavlink_channel_t channel)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;

    uint8_t systemid    = JIG_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = channel;
    
    // Reverse tilt and Pan with the Pixhawk
    command_long.command            = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    command_long.param1             = 0;
    command_long.param2             = 0;
    command_long.param3             = 0;
    command_long.param4             = 1;
    command_long.param5             = 0;
    command_long.param6             = 0;
    command_long.param7             = 0;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);
    
    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    
    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief requestGimbalModeRC
    @return none
*/
#define REQUEST_GIMBAL_MODE_RC_PRINT_DEBUG	0
bool mavlinkHandle_t::requestGimbalModeRC(modeRC_control_gimbal_t modeRC)
{
	bool ret = false;
	static uint32_t timeSendParam = 0;
	static uint32_t timeRequestParam = 0;
	char* param_id = "RADIO_TYPE";

	if(mavlinkSerial2.paramValue.param_index == 28)
	{
		if(mavlinkSerial2.paramValue.param_value == (float)modeRC)
		{
			timeSendParam = 0;
			timeRequestParam = 0;
			mavlinkSerial2.paramValue.param_index = 0;
			mavlinkSerial2.paramValue.param_value = 0;

			#if (REQUEST_GIMBAL_MODE_RC_PRINT_DEBUG == 1)
				Serial.println("[requestGimbalModeRC] DONE ");
			#endif
			
			ret = true;
		}
	}
	else
	{
		if(millis() - timeSendParam > 1000 || timeSendParam == 0)
		{
			timeSendParam = millis();

			mavlink_set_param_gimbal(MAVLINK_COMM_0, (float)modeRC, param_id);

			#if (REQUEST_GIMBAL_MODE_RC_PRINT_DEBUG == 1)
				Serial.println("[requestGimbalModeRC] send mode : " + String(modeRC));
			#endif
		}
		else
		{
			if(millis() - timeRequestParam > 500 || timeRequestParam == 0)
			{
				timeRequestParam = millis();

				mavlink_param_request_read(MAVLINK_COMM_0, 28, param_id);

				#if (REQUEST_GIMBAL_MODE_RC_PRINT_DEBUG == 1)
					Serial.println("[requestGimbalModeRC] send request mode : " + String(modeRC));
				#endif
			}
		}
	}

	return ret;
}

/** @brief sendData
    @return none
*/
void mavlinkHandle_t::sendData(void)
{
	static uint32_t timeSequence = 0;

//	if(heartBeat.flag_heartbeat == true)
	if(millis() - timeSequence > 500 || timeSequence == 0)
	{
//		heartBeat.flag_heartbeat = false;
		timeSequence = millis();

		sendheartbeat(MAVLINK_COMM_0);

		sendheartbeat(MAVLINK_COMM_1);
	}

}

/** @brief recieverData
    @return none
*/
void mavlinkHandle_t::recieverData(void)
{
	if(protocol.serial_readData(&Serial2, MAVLINK_COMM_0, &mavlinkStateSerial2))
	{
		mavlinkMessageHandle(MAVLINK_COMM_0, &mavlinkSerial2, &mavlinkStateSerial2);
	}

	#if (USE_SOFTWARE_SERIAL == 1)

		if(protocol.swSerial_readData(MAVLINK_COMM_1, &mavlinkStateSerial1))
		{
			mavlinkMessageHandle(MAVLINK_COMM_1, &mavlinkSerial1, &mavlinkStateSerial1);
		}

	#else
		if(protocol.serial_readData(&Serial1, MAVLINK_COMM_1, &mavlinkStateSerial1))
		{
			mavlinkMessageHandle(MAVLINK_COMM_1, &mavlinkSerial1, &mavlinkStateSerial1);
		}
	#endif
}

/** @brief getGimbalReturnHome
    @return bool
*/
#define GET_GIMBAL_RETURN_HOME_PRINT_DEBUG	0
bool mavlinkHandle_t::getGimbalReturnHome(void)
{
	bool ret = false;

	static uint32_t timeSetGimbalReturnHome = 0;
	static bool clearAckCommand = false;
	int16_t homeSetPoint = 0;

	if(clearAckCommand == false)
	{
		clearAckCommand = true;

		uint8_t len = sizeof(mavlink_command_ack_t);
		memset(&mavlinkSerial2.ackCommand, 0, len);
	}

	if(millis() - timeSetGimbalReturnHome > 1000 || timeSetGimbalReturnHome == 0)
	{
		timeSetGimbalReturnHome = millis();

		if(mavlinkSerial2.ackCommand.command != MAV_CMD_USER_2)
		{
			mavlink_set_gimbal_home(MAVLINK_COMM_0);
		}

		#if (GET_GIMBAL_RETURN_HOME_PRINT_DEBUG == 1)
			Serial.println("[getGimbalReturnHome] set gimbal return home running ....... | " 
			+ String(mavlinkSerial2.mountOrientation.yaw) 
			+ " | " + String(mavlinkSerial2.mountOrientation.roll) 
			+ " | " + String(mavlinkSerial2.mountOrientation.pitch) 
			+ "ackCommand :" + String(mavlinkSerial2.ackCommand.command) 
			+ " | result : " + String(mavlinkSerial2.ackCommand.result));
		#endif
	}
	else
	{
		if(mavlinkSerial2.ackCommand.command == MAV_CMD_USER_2)
		{
			
			uint16_t deltaPan = abs((int16_t)mavlinkSerial2.mountOrientation.yaw - homeSetPoint);
			uint16_t deltaRoll = abs((int16_t)mavlinkSerial2.mountOrientation.roll - homeSetPoint);
			uint16_t deltaTilt = abs((int16_t)mavlinkSerial2.mountOrientation.pitch - homeSetPoint);

			/// kiem tra gimbal axis
			if(findGimbalAxis(mavlinkSerial2.heartBeat.vehicle_system_id) == true)
			{
				/// kiem tra goc gimbal
				if(mavlinkSerial2.mountOrientation.roll != 0.00 && mavlinkSerial2.mountOrientation.pitch != 0.00)
				{
					/// kiem tra tinh toan  goc gimbal
					if(deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5)
					{
						timeSetGimbalReturnHome = 0;
						clearAckCommand = false;

						uint8_t len = sizeof(mavlink_command_ack_t);
						memset(&mavlinkSerial2.ackCommand, 0, len);

						#if (GET_GIMBAL_RETURN_HOME_PRINT_DEBUG == 1)
							Serial.println("[getGimbalReturnHome] gimbal is HOME | " + String(deltaRoll) + " | " + String(deltaTilt)
							+ "ackCommand :" + String(mavlinkSerial2.ackCommand.command) + " | result : " + String(mavlinkSerial2.ackCommand.result));
						#endif
						ret = true;
					}
				}
			}
			else
			{
				/// kiem tra goc gimbal
				if(mavlinkSerial2.mountOrientation.yaw != 0.00 && mavlinkSerial2.mountOrientation.roll != 0.00 && mavlinkSerial2.mountOrientation.pitch != 0.00)
				{
					/// kiem tra tinh toan  goc gimbal
					if(deltaPan < 5 && deltaRoll < 5 && deltaTilt < 5)
					{
						timeSetGimbalReturnHome = 0;
						clearAckCommand = false;

						uint8_t len = sizeof(mavlink_command_ack_t);
						memset(&mavlinkSerial2.ackCommand, 0, len);

						#if (GET_GIMBAL_RETURN_HOME_PRINT_DEBUG == 1)
							Serial.println("[getGimbalReturnHome] gimbal is HOME | " + String(deltaPan) + " | " + String(deltaRoll) + " | " + String(deltaTilt)
							+ "ackCommand :" + String(mavlinkSerial2.ackCommand.command) + " | result : " + String(mavlinkSerial2.ackCommand.result));
						#endif
						ret = true;
					}
				}
			}			
		}
	}

	return ret;
}

/** @brief applyControlGimbalWithRC
    @return bool
*/
#define APPLY_CONTROL_GIMBAL_WITH_RC_PRINT_DEBUG	0
bool mavlinkHandle_t::applyControlGimbalWithRC(modeRC_control_gimbal_t modeRC, bool RcOrMavlink)
{
	bool ret = false;
	static bool checkGimbalHome = false;
	static bool checkModeControl = false;
	static bool checkRCintput = false;
	static bool removeCommandAck = false;
	static uint32_t timeWaittingAckCommand = 0;
	static uint32_t timeDebug = 0;
	static uint8_t countTimeOut_setGimbalRcControl = 0;

	if(millis() - timeDebug > 1000 || timeDebug == 0)
	{
		timeDebug = millis();

		#if (APPLY_CONTROL_GIMBAL_WITH_RC_PRINT_DEBUG == 1)
			Serial.println("[applyControlGimbalWithRC] checkGimbalHome : " + String(checkGimbalHome) + " | checkModeControl : " + String(checkModeControl) + " | checkRCintput : " + String(checkRCintput));
		#endif
	}

	if(checkGimbalHome == false)
	{
		checkGimbalHome = getGimbalReturnHome();

		control.type = COMMAND_WGRH;
	}
	else
	{
		if(checkModeControl == false)
		{
			checkModeControl = requestGimbalModeRC(modeRC);
		}
		else
		{
			if(checkRCintput == false)
			{
				if(millis() - timeWaittingAckCommand > 1000 || timeWaittingAckCommand == 0)
				{
					timeWaittingAckCommand = millis();

					if(removeCommandAck == false)
					{
						removeCommandAck = true;

						mavlinkSerial2.ackCommand.command = 0;
						mavlinkSerial2.ackCommand.result = 0;
					}

					if(RcOrMavlink == true)
					{
						mavlink_remoteControl(MAVLINK_COMM_0, REMOTE_CONTROL_MODE);

						#if (APPLY_CONTROL_GIMBAL_WITH_RC_PRINT_DEBUG == 1)
							Serial.println("[applyControlGimbalWithRC] REMOTE_CONTROL_MODE ... command : " + String(mavlinkSerial2.ackCommand.command) + " | result : " + String(mavlinkSerial2.ackCommand.command));
						#endif
					}
					else
					{
						mavlink_remoteControl(MAVLINK_COMM_0, MAVLINK_CONTROL_MODE);

						#if (APPLY_CONTROL_GIMBAL_WITH_RC_PRINT_DEBUG == 1)
							Serial.println("[applyControlGimbalWithRC] MAVLINK_CONTROL_MODE ... command : " + String(mavlinkSerial2.ackCommand.command) + " | result : " + String(mavlinkSerial2.ackCommand.command));
						#endif
					}

					if(++countTimeOut_setGimbalRcControl > 5)
					{
						countTimeOut_setGimbalRcControl = 0;

						#if (APPLY_CONTROL_GIMBAL_WITH_RC_PRINT_DEBUG == 1)
							Serial.println("[applyControlGimbalWithRC] time out set gimbal RC control ...... reboot Gimbal");
						#endif
						mavlink_set_gimbal_reboot(MAVLINK_COMM_0);
					}
				}		

				if(mavlinkSerial2.ackCommand.command == MAV_CMD_DO_MOUNT_CONFIGURE)
				{
					#if (APPLY_CONTROL_GIMBAL_WITH_RC_PRINT_DEBUG == 1)
						Serial.println("[applyControlGimbalWithRC] DONE");
					#endif
					checkRCintput = true;
				}
			}
			else
			{
				checkGimbalHome = false;
				checkModeControl = false;
				checkRCintput = false;
				timeWaittingAckCommand = 0;
				removeCommandAck = false;

				ret = true;
			}
		}
	}


	return ret;
}

/** @brief applyControlJig
    @return none
*/
#define APPLY_CONTROL_JIG_PRINT_DEBUG	0
bool mavlinkHandle_t::applyControlJig(modeRC_control_gimbal_t modeControl, controlJigMode_t modeInput, controlJigMode_t modeOutput)
{
	bool ret = false;

	static bool applyControl = false;
	static uint32_t timeSendModeRcControl = 0;
	static uint8_t countMode = 0;
	static bool getResultQcMode = false;

	if(applyControl == false)
	{
		static bool rcInput = true;

		if(modeInput < CONTROL_JIG_MODE_COM2) rcInput = true;
		else rcInput = false;

		applyControl = applyControlGimbalWithRC(modeControl, rcInput);

		QcStatus = 0;
	}
	else
	{
		control.type = COMMAND_START;

		/// running
		// management.qcModeStatus = JIG_QC_MODE_STATUS_RUNNING;
		QcStatus = 1;

		if(mavlinkSerial1.heartBeat.type == COMMAND_START)
		{
			if(millis() - timeSendModeRcControl > 1000 || timeSendModeRcControl == 0)
			{
				timeSendModeRcControl = millis();

				// control.base_mode = (uint8_t)modeInput;

				if(++countMode > 15)
				{
					#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
						Serial.println("[controlJig] timeOut ---> next mode: " + String(modeControl));
					#endif
					applyControl = false;
					timeSendModeRcControl = 0;
					countMode = 0;

					/// QcModeStatus FAIL
					// management.qcModeStatus = JIG_QC_MODE_STATUS_FAILED;
					QcStatus = 3;

					/// check result fail
					checkresultQC = true;

					getResultQcMode = true;

					#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
						Serial.printf("[applyControlJig] qcModeStatus : %d\n", management.qcModeStatus);
					#endif
					ret = true;
				}
				else
				{
					if(mavlinkSerial1.heartBeat.base_mode == (uint8_t)modeInput)
					{
						#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
							Serial.println("[controlJig] apply mode : " + String(modeInput) + " | countControl : " + String(countMode));
						#endif
					}
					else
					{
						#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
							Serial.println("[controlJig] send mode control : " + String(modeInput));
						#endif
					}

					/// apply mode test COM2
					if(mavlinkSerial1.heartBeat.base_mode == CONTROL_JIG_MODE_COM2)
					{
						controlGimbal(0, 0, 0, LOCK_MODE, false);

						if((mavlinkSerial1.heartBeat.custom_mode & 0x08) == 0x08)
						{
							#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
								Serial.println("[controlJig] test DONE ---> next mode: " + String(modeOutput));
							#endif

							applyControl = false;
							timeSendModeRcControl = 0;
							countMode = 0;

							/// QcModeStatus PASS
							// management.qcModeStatus = JIG_QC_MODE_STATUS_PASSED;
							QcStatus = 2;

							getResultQcMode = true;

							#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
								Serial.printf("[applyControlJig] qcModeStatus : %d\n", QcStatus);//management.qcModeStatus);
							#endif
							ret = true;
						}
					}
					else
					{
						if(mavlinkSerial1.heartBeat.autopilot == 2)
						{
							#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
								Serial.println("[controlJig] test DONE ---> next mode: " + String(modeOutput));
							#endif
							applyControl = false;
							timeSendModeRcControl = 0;
							countMode = 0;

							/// QcModeStatus PASS
							// management.qcModeStatus = JIG_QC_MODE_STATUS_PASSED;
							QcStatus = 2;

							getResultQcMode = true;

							#if (APPLY_CONTROL_JIG_PRINT_DEBUG == 1)
								Serial.printf("[applyControlJig] qcModeStatus : %d\n", QcStatus);//management.qcModeStatus);
							#endif
							ret = true;
						}
					}
				}
			}
			else
			{
				
			}
		}
	}

	return ret;
}

/** @brief controlJig
    @return none
*/
void mavlinkHandle_t::controlJig(void)
{
	switch (state)
	{
		case CONTROL_JIG_STATE_IDLE:
		{
			#if (DEBUG_STATE == 1)

				Serial.println("CONTROL_JIG_STATE_IDLE");
				delay(1000);
				state = CONTROL_JIG_STATE_START;

			#else 
				static uint32_t timeWaitCommandStatus = 0;
				static uint32_t timeWattingJigReset = 0;
				static uint8_t resetJigCount = 0;
				static bool flagJigReset = false;

				mode = CONTROL_JIG_MODE_SBUS;

				if(timeWattingJigReset == 0)
				{
					control.type = COMMAND_RESET;

					Serial.println("CONTROL_JIG_STATE_IDLE");
				}

				if(millis() - timeWattingJigReset > 1000 && flagJigReset == false)
				{
					timeWattingJigReset = millis();

					flagJigReset = true;
				}

				if(flagJigReset == true)
				{
					/// watting jig reset
					if(resetJigCount == 3)
					{
						control.type = COMMAND_STOP;

						/// kiem tra trang thai jig
						if(mavlinkSerial1.heartBeat.system_status == COMMAND_STATUS_STANDBY)
						{
							state = CONTROL_JIG_STATE_SETTING_PARAM_GIMBAL;
							Serial.println("CONTROL_JIG_STATE_SETTING_PARAM_GIMBAL");

							flagJigReset = false;
							resetJigCount = 0;
							timeWattingJigReset = 0;
							timeWaitCommandStatus = 0;		
						}
					}

					if(millis() - timeWaitCommandStatus > 1000 || timeWaitCommandStatus == 0)
					{
						timeWaitCommandStatus = millis();

						resetJigCount ++;

						if(resetJigCount > 10)
						{
							resetJigCount = 0;
							control.type = COMMAND_RESET;
						}

						Serial.println("[controlJig] Waitting feed back command status from STM32 ... count : " + String(resetJigCount));
					}					
				}
			#endif
		}break;
		case CONTROL_JIG_STATE_SETTING_PARAM_GIMBAL :
		{
			static bool enableSEttingParam = false;

			control.type = COMMAND_SETPARAM;

			if(management.productReady == true)
			{
				enableSEttingParam = true;
			}

			if(enableSEttingParam == true)
			{
				if(settingParamGimbal() == true)
				{
					enableSEttingParam = false;

					/// next state
					state = CONTROL_JIG_STATE_START;
					Serial.println("CONTROL_JIG_STATE_START");
				}
			}
		}break;
		case CONTROL_JIG_STATE_START:
		{
			#if (DEBUG_STATE == 1)

				Serial.println("CONTROL_JIG_STATE_START");
				delay(1000);
				state = CONTROL_JIG_STATE_RUNNING;

			#else 
				static uint32_t timeWaitCommandStart = 0;

				if(millis() - timeWaitCommandStart > 1000 || timeWaitCommandStart == 0)
				{
					timeWaitCommandStart = millis();

					control.type = COMMAND_WGRH;

					Serial.println("[controlJig] Waitting feed back command start and status from STM32 ...");
				}

				/// wait reciever command gimbal return home and command status from stm32
				if(mavlinkSerial1.heartBeat.type == COMMAND_WGRH || mavlinkSerial1.heartBeat.system_status == COMMAND_STATUS_RUNNING)
				{
					/// next state
					state = CONTROL_JIG_STATE_RUNNING;
					control.base_mode = CONTROL_JIG_MODE_SBUS;
					Serial.println("CONTROL_JIG_STATE_RUNNING");
				}			
			#endif
		}break;
		case CONTROL_JIG_STATE_RUNNING:
		{
			#if (DEBUG_STATE == 1)

				Serial.println("CONTROL_JIG_STATE_RUNNING");

			#else 
			#endif

			switch (mode)
			{
				case CONTROL_JIG_MODE_SBUS:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_SBUS");
						delay(1000);
						mode = CONTROL_JIG_MODE_PPM;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_SBUS;
						
						if(applyControlJig(GIMBAL_RC_MODE_SBUS, modeInput, CONTROL_JIG_MODE_PPM) == true)
						{
							mode = CONTROL_JIG_MODE_PPM;
							control.base_mode = CONTROL_JIG_MODE_PPM;
							Serial.println("CONTROL_JIG_MODE_PPM");
						}

					#endif
				}break;
				case CONTROL_JIG_MODE_PPM:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_PPM");
						delay(1000);
						mode = CONTROL_JIG_MODE_CAN;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_PPM;
						
						if(applyControlJig(GIMBAL_RC_MODE_PPM, modeInput, CONTROL_JIG_MODE_CAN) == true)
						{
							mode = CONTROL_JIG_MODE_CAN;
							control.base_mode = CONTROL_JIG_MODE_CAN;
							Serial.println("CONTROL_JIG_MODE_CAN");
						}

					#endif
				}break;
				case CONTROL_JIG_MODE_CAN:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_CAN");
						delay(1000);
						mode = CONTROL_JIG_MODE_COM2;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_CAN;
						
						if(applyControlJig(GIMBAL_RC_MODE_CAN, modeInput, CONTROL_JIG_MODE_COM2) == true)
						{
							mode = CONTROL_JIG_MODE_COM2;
							control.base_mode = CONTROL_JIG_MODE_COM2;
							Serial.println("CONTROL_JIG_MODE_COM2");
						}
					
					#endif
				}break;
				case CONTROL_JIG_MODE_COM2:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_COM2");
						delay(1000);
						mode = CONTROL_JIG_MODE_COM4;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_COM2;
						
						if(applyControlJig(GIMBAL_RC_MODE_MAVLINK, modeInput, CONTROL_JIG_MODE_COM4) == true)
						{
							mode = CONTROL_JIG_MODE_COM4;
							control.base_mode = CONTROL_JIG_MODE_COM4;
							Serial.println("CONTROL_JIG_MODE_COM4");
						}

					#endif
				}break;
				case CONTROL_JIG_MODE_COM4:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_COM4");
						delay(1000);
						mode = CONTROL_JIG_MODE_AUX;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_COM4;
						
						if(applyControlJig(GIMBAL_RC_MODE_MAVLINK, modeInput, CONTROL_JIG_MODE_AUX) == true)
						{
							mode = CONTROL_JIG_MODE_AUX;
							control.base_mode = CONTROL_JIG_MODE_AUX;
							Serial.println("CONTROL_JIG_MODE_AUX");
						}

					#endif
				}break;
				case CONTROL_JIG_MODE_AUX:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_AUX");
						delay(1000);
						mode = CONTROL_JIG_MODE_VIRATE;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_AUX;
						
						if(applyControlJig(GIMBAL_RC_MODE_MAVLINK, modeInput, CONTROL_JIG_MODE_VIRATE) == true)
						{
							mode = CONTROL_JIG_MODE_VIRATE;
							control.base_mode = CONTROL_JIG_MODE_VIRATE;
							Serial.println("CONTROL_JIG_MODE_VIRATE");
						}

					#endif
				}break;
				case CONTROL_JIG_MODE_VIRATE:
				{
					#if (DEBUG_STATE == 1)

						Serial.println("CONTROL_JIG_MODE_VIRATE");
						delay(1000);
						state = CONTROL_JIG_STATE_DONE;

					#else 

						controlJigMode_t modeInput = CONTROL_JIG_MODE_VIRATE;
						
						if(applyControlJig(GIMBAL_RC_MODE_MAVLINK, modeInput, CONTROL_JIG_MODE_DONE) == true)
						{
							mode = CONTROL_JIG_MODE_CHECK_ALL_RESULT;
							control.base_mode = CONTROL_JIG_MODE_CHECK_ALL_RESULT;
							Serial.println("CONTROL_JIG_MODE_CHECK_ALL_RESULT");
						}

					#endif
				}break;
				case CONTROL_JIG_MODE_CHECK_ALL_RESULT:
				{
					/// kiem tra ket qua
					if(mavlinkSerial1.heartBeat.custom_mode == 0x7F)
					{
						mode = CONTROL_JIG_MODE_DONE;
						Serial.println("CONTROL_JIG_MODE_DONE");
					}
					else
					{
						mode = CONTROL_JIG_MODE_ERROR;
						Serial.println("CONTROL_JIG_MODE_ERROR");
					}
				}break;
				
				default:
					break;
			}
		}break;
		case CONTROL_JIG_STATE_DONE:
		{
			#if (DEBUG_STATE == 1)
				Serial.println("CONTROL_JIG_STATE_DONE");
				delay(1000);
				state = CONTROL_JIG_STATE_RESET;
			#else

				if(mavlinkSerial2.gimbalStatus.mode != 0)
				{
					mavlink_control_motor(MAVLINK_COMM_0, TURN_OFF);
					control.type = 0;

					// delay(10000);
					// ESP.restart();
				}
			#endif
		}break;
		case CONTROL_JIG_STATE_RESET:
		{
			#if (DEBUG_STATE == 1)
				Serial.println("CONTROL_JIG_STATE_RESET");
				delay(1000);
				state = CONTROL_JIG_STATE_IDLE;
			#endif
		}break;
		
		default:
			break;
	}
}

/** @brief jigProcessState_IDLE
    @return none
*/
bool mavlinkHandle_t::jigProcessState_IDLE(void)
{
	bool ret = false;
	static bool sendFirstJigStatus = false;
	static bool sendJigStatusReady = false;
	static bool firstRun = false;

	if(sendFirstJigStatus == false)
	{
		sendFirstJigStatus = true;

		/// send jigStatus lan dau
		management.flagSendJigStatus = true;

		Serial.println("[jigProcessState_IDLE] : send first jig status");
	}

	if(management.jigReady ==  true && sendJigStatusReady == false)
	{
		sendJigStatusReady = true;

		/// set variables jig status running
		management.jigStatus = JIG_STATUS_STANBY;

		// send jigStatus ready
		management.flagSendJigStatus = true;

		Serial.println("[jigProcessState_IDLE] : send jig status ready");
	}

	if(management.jigReady ==  true && management.productReady == true)
	{
		sendFirstJigStatus = false;
		sendJigStatusReady = false;

		// send jigStatus ready and product ready
		management.flagSendJigStatus = true;

		if(firstRun == false)
		{
			firstRun = true;

			/// set variables product on status
			management.productOnState = PRODUCT_WAIT_FOR_QC;
		}

		/// send product on jigStatus
		management.flagSendProductOnJigStatus = true;

		/// send Qcmode
		management.flagSendJigQcMode = true;

		Serial.println("[jigProcessState_IDLE] : send jig status & product ready");

		ret = true;
	}

	return ret;
}

/** @brief jigProcessState_CONNECT
    @return none
*/
bool mavlinkHandle_t::jigProcessState_CONNECT(void)
{
	bool ret = true;



	return ret;
}

/** @brief jigProcessState_PROCESS
    @return none
*/
bool checkReplaceValue(int *value, uint8_t newValue)
{
	bool ret = false;

	if(*value != newValue)
	{
		/// get new value
		*value = newValue;

		ret = true;
		Serial.println("[checkReplaceValue] ok");
	}

	return ret;
}

/** @brief jigProcessState_PROCESS
    @return none
*/
bool mavlinkHandle_t::jigProcessState_PROCESS(void)
{
	bool ret = false;
	static jigControl_t BLEcontrol;
	static bool jigStatusRunning = false;
	static bool flagWQcModeStatus = false;
	static uint32_t timeWQcModeStatus = 0;
	static int checkQcMode = 0;
	static int checkQcStatus = -1;

	if(management.newControl == true)
	{
		management.newControl = false;

		BLEcontrol = management.getJigControl();
	}

	if(BLEcontrol == JIG_CONTROL_STOP)
	{
		/// kiem tra QcMode
		if(mavlinkSerial1.heartBeat.base_mode != 0)
		{
			Serial.println("[jigProcessState_PROCESS] control STOP back to state IDLE");
			management.jigState = JIG_STATE_IDLE;
		}
	}
	else if(BLEcontrol == JIG_CONTROL_START || BLEcontrol == JIG_CONTROL_RESUME)
	{
		/// send jig status running
		if(jigStatusRunning == false)
		{	
			jigStatusRunning = true;

			/// set variables jig status running
			management.jigStatus = JIG_STATUS_RUNNING;

			/// set flag send jig status
			management.flagSendJigStatus = true;

			/// set variables product on status
			management.productOnState = PRODUCT_RUNNING;

			/// set flag send product On jig status
			management.flagSendProductOnJigStatus = true;
		}
		
		if(checkReplaceValue(&checkQcMode, mavlinkSerial1.heartBeat.base_mode) == true)
		{
			/// reset QCMode Status
			management.qcModeStatus = JIG_QC_MODE_STATUS_IDLE;

			management.qcModeStatus = (JigTestQcModeStatus_t)QcStatus;

			/// set flag send Qcmode
			management.flagSendJigQcMode = true;
		}

		if(checkReplaceValue(&checkQcStatus, QcStatus) == true && flagWQcModeStatus == false)
		{
			flagWQcModeStatus = true;
		}

		if(flagWQcModeStatus == true)
		{
			if(millis() - timeWQcModeStatus > 2000)
			{
				timeWQcModeStatus = millis();

				flagWQcModeStatus = false;

				Serial.printf("[jigProcessState_PROCESS] qcStatus : %d\n", QcStatus);//management.qcModeStatus);

				management.qcModeStatus = (JigTestQcModeStatus_t)QcStatus;

				/// set flag send QcMode Status
				management.flagSendJigQcMode = true;
			}
		}

		controlJig();
	}
	else if(BLEcontrol == JIG_CONTROL_PAUSE)
	{
		static uint32_t timePause = 0;

		if(millis() - timePause > 1000)
		{
			timePause = millis();

			Serial.println("[jigProcessState_PROCESS] JIG_CONTROL_PAUSE");

			/// set flag send control
			management.flagSendJigControl = true;
		} 		
	}

	/// check running DONE
	if(mavlinkSerial1.heartBeat.base_mode == CONTROL_JIG_MODE_CHECK_ALL_RESULT)
	{
		jigStatusRunning = false;
		flagWQcModeStatus = false;
		timeWQcModeStatus = 0;
		BLEcontrol = JIG_CONTROL_STOP;
		checkQcMode = 0;
		checkQcStatus = -1;


		/// set variables product on status
		management.productOnState = checkresultQC ? PRODUCT_FAIL : PRODUCT_COMPLETE;

		/// set flag send product On jig status
		management.flagSendProductOnJigStatus = true;

		ret = true;
	}
	
	return ret;
}

/** @brief jigProcessState_DONE
    @return none
*/
bool mavlinkHandle_t::jigProcessState_DONE(void)
{
	bool ret = false;
	static uint32_t timeBackToStanby = 0;
	static uint8_t countBackToStanby = 0;

	if(millis() - timeBackToStanby > 1000)
	{
		timeBackToStanby = millis();

		Serial.printf("[jigProcessState_DONE] countBackToStanby : %d\n", countBackToStanby);

		/// reset stm32
		control.type = COMMAND_RESET;

		control.base_mode = 0;

		if(++countBackToStanby >= 4)
		{
			timeBackToStanby = 0;
			countBackToStanby = 0;

			management.JigControlBuffer[0] = 0x00;
			management.sendProfileDone = false;

			/// set variables jig status running
			management.jigStatus = JIG_STATUS_STANBY;

			// send jigStatus ready
			management.flagSendJigStatus = true;

			control.type = COMMAND_STOP;

			state = CONTROL_JIG_STATE_IDLE;

			Serial.printf("[jigProcessState_DONE] back to stanby approve \n");

			ret = true;
		}
	}

	return ret;
}

/** @brief jigProcessState
    @return none
*/
void mavlinkHandle_t::jigProcessState(void)
{
	static uint32_t timePrintDebug = 0;
	static uint32_t timeOutHeartBeat = 0;

	if(millis() - timeOutHeartBeat > 2000 || timeOutHeartBeat == 0)
	{
		timeOutHeartBeat = millis();

		/// get jig ready
		management.jigReady = getSerial1HeartBeatReady(0);

		/// get product ready
		management.productReady = getSerial2HeartBeatReady(0);
	}

	switch (management.jigState)
	{
		case JIG_STATE_IDLE:
		{
			if(jigProcessState_IDLE() == true)
			{
				management.jigState = JIG_STATE_CONNECT;
			}
		}
		break;
		case JIG_STATE_CONNECT:
		{
			if(jigProcessState_CONNECT() == true)
			{
				management.jigState = JIG_STATE_PROCESS;
			}
		}
		break;
		case JIG_STATE_PROCESS:
		{
			if(jigProcessState_PROCESS() == true)
			{
				management.jigState = JIG_STATE_DONE;
			}
		}
		break;
		case JIG_STATE_DONE:
		{
			if(jigProcessState_DONE() == true)
			{
				management.jigState = JIG_STATE_IDLE;
			}
		}
		break;
		
		default:
			break;
	}

	if(millis() - timePrintDebug > 1000)
	{
		timePrintDebug = millis();

		Serial.printf("[jigProcessState] state : %d\n", management.jigState);
	}
}

/** @brief process
    @return none
*/
void mavlinkHandle_t::process( void *pvParameters )
{
	// static uint32_t timeOutHeartBeat = 0;

	sendData();
	recieverData();

	jigProcessState();

	// if(millis() - timeOutHeartBeat > 2000 || timeOutHeartBeat == 0)
	// {
	// 	timeOutHeartBeat = millis();

	// 	/// get jig ready
	// 	management.jigReady = getSerial1HeartBeatReady(0);

	// 	/// get product ready
	// 	management.productReady = getSerial2HeartBeatReady(0);
	// }

	// /// get jigControl from app
	// jigControl_t jigControl = management.getJigControl();

	// if(jigControl == JIG_CONTROL_STOP)
	// {

	// }
	// else if(jigControl == JIG_CONTROL_START)
	// {
	// 	controlJig();
	// }
	// else if(jigControl == JIG_CONTROL_PAUSE)
	// {

	// }
	// else if(jigControl == JIG_CONTROL_RESUME)
	// {

	// }
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
