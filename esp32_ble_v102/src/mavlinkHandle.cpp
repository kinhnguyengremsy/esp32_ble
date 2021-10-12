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
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
mavlinkProtocol protocol;
mav_state_t mavlinkStateSerial2, mavlinkStateSerial1;
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
void mavlinkHandle_t::controlGimbal(int16_t tilt, int16_t roll, int16_t pan, control_gimbal_mode_t mode)
{
	static controlAngle_state state;
	static uint32_t timeState = 0;

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
			if(mavlinkSerial2.ackCommand.command == 0)
			{
				mavlink_set_gimbal_home(MAVLINK_COMM_0);
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
						if(deltaRoll < 5 && deltaTilt < 5 )
						{
							Serial.println("[set angle cw gimbal] : DONE");

							if(mavlinkSerial2.ackCommand.result == MAV_RESULT_IN_PROGRESS)
							state = CONTROL_ANGLE_STATE_MOVE_CCW;

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
							state = CONTROL_ANGLE_STATE_MOVE_CCW;

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
							state = CONTROL_ANGLE_STATE_MOVE_CCW;

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
							state = CONTROL_ANGLE_STATE_MOVE_CCW;

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

				ret = true;
			}
		}
	}

	if(millis() - timeOutRequestParamGimbal > 30000)
	{
		timeOutRequestParamGimbal = millis();

		Serial.println("[requestParamGimbal] timeOut request param gimbal .... try again !!!");

		/// reset paramValue receiver from gimbal
		uint8_t len = sizeof(mavlink_msg_param_value_t);
		uint8_t d = 0;
		memcpy(&mavlinkSerial2.paramValue, &d, len);

		paramIndex_temp = 0;
		countParamRead_true = 0;
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
bool mavlinkHandle_t::requestGimbalModeRC(modeRC_control_gimbal_t modeRC)
{
	bool ret = false;
	static uint32_t timeSendParam = 0;
	char* param_id = "RADIO_TYPE";

	if(mavlinkSerial2.paramValue.param_index == 28)
	{
		if(mavlinkSerial2.paramValue.param_value == (float)modeRC)
		{
			timeSendParam = 0;

			Serial.println("[requestGimbalModeRC] DONE ");
			
			ret = true;
		}
	}
	else
	{
		if(millis() - timeSendParam > 500 || timeSendParam == 0)
		{
			timeSendParam = millis();

			mavlink_set_param_gimbal(MAVLINK_COMM_0, (float)modeRC, param_id);
			Serial.println("[requestGimbalModeRC] send mode : " + String(modeRC));
		}
		else
		{
			static uint32_t timeRequestParam = 0;

			if(millis() - timeRequestParam > 100 || timeRequestParam == 0)
			{
				timeRequestParam = millis();

				mavlink_param_request_read(MAVLINK_COMM_0, 28, param_id);

				Serial.println("[requestGimbalModeRC] send request mode : " + String(modeRC));
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

	if(protocol.serial_readData(&Serial1, MAVLINK_COMM_1, &mavlinkStateSerial1))
	{
		mavlinkMessageHandle(MAVLINK_COMM_1, &mavlinkSerial1, &mavlinkStateSerial1);
	}
}

/** @brief process
    @return none
*/
void mavlinkHandle_t::process(void *arg)
{
	static bool offMotor = false;
	static bool modeRC = false;

	sendData();
	recieverData();

	if(modeRC == false)
	{
		modeRC = requestGimbalModeRC(GIMBAL_RC_MODE_MAVLINK);
	}

	if(mavlinkSerial2.gimbalStatus.mode != 0 && offMotor == false)
	{
		offMotor = true;

		mavlink_control_motor(MAVLINK_COMM_0, TURN_OFF);

		Serial.println("[gimbalStatus] motor : TURN OFF");
	}

}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
