# Use Heartbeat msg :

    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t type; /*<  Type of the MAV (quadrotor, helicopter, etc.)*/
    uint8_t autopilot; /*<  Autopilot type / class.*/
    uint8_t base_mode; /*<  System mode bitmap.*/
    uint8_t system_status; /*<  System status flag.*/
    uint8_t system_status_send; /*<  System status flag.*/
    uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

# STM32

- send :
	+ system_status : 1 : STANBY | 2 : RUNNING | 3 : DONE | 4 : SYS_RESET
	+ custom_mode : result test (0x00 - 0b00000000) pass : 1 | fail : 0
- reciever :
	+ type : 1 : START | 2 STOP
	+ base_mode : modeRC_control (1 : SBUS, 2 : PPM, 3 : CAN, 4 : COM2, 5 : COM4, 6 : AUX, 7 : VIRATE, ...)

# ESP32

- send :
	+ type : 1 : START | 2 STOP
	+ base_mode : modeRC_control (1 : SBUS, 2 : PPM, 3 : CAN, 4 : COM2, 5 : COM4, 6 : AUX, 7 : VIRATE, ...)
- reciever
	+ system_status : 1 : STANBY | 2 : RUNNING | 3 : DONE | 4 : SYS_RESET
	+ custom_mode : result test (0x00 - 0b00000000) pass : 1 | fail : 0
