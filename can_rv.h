#ifndef CAN_RV_H
#define CAN_RV_H

#include <stdint.h>
//#include "delay.h"
#include "zlgcan.h"

#define param_get_pos	0x01
#define param_get_spd	0x02
#define param_get_cur	0x03
#define param_get_pwr	0x04
#define param_get_acc	0x05
#define param_get_lkgKP	0x06
#define param_get_spdKI	0x07
#define param_get_fdbKP	0x08
#define param_get_fdbKD	0x09

#define comm_ack	0x00
#define comm_auto	0x01

typedef struct 
{
	uint16_t motor_id;
	uint8_t INS_code;		//instruction code.
	uint8_t motor_fbd;	//motor CAN communication feedback.
}MotorCommFbd;

typedef struct 
{
	uint16_t angle_actual_int;
	uint16_t angle_desired_int;
	int16_t speed_actual_int;
	int16_t speed_desired_int;
	int16_t current_actual_int;
	int16_t current_desired_int;
	float 	speed_actual_rad;
	float 	speed_desired_rad;
	float 	angle_actual_rad;	
	float   angle_desired_rad;
	uint16_t	motor_id;
	uint8_t 	temperature;
	uint8_t		error;
	float     angle_actual_float;
	float 		speed_actual_float;
	float 		current_actual_float;
	float     angle_desired_float;
	float 		speed_desired_float;
	float 		current_desired_float;
    float		power;
	uint16_t	acceleration;
	uint16_t	linkage_KP;
	uint16_t 	speed_KI;
	uint16_t	feedback_KP;
	uint16_t	feedback_KD;
}OD_Motor_Msg;

extern OD_Motor_Msg rv_motor_msg[8];
extern uint16_t motor_id_check;
extern MotorCommFbd motor_comm_fbd;

void MotorIDReset(ZCAN_Transmit_Data *can_Txdata);
void MotorIDSetting(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,uint16_t motor_id_new);
void MotorSetting(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,uint8_t cmd);
void MotorCommModeReading(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id);
void MotorIDReading(ZCAN_Transmit_Data *can_Txdata);
void send_motor_ctrl_cmd(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,float kp,float kd,float pos,float spd,float tor);
void set_motor_position(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status);
void set_motor_speed(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,float spd,uint16_t cur,uint8_t ack_status);
void set_motor_cur_tor(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,int16_t cur_tor,uint8_t ctrl_status,uint8_t ack_status);
void set_motor_acceleration(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,uint16_t acc,uint8_t ack_status);//
void set_motor_linkage_speedKI(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,uint16_t linkage,uint16_t speedKI,uint8_t ack_status);//
void set_motor_feedbackKP(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,uint16_t fdbKP,uint8_t ack_status);//
void get_motor_parameter(ZCAN_Transmit_Data *can_Txdata,uint16_t motor_id,uint8_t param_cmd);
void set_motors_current(ZCAN_Transmit_Data *can_Txdata,OD_Motor_Msg rv_MotorMsg[8],uint8_t motor_quantity);//
void RV_can_data_repack(ZCAN_Receive_Data can_Rxdata,uint8_t comm_mode);//
int float_to_uint(float x, float x_min, float x_max, int bits);

#endif