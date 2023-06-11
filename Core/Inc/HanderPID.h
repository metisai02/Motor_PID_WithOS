/*
 * HanderPID.h
 *
 *  Created on: Jun 10, 2023
 *      Author: DELL
 */

#ifndef INC_HANDERPID_H_
#define INC_HANDERPID_H_


#include "main.h"
typedef struct
{
  float P_part;
  float I_part;
  float D_part;
} PID_control;
typedef enum
{
  Select_Velo,
  Select_Posi,
} Select_Tune;
typedef enum
{
  Select_PID1 = 2,
  Select_PID2,
} Select_PID;

typedef struct
{
  float LPF_output;
  float pre_LPF_output;
} LPF_parameters;
typedef struct
{
  int32_t position;
  //	int32_t position_real;
  int16_t speed_by_encoder; // don vi: xung/(tgian ngat timer)
  int16_t pre_speed_by_encoder;
  short pre_counter;
  int32_t velocity; // vong/phut
  //	volatile float velocity_degrees_p_sec;
  //	volatile float velocity_not;
} instance_encoder;

void tune_PID_after1(Select_Tune select);
void tune_PID_after2(Select_Tune select);
void control_PID_Velocity(PID_control *pid_tune, float setpoint_velo, float Kp, float Ki, float Kd);
void control_PID_Position(PID_control *pid_tune, float setpoint_posi_rotation, float Kp, float Ki, float Kd);
float update_para_LPF(float input);
void PWM_control_position(TIM_HandleTypeDef *htim, float duty);
void PWM_control_velocity(TIM_HandleTypeDef *htim, float duty);
void encoder();
#endif /* INC_HANDERPID_H_ */
