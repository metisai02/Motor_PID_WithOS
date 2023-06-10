void tune_PID_after2(Select_Tune select)
{
  switch (select)
  {
  case Select_Posi:
    if (count_PID_position_first_time == true)
    {
      control_PID_Position(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);      // theo độ, tinh o lan dau vua moi chay dong co 1 lan duy nhat
      control_PID_Velocity(&PID_contr, output_pid_posi, Kp_true, Ki_true, Kd_true); // setpoint cua speed bang voi output cua position
      output_pid = output_pid_velo;
      PWM_control_position(&htim1, output_pid);
      count_PID_position_first_time = false;
      break;
    }
    if (count_PID == 3) // 3 lan tinh PID toc do moi tinh 1 lan PID vi tri
    {
      //      if (countUpdate == 250)
      //      {
      //        setpointQt += 10;
      //        countUpdate = 0;
      //      }
      control_PID_Position(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);      // tinh lai output_pid_posi moi
      control_PID_Velocity(&PID_contr, output_pid_posi, Kp_true, Ki_true, Kd_true); // setpoint cua speed bang voi output cua position
      output_pid = output_pid_velo;
      PWM_control_position(&htim1, output_pid);
      count_PID = 0;
      break;
    }
    else if (count_PID != 3)
    {
      //      if (countUpdate == 250)
      //      {
      //        setpointQt += 10;
      //        countUpdate = 0;
      //      }
      control_PID_Velocity(&PID_contr, output_pid_posi, Kp_true, Ki_true, Kd_true); // setpoint cua speed bang voi output cua position
      output_pid = output_pid_velo;
      PWM_control_position(&htim1, output_pid);
      break;
    }
  case Select_Velo:
    //    if (countUpdate == 250)
    //    {
    //      setpointQt += 10;
    //      countUpdate = 0;
    //    }
    control_PID_Velocity(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);
    output_pid = output_pid_velo; // dong nhat het ve output_pid cho de kiem soat @_@
    PWM_control_velocity(&htim1, output_pid);
    break;
  default:
    break;
  }
}
void tune_PID_after1(Select_Tune select)
{
  switch (select)
  {
  case Select_Posi:
    control_PID_Position(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);
    output_pid = output_pid_posi;
    PWM_control_position(&htim1, output_pid);
    break;
  case Select_Velo:
    control_PID_Velocity(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);
    output_pid = output_pid_velo;
    PWM_control_velocity(&htim1, output_pid);
    break;
  default:
    break;
  }
}
void control_PID_Velocity(PID_control *pid_tune, float setpoint_velo, float Kp, float Ki, float Kd)
{ // velocity vong/phut
  velocity_real = (float)instance_enc.speed_by_encoder * 60.0f / (Ts * Pulseee);
  velocity_real = update_para_LPF(velocity_real);
  error_velo = setpoint_velo - (velocity_real);
  instance_enc.velocity = velocity_real;
  pid_tune->P_part = error_velo;
  pid_tune->I_part += error_velo * Ts;
  pid_tune->D_part = (error_velo - pre_error_velo) / Ts;
  //  if(error_velo < 0.005f*setpoint_velo)
  //  {
  //	pid_tune->I_part = 0;
  //  }
  output_pid_velo = Kp * (pid_tune->P_part) + Ki * (pid_tune->I_part) + Kd * (pid_tune->D_part);
  if (output_pid_velo > 90.0)
  {
    output_pid_velo = 90.0;
  }
  else if (output_pid_velo < -90.0)
  {
    output_pid_velo = -90.0;
  }
  pre_error_velo = error_velo;
}
void control_PID_Position(PID_control *pid_tune, float setpoint_posi_rotation, float Kp, float Ki, float Kd)
{

  now_position = (float)instance_enc.position * 360 / Pulseee; // now_position = độ
  number_rotation = now_position / 360;
  //	setpoint_posi_degrees = setpoint_posi_rotation*360;   // setpoint_posi_rotation la set số vòng cho dễ set
  //	now_position1 = 0.85*now_position1 + 0.15*now_position;
  error_posi = setpoint_posi_rotation - (now_position);
  pid_tune->P_part = error_posi;
  pid_tune->I_part += error_posi * Ts;
  pid_tune->D_part = (error_posi - pre_error_posi) / Ts;
  if (error_posi < 0.001f * setpoint_posi_rotation)
  {
    pid_tune->I_part = 0;
  }
  output_pid_posi = Kp * (pid_tune->P_part) + Ki * (pid_tune->I_part) + Kd * (pid_tune->D_part);
  if (output_pid_posi > 90.0)
  {
    output_pid_posi = 90.0;
  }
  else if (output_pid_posi < -90.0)
  {
    output_pid_posi = -90.0;
  }
  //	else if(output_pid < 0)
  //	{
  //		output_pid = 0;
  //	}
  pre_error_posi = error_posi;
}
void PWM_control_position(TIM_HandleTypeDef *htim, float duty)
{
  //	if(duty>90.0)
  //	{
  //		duty = 90.0;
  //	}
  if (duty > 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // chieu thuan cung chieu kim dong ho
                                                                 //    htim1.Instance->CCR2 = duty * (htim1.Instance->ARR) / 100;
                                                                 //    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR3 = (duty) * (htim1.Instance->ARR) / 100; // nguoc chieu kim dong ho
                                                                 //    htim1.Instance->CCR2 = 0;

    ;
    //		htim1.Instance->CCR3 =  duty*900/100;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    htim1.Instance->CCR3 = (-duty) * (htim1.Instance->ARR) / 100; // nguoc chieu kim dong ho
                                                                  //    htim1.Instance->CCR2 = 0;
                                                                  //    htim1.Instance->CCR2 = duty * (htim1.Instance->ARR) / 100;
                                                                  //    htim1.Instance->CCR3 = 0;
  }
  else
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    //	  htim1.Instance->CCR2 = 0;
    //	  htim1.Instance->CCR3 = 0;
  }
}
void PWM_control_velocity(TIM_HandleTypeDef *htim, float duty)
{
  if (duty > 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET); // chieu thuan cung chieu kim dong ho
                                                               //    htim1.Instance->CCR2 = duty * (htim1.Instance->ARR) / 100;
                                                               //    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR3 = (duty) * (htim1.Instance->ARR) / 100;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    htim1.Instance->CCR3 = (-duty) * (htim1.Instance->ARR) / 100; // nguoc chieu kim dong ho
                                                                  //    htim1.Instance->CCR2 = 0;
  }
  else
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    //	  htim1.Instance->CCR2 = 0;
    //	  htim1.Instance->CCR3 = 0;
  }
}

void encoder()
{
  if (data_after_cut[0] == 0x44)
  {
    htim2.Instance->CNT = 0;
    instance_enc.speed_by_encoder = 0;
    instance_enc.pre_speed_by_encoder = 0;
  }
  else
  {
    instance_enc.speed_by_encoder = htim2.Instance->CNT - instance_enc.pre_speed_by_encoder; // so xung giua 2 lan doc encoder
    //	htim2.Instance->CNT = 0;
    instance_enc.pre_speed_by_encoder = htim2.Instance->CNT;
    //	instance_enc.speed_by_encoder = htim2.Instance->CNT;
    instance_enc.position += instance_enc.speed_by_encoder;
  }
  //	htim2.Instance->CNT = 0;
}
void first_para_LPF()
{
  // Ham dung de khoi tao gia tri ban dau bang 0 cho cac thong so bo loc thong thap
  LPF_para.LPF_output = 0;
  LPF_para.pre_LPF_output = 0;
}

float update_para_LPF(float input)
{
  LPF_para.LPF_output = LPF_para.pre_LPF_output * 0.1f + 0.9f * input;
  LPF_para.pre_LPF_output = LPF_para.LPF_output;
  return LPF_para.LPF_output;
}