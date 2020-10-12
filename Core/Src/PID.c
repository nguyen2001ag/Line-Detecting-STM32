#include "PID.h"

void PID_SET_PARAMs(PID_parameter* pid_parameters,float Kp,float Ki,float Kd)
{
    pid_parameters->Kp = Kp;
    pid_parameters->Ki = Ki;
    pid_parameters->Kd = Kd;
}
void PID_PROCESS(PID_parameter* pid_parameter, float vitri,float setpoint)   //setpoint = 0;
{
    pid_parameter->error =  vitri - setpoint;

    pid_parameter->KP_part = pid_parameter->Kp * (pid_parameter->error - pid_parameter->pre_error);
    pid_parameter->KI_part = 0.5* pid_parameter->Ki * pid_parameter->Ts * (pid_parameter->error + pid_parameter->pre_error);
    pid_parameter->KD_part =(pid_parameter->Kd / pid_parameter->Ts) * (pid_parameter->error - 2*pid_parameter->pre_error +pid_parameter->pre2_error);

    pid_parameter->Out_right = pid_parameter->pre_Out_right- pid_parameter->KP_part - pid_parameter->KI_part -pid_parameter->KD_part;
    pid_parameter->Out_left = pid_parameter->pre_Out_left+ pid_parameter->KP_part + pid_parameter->KI_part +pid_parameter->KD_part;

    if (pid_parameter->Out_left > pid_parameter->PID_Saturation)
      {
            pid_parameter->Out_left = pid_parameter->PID_Saturation;
       }
      else if (pid_parameter->Out_left < (-pid_parameter->PID_Saturation))
      {
            pid_parameter->Out_left = -pid_parameter->PID_Saturation;
      }

    if (pid_parameter->Out_right > pid_parameter->PID_Saturation)
          {
                pid_parameter->Out_right = pid_parameter->PID_Saturation;
           }
          else if (pid_parameter->Out_right < (-pid_parameter->PID_Saturation))
          {
                pid_parameter->Out_right = -pid_parameter->PID_Saturation;
          }
    pid_parameter->pre2_error = pid_parameter->pre_error;
    pid_parameter->pre_error = pid_parameter->error;
    pid_parameter->pre_Out_right = pid_parameter->Out_right;
    pid_parameter->pre_Out_left = pid_parameter->Out_left;
}

float PID_ReadValue_left (PID_parameter* pid_parameter){
    return pid_parameter->Out_left;
}

float PID_ReadValue_right (PID_parameter* pid_parameter){
    return pid_parameter->Out_right;
}
