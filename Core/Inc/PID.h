#ifndef _PID_H_
#define _PID_H_

typedef struct
{
            float Kp;
            float Ki;
            float Kd;
            float error ;
            float pre_error;
            float pre2_error;
            float Ts;
            float KP_part;
            float KI_part;
            float KD_part;
            float Out_left;
            float pre_Out_left;
            float Out_right;
            float pre_Out_right;
            float PID_Saturation;
}PID_parameter;

void PID_SET_PARAMs(PID_parameter* pid_parameters,float Kp,float Ki,float Kd);

void PID_PROCESS(PID_parameter* pid_parameter, float vitri,float setpoint);

float PID_ReadValue_left(PID_parameter* pid_parameter);

float PID_ReadValue_right(PID_parameter* pid_parameter);

#endif
