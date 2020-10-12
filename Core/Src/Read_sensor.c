#include "Read_sensor.h"
#include "stdbool.h"


char sensor[4];
int state = 0;
float error;

float read_sensor_values(float pre_error)
{
  sensor[0]=HAL_GPIO_ReadPin(IS1_GPIO_Port, IS1_Pin);
  sensor[1]=HAL_GPIO_ReadPin(IS2_GPIO_Port, IS2_Pin);
  sensor[2]=HAL_GPIO_ReadPin(IS3_GPIO_Port, IS3_Pin);
  sensor[3]=HAL_GPIO_ReadPin(IS4_GPIO_Port, IS4_Pin);

  if((sensor[0]==!state)&&(sensor[1]==!state)&&(sensor[2]==!state)&&(sensor[3]==state))
  error=3;
  else if((sensor[0]==!state)&&(sensor[1]==!state)&&(sensor[2]==state)&&(sensor[3]==state))
  error=2;
  else if((sensor[0]==!state)&&(sensor[1]==!state)&&(sensor[2]==state)&&(sensor[3]==!state))
  error=1;
  else if((sensor[0]==!state)&&(sensor[1]==state)&&(sensor[2]==state)&&(sensor[3]==!state))
  error=0;
  else if((sensor[0]==!state)&&(sensor[1]==state)&&(sensor[2]==!state)&&(sensor[3]==!state))
  error=-1;
  else if((sensor[0]==state)&&(sensor[1]==state)&&(sensor[2]==!state)&&(sensor[3]==!state))
  error=-2;
  else if((sensor[0]==state)&&(sensor[1]==!state)&&(sensor[2]==!state)&&(sensor[3]==!state))
  error=-3;
  else if((sensor[0]==!state)&&(sensor[1]==!state)&&(sensor[2]==!state)&&(sensor[3]==!state)&&(sensor[4]==!state))
    {
	  if(pre_error==-3) error=-4;
      else error=4;
    }
  return error;
}
