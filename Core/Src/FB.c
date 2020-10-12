#include <FB.h>

void forward(void)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, SET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, SET);
}

void backward(void)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, RESET);
}

void left(void)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, SET);
}

void right(void)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, SET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, RESET);
}
