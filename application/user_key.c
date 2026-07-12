#include "user_key.h"
#include "button.h"
#include "gimbal.h"

static Button_TypeDef user_key;
static uint8_t laser_on = 0;

static void User_Key_Pressed_Callback(void)
{
	if (Gimbal_GetState() == GIMBAL_STATE_RUNNING)
		return;

	User_Laser_Set(!laser_on);
}

// 设置激光输出状态.
void User_Laser_Set(uint8_t enable)
{
	laser_on = enable ? 1U : 0U;
	HAL_GPIO_WritePin(jiguang_GPIO_Port, jiguang_Pin,
	                 laser_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void User_Key_Init(void)
{
	Button_InitTypeDef key_init;

	key_init.GPIOx = key_GPIO_Port;
	key_init.GPIO_Pin = key_Pin;

	User_Laser_Set(0U);
	My_Button_Init(&user_key, &key_init);
	My_Button_SetPressCb(&user_key, User_Key_Pressed_Callback);
}

void User_Key_Proc(void)
{
	My_Button_Proc(&user_key);
}
