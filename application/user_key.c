#include "user_key.h"
#include "button.h"

static Button_TypeDef user_key;
static uint8_t laser_on = 0;

static void User_Key_Pressed_Callback(void)
{
	laser_on = !laser_on;
	HAL_GPIO_WritePin(jiguang_GPIO_Port, jiguang_Pin, laser_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void User_Key_Init(void)
{
	Button_InitTypeDef key_init;

	key_init.GPIOx = key_GPIO_Port;
	key_init.GPIO_Pin = key_Pin;

	HAL_GPIO_WritePin(jiguang_GPIO_Port, jiguang_Pin, GPIO_PIN_RESET);
	My_Button_Init(&user_key, &key_init);
	My_Button_SetPressCb(&user_key, User_Key_Pressed_Callback);
}

void User_Key_Proc(void)
{
	My_Button_Proc(&user_key);
}
