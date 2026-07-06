#ifndef CONTROL_H
#define CONTROL_H

void Control_Init(void);
void Control_Proc(void);

void SetGimbal0Speed(float speed_rpm);   //yaw轴目标转速(rpm)
void SetGimbal1Speed(float speed_rpm);   //pitch轴目标转速(rpm)

#endif
