//
// Created by hanjin on 24-11-13.
//

#ifndef OMNIDRIVE_H
#define OMNIDRIVE_H

#include "main.h"
#include "pid.h"
#include "motor.h"

/**
 * @note v0.0.1 平移运动与旋转运动是分开状态，只能分开调用
 */
#define version "0.0.1"

#define WHEEL_TO_CENTER (10.0f)         ///< 轮子到中心的距离(cm)
#define WHEEL_RADIUS    (8.0f)          ///< 轮子半径(cm)

#define INVSQRT2        (0.7071068f)    ///< sqrt(2) / 2
#define RAD2ROUND       (0.1591549f)    ///< 1/2π

// #define SPEED_LOOP_ERROR (3)

typedef enum
{
    SPEED_LOOP = 0U,
    POSITION_LOOP = 1U,
} WheelState_t;

typedef struct
{
    uint8_t id; ///< 从左上角起逆时针方向分别为0~3 轮子均顺时针旋转
    Motor_t motor; ///< 电机类
    PID_t speed_loop, position_loop; ///< 速度环pid和位置环pid
    WheelState_t state; ///< 处于哪种控制模式下
    uint8_t enable; ///< 是否开启
} Wheel_t;

typedef struct
{
    float vx, vy, omega;
} Velocity_t;

typedef struct
{
    float x, y;
} Displacement_t;

void switchWheelState(Wheel_t* wheel, WheelState_t newState);

void setVelocity(const Velocity_t velocity, Wheel_t* wheel);
void setDisplacement(const Displacement_t displacement, Wheel_t* wheel);
void setRotation(const float rotation, Wheel_t* wheel);


#endif //OMNIDRIVE_H
