//
// Created by hanjin on 24-11-13.
//

#include "omni4.h"

#include "main.h"
#include <math.h>
#include <stdio.h>

#include "pid.h"

/**
 * @brief 切换轮子的控制状态
 * @param wheel
 * @param newState
 */
void switchWheelState(Wheel_t* wheel, WheelState_t newState)
{
    if (wheel->state == newState) return;
    wheel->state = newState;
    switch (newState)
    {
    case SPEED_LOOP:
        wheel->speed_loop.output = wheel->position_loop.output;
        break;
    case POSITION_LOOP:
        wheel->position_loop.output = wheel->speed_loop.output;
        break;
    }
}

/*
float Pose2WheelVelocity(const Pose_t* pose, const uint8_t wheelId)
{
    switch (wheelId)
    {
    case 0:
        return INVSQRT2 * (
            (sin(pose->theta) - cos(pose->theta)) * pose->vxw +
            (-sin(pose->theta) - cos(pose->theta)) * pose->vyw
        ) + WHEEL_TO_CENTER * pose->omega;
    case 1:
        return INVSQRT2 * (
            (sin(pose->theta) + cos(pose->theta)) * pose->vxw +
            (sin(pose->theta) - cos(pose->theta)) * pose->vyw
        ) + WHEEL_TO_CENTER * pose->omega;
    case 2:
        return INVSQRT2 * (
            (-sin(pose->theta) + cos(pose->theta)) * pose->vxw +
            (sin(pose->theta) + cos(pose->theta)) * pose->vyw
        ) + WHEEL_TO_CENTER * pose->omega;
    case 3:
        return INVSQRT2 * (
            (-sin(pose->theta) - cos(pose->theta)) * pose->vxw +
            (-sin(pose->theta) + cos(pose->theta)) * pose->vyw
        ) + WHEEL_TO_CENTER * pose->omega;
    default: return 0;
    }
}
*/

float wheelVelocity(const Velocity_t velocity, const uint8_t wheelId)
{
    switch (wheelId)
    {
    case 0:
        return INVSQRT2 * (-velocity.vx - velocity.vy) + WHEEL_TO_CENTER * velocity.omega;
    case 1:
        return INVSQRT2 * (velocity.vx - velocity.vy) + WHEEL_TO_CENTER * velocity.omega;
    case 2:
        return INVSQRT2 * (velocity.vx + velocity.vy) + WHEEL_TO_CENTER * velocity.omega;
    case 3:
        return INVSQRT2 * (-velocity.vx + velocity.vy) + WHEEL_TO_CENTER * velocity.omega;
    default:
        return 0;
    }
}

float wheelDisplacement(const Displacement_t displacement, const uint8_t wheelId)
{
    switch (wheelId)
    {
    case 0:
        return INVSQRT2 * (-displacement.x - displacement.y);
    case 1:
        return INVSQRT2 * (displacement.x - displacement.y);
    case 2:
        return INVSQRT2 * (displacement.x + displacement.y);
    case 3:
        return INVSQRT2 * (-displacement.x + displacement.y);
    default:
        return 0;
    }
}

/**
 * @brief 设置底盘移动速度
 * @attention 并不支持直行同时旋转到位，行走和旋转操作请分开执行
 * @param velocity
 * @param wheel
 */
void setVelocity(const Velocity_t velocity, Wheel_t* wheel)
{
    switchWheelState(wheel, SPEED_LOOP);
    /* 设置速度环 轮子速度(cm/s) / 轮子半径(cm) / 2π * 60(s) = 轮子转速(rpm) */
    __PID_SET_TARGET(&wheel->speed_loop, wheelVelocity(velocity, wheel->id) / WHEEL_RADIUS * RAD2ROUND * 60);
}

/**
 * @brief 设置底盘移动距离
 * @todo 在位置环执行过程中限制电机最大输出，避免加速太快打滑
 * @param displacement
 * @param wheel
 */
void setDisplacement(const Displacement_t displacement, Wheel_t* wheel)
{
    switchWheelState(wheel, POSITION_LOOP);
    // 归零计数，以当前位置为起点
    wheel->motor.real_round = 0;
    __PID_RESET(&wheel->position_loop);
    /* 设置位置环 轮子移动距离(cm) / 轮子半径(cm) / 2π = 轮子要转的圈数(round) */
    __PID_SET_TARGET(&wheel->position_loop, wheelDisplacement(displacement, wheel->id) / WHEEL_RADIUS * RAD2ROUND);
    printf("WheelId: %d, Displacement: %f\n", wheel->id, wheel->position_loop.target);
}

/**
 * @brief 设置底盘旋转角度
 * @param rotation
 * @param wheel
 */
void setRotation(const float rotation, Wheel_t* wheel)
{
    switchWheelState(wheel, POSITION_LOOP);
    // 归零计数，以当前位置为起点
    wheel->motor.real_round = 0;
    __PID_RESET(&wheel->position_loop);
    /* 设置位置环 底盘旋转角度(rad) * 轮子距中心距离(cm) / 轮子半径(cm) / 2π = 轮子要转的圈数(round) */
    __PID_SET_TARGET(&wheel->position_loop, rotation * WHEEL_TO_CENTER / WHEEL_RADIUS * RAD2ROUND);
}

// _Bool inPlace(Wheel_t* wheel) {}
