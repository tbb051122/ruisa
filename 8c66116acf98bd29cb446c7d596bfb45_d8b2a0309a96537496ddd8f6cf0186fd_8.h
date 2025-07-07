/**
 * @file modbus_drive_control.h
 * @brief 瑞萨RA6M5 串口6 Modbus RTU电机控制头文件
 */

#ifndef MODBUS_DRIVE_CONTROL_H
#define MODBUS_DRIVE_CONTROL_H

#include "hal_data.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 驱动器配置 */
#define DRIVE_ADDRESS   0x01    // 驱动器地址

/* 寄存器地址 */
#define REG_HEARTBEAT   0x1770  // 心跳寄存器(6000)
#define REG_CTRL_MODE   0x1771  // 控制模式寄存器(6001)
#define REG_DUTY_CYCLE  0x1775  // 占空比寄存器(6005)

/* 控制模式 */
#define MODE_DUTY_CYCLE 0x0002  // 占空比控制模式
#define MODE_STOP       0xFFFF  // 停止模式

/* 占空比范围 */
#define DUTY_CYCLE_MAX   1000   // 最大正向占空比
#define DUTY_CYCLE_MIN  -1000   // 最大反向占空比
#define DUTY_CYCLE_STOP     0   // 停止占空比

/* 函数声明 */

/**
 * @brief 串口6回调函数
 */
void motor_uart_callback(uart_callback_args_t *p_args);

/**
 * @brief 初始化电机控制系统
 * @return fsp_err_t 初始化结果
 */
fsp_err_t init_duty_cycle_control(void);

/**
 * @brief 发送心跳包
 * @param heartbeat_value 心跳值
 * @return fsp_err_t 发送结果
 */
fsp_err_t send_heartbeat(uint16_t heartbeat_value);

/**
 * @brief 设置占空比控制
 * @param duty_cycle 占空比值(-1000~1000)
 * @return fsp_err_t 设置结果
 */
fsp_err_t set_duty_cycle_control(int16_t duty_cycle);

/**
 * @brief 心跳任务(需要每500ms调用)
 */
void heartbeat_task(void);

/**
 * @brief 停止电机
 * @return fsp_err_t 设置结果
 */
fsp_err_t stop_motor(void);

/**
 * @brief 紧急停止电机
 * @return fsp_err_t 设置结果
 */
fsp_err_t emergency_stop_motor(void);

/**
 * @brief 电机正转
 * @param speed_percent 速度百分比(1-100)
 * @return fsp_err_t 设置结果
 */
fsp_err_t motor_forward(uint8_t speed_percent);

/**
 * @brief 电机反转
 * @param speed_percent 速度百分比(1-100)
 * @return fsp_err_t 设置结果
 */
fsp_err_t motor_reverse(uint8_t speed_percent);

/**
 * @brief 获取心跳计数器值
 * @return uint16_t 心跳计数器值
 */
uint16_t get_heartbeat_counter(void);

/**
 * @brief 重置心跳计数器
 */
void reset_heartbeat_counter(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_DRIVE_CONTROL_H */
