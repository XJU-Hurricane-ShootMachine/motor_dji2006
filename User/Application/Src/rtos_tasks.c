/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"
#include "uart2_calbackl.h"
#include "remote_ctrl.h"
#include "dji_angle.h"

#include "shoot_machine.h"

#include "queue.h"
#include "semphr.h "

/*控制摩擦轮任务与实行任务的消息队列*/
QueueHandle_t Queue_From_Fir; // 单个变量消息队列句

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task_message_handle; //轮询函数
void task_message(void *pvParameters);

static TaskHandle_t task_motor_handle; //2006电机控制函数
void task_motor(void *pvParameters);

dji_motor_handle_t dji_motor_1; //电机结构体
pid_t pid_pos;
pid_t pid_spd;

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 512, NULL, 2, &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);
    taskENTER_CRITICAL(); //进入临界区

    xTaskCreate(task_message, "task_message", 256, NULL, 2,
                &task_message_handle);
    xTaskCreate(task_motor, "task_motor", 256, NULL, 2, &task_motor_handle);
    Queue_From_Fir = xQueueCreate(1, sizeof(float));

    pid_init(&pid_pos, 16384, 5000, 30, 8000, POSITION_PID, 8.0f, 0.001f, 0.0f);
    pid_init(&pid_spd, 8192, 8192, 30, 8000, POSITION_PID, 6.0f, 0.001f, 0.2f);
    dji_motor_init(&dji_motor_1, DJI_M2006, CAN_Motor1_ID, can1_selected);
    vTaskDelete(start_task_handle);
    taskEXIT_CRITICAL();
}

void task_key(uint8_t key) {

    if (key == 1) {
        float tartget_angle = 90.0;
        xQueueSend(Queue_From_Fir, &tartget_angle, 0);
    }

    if (key == 2) {
        float tartget_angle = 180.0;
        xQueueSend(Queue_From_Fir, &tartget_angle, 0);
    }

    if (key == 3) {
        float tartget_angle = -90.0;
        xQueueSend(Queue_From_Fir, &tartget_angle, 0);
    }
}

/**
 * @brief Task_message: 指定串口轮询数据函数
 *
 * @param pvParameters Start parameters.
 */
void task_message(void *pvParameters) {
    UNUSED(pvParameters);
    message_add_polling_handle(&usart2_handle);
    message_register_recv_callback(MSG_REMOTE, remote_receive_callback);
    remote_register_key_callback(1, task_key);
    remote_register_key_callback(2, task_key);
    remote_register_key_callback(3, task_key);

    while (1) {
        message_polling_data();
        vTaskDelay(1);
    }
}

/**
  * @brief 电机控制任务
  * 
  * @param pvParameters 
  */
void task_motor(void *pvParameters) {
    UNUSED(pvParameters);
    static float speed_out = 0.0;
    static float angle_out = 0.0;
    float tartget_angle = 0.0;

    while (1) {

        xQueueReceive(Queue_From_Fir, &tartget_angle, 5);

        angle_out = pid_calc(&pid_pos, tartget_angle, dji_motor_1.rotor_degree);
        speed_out = pid_calc(&pid_spd, angle_out, dji_motor_1.speed_rpm);
        dji_motor_set_current(can1_selected, DJI_MOTOR_GROUP1,
                              (int16_t)speed_out, 0.0, 0.0, 0.0);
        vTaskDelay(5);
    }
}
