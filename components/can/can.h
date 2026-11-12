#ifndef __CAN_H_
#define __CAN_H_

#include <stdint.h>
#include <stdbool.h>

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define CAN_TX_PIN 19
#define CAN_RX_PIN 25
#define CAN_BITRATE 500000

/*
    CAN ID   数据                                  DLC   缩放比例
    0x001    电机状态、CAN状态                    2     原始值
    0x201    目标转速、实际转速、Iq_ref、Iq       8     RPM直接传，电流×1000
    0x202    Ia、Ib、Ic                           6     ×1000
    0x203    Vbus、Ibus、NTC                      6     ×10、×1000、×100
    0x204    三相PWM占空比                        6     ×100
    0x205    vd、vq、valpha、vbeta                8     ×1000
*/

typedef struct
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_Frame_t;

typedef struct
{
    uint8_t motors_state;
    uint8_t can_state;

    int16_t target_rpm;
    int16_t actual_rpm;
    float iq_ref;
    float iq;

    float ia;
    float ib;
    float ic;

    float vbus;
    float cbus;
    float ntc;

    float duty_a;
    float duty_b;
    float duty_c;

    float vd;
    float vq;
    float valpha;
    float vbeta;
} Motor_Data_t;

extern Motor_Data_t motor_data;
extern SemaphoreHandle_t motor_data_mutex;

esp_err_t Can_Init(twai_node_handle_t *node_hdl, QueueHandle_t rx_queue);

int16_t Can_Read_16(const uint8_t *data);

void CAN_ParseFrame(const CAN_Frame_t *frame);

void CAN_PrintFrame(const CAN_Frame_t *frame);

void CAN_PrintMotorData(void);

#endif
