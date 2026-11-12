#include "can.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "CAN";

Motor_Data_t motor_data = {0};
SemaphoreHandle_t motor_data_mutex = NULL;

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

int16_t Can_Read_16(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    return (int16_t)value;
}

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;

    QueueHandle_t rx_queue = (QueueHandle_t)user_ctx;
    if (rx_queue == NULL)
    {
        return false;
    }

    uint8_t recv_buff[8] = {0};

    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK)
    {
        return false;
    }

    if (rx_frame.header.ide != false ||
        rx_frame.header.rtr != false ||
        rx_frame.header.fdf != false ||
        rx_frame.header.dlc > 8)
    {
        return false;
    }

    CAN_Frame_t packet = {0};
    packet.id = (uint16_t)rx_frame.header.id;
    packet.dlc = (uint8_t)rx_frame.header.dlc;

    for (uint8_t i = 0; i < packet.dlc; i++)
    {
        packet.data[i] = recv_buff[i];
    }

    xQueueSendFromISR(rx_queue, &packet, &higher_priority_task_woken);

    return higher_priority_task_woken == pdTRUE;
}

esp_err_t Can_Init(twai_node_handle_t *node_hdl, QueueHandle_t rx_queue)
{
    if (node_hdl == NULL || rx_queue == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *node_hdl = NULL;

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN_TX_PIN,
            .rx = CAN_RX_PIN,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing.bitrate = CAN_BITRATE,
        .tx_queue_depth = 5,
        .intr_priority = 3,
    };

    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&node_config, node_hdl), TAG, "Create node failed");

    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };

    ESP_RETURN_ON_ERROR(
        twai_node_register_event_callbacks(
            *node_hdl,
            &user_cbs,
            (void *)rx_queue),
        TAG, "Register callbacks failed");

    ESP_RETURN_ON_ERROR(
        twai_node_enable(*node_hdl),
        TAG, "Enable node failed");

    ESP_LOGI(TAG, "TWAI init ok: tx=%d rx=%d bitrate=%d",
             CAN_TX_PIN,
             CAN_RX_PIN,
             CAN_BITRATE);

    return ESP_OK;
}

void CAN_ParseFrame(const CAN_Frame_t *frame)
{
    if (frame == NULL || motor_data_mutex == NULL)
    {
        return;
    }

    xSemaphoreTake(motor_data_mutex, portMAX_DELAY);

    switch (frame->id)
    {
    case 0x001:
        if (frame->dlc == 2)
        {
            motor_data.motors_state = frame->data[0];
            motor_data.can_state = frame->data[1];
        }
        break;

    case 0x201:
        if (frame->dlc == 8)
        {
            motor_data.target_rpm = Can_Read_16(&frame->data[0]);
            motor_data.actual_rpm = Can_Read_16(&frame->data[2]);
            motor_data.iq_ref = Can_Read_16(&frame->data[4]) / 1000.0f;
            motor_data.iq = Can_Read_16(&frame->data[6]) / 1000.0f;
        }
        break;

    case 0x202:
        if (frame->dlc == 6)
        {
            motor_data.ia = Can_Read_16(&frame->data[0]) / 1000.0f;
            motor_data.ib = Can_Read_16(&frame->data[2]) / 1000.0f;
            motor_data.ic = Can_Read_16(&frame->data[4]) / 1000.0f;
        }
        break;

    case 0x203:
        if (frame->dlc == 6)
        {
            motor_data.vbus = Can_Read_16(&frame->data[0]) / 10.0f;
            motor_data.cbus = Can_Read_16(&frame->data[2]) / 1000.0f;
            motor_data.ntc = Can_Read_16(&frame->data[4]) / 100.0f;
        }
        break;

    case 0x204:
        if (frame->dlc == 6)
        {
            motor_data.duty_a = Can_Read_16(&frame->data[0]) / 100.0f;
            motor_data.duty_b = Can_Read_16(&frame->data[2]) / 100.0f;
            motor_data.duty_c = Can_Read_16(&frame->data[4]) / 100.0f;
        }
        break;

    case 0x205:
        if (frame->dlc == 8)
        {
            motor_data.vd = Can_Read_16(&frame->data[0]) / 1000.0f;
            motor_data.vq = Can_Read_16(&frame->data[2]) / 1000.0f;
            motor_data.valpha = Can_Read_16(&frame->data[4]) / 1000.0f;
            motor_data.vbeta = Can_Read_16(&frame->data[6]) / 1000.0f;
        }
        break;

    default:
        break;
    }

    xSemaphoreGive(motor_data_mutex);
}

void CAN_PrintFrame(const CAN_Frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    ESP_LOGI(TAG,
             "RX id=0x%03X dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned)frame->id,
             (unsigned)frame->dlc,
             frame->data[0],
             frame->data[1],
             frame->data[2],
             frame->data[3],
             frame->data[4],
             frame->data[5],
             frame->data[6],
             frame->data[7]);
}

void CAN_PrintMotorData(void)
{
    if (motor_data_mutex == NULL)
    {
        return;
    }

    if (xSemaphoreTake(motor_data_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return;
    }

    Motor_Data_t data = motor_data;

    xSemaphoreGive(motor_data_mutex);

    ESP_LOGI(TAG,
             "motor=%u can=%u rpm=%d/%d iq=%.2f/%.2f "
             "ia=%.2f ib=%.2f ic=%.2f "
             "vbus=%.2f cbus=%.3f ntc=%.2f "
             "duty=%.2f/%.2f/%.2f "
             "vd=%.2f vq=%.2f va=%.2f vb=%.2f",
             (unsigned)data.motors_state,
             (unsigned)data.can_state,
             (int)data.target_rpm,
             (int)data.actual_rpm,
             data.iq_ref,
             data.iq,
             data.ia,
             data.ib,
             data.ic,
             data.vbus,
             data.cbus,
             data.ntc,
             data.duty_a,
             data.duty_b,
             data.duty_c,
             data.vd,
             data.vq,
             data.valpha,
             data.vbeta);
}
