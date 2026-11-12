#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"

#include "can.h"
#include "gpio.h"

static const char *TAG = "APP";

static QueueHandle_t s_can_queue = NULL;
static twai_node_handle_t s_can_node = NULL;

static void CAN_RxTask(void *argument)
{
    (void)argument;

    CAN_Frame_t frame;
    TickType_t last_print = xTaskGetTickCount();

    ESP_LOGI(TAG, "CAN_RxTask started");

    while (1)
    {
        if (xQueueReceive(s_can_queue, &frame, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            CAN_ParseFrame(&frame);
            // CAN_PrintFrame(&frame);
        }

        TickType_t now = xTaskGetTickCount();

        if (now - last_print >= pdMS_TO_TICKS(1000))
        {
            last_print = now;
            CAN_PrintMotorData();
        }
    }
}

void app_main(void)
{
    Led_Init();

    s_can_queue = xQueueCreate(64, sizeof(CAN_Frame_t));
    motor_data_mutex = xSemaphoreCreateMutex();

    if (s_can_queue == NULL || motor_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Create queue/mutex failed");
        return;
    }

    esp_err_t ret = Can_Init(&s_can_node, s_can_queue);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Can_Init failed: %s", esp_err_to_name(ret));
        return;
    }

    BaseType_t task_ret = xTaskCreate(
        CAN_RxTask,
        "CAN_RxTask",
        4096,
        NULL,
        10,
        NULL);

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Create CAN_RxTask failed");
        return;
    }

    ESP_LOGI(TAG, "CAN receiver started");
}
