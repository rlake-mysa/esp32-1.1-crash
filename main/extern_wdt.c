#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <driver/gpio.h>
#include <driver/timer.h>
#include <soc/timer_group_struct.h>

static QueueHandle_t wdt_queue;

static bool IRAM_ATTR
tg1_isr(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t   x;
    xQueueReceiveFromISR(wdt_queue, &x, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken;
}

void
extern_wdt_init(void)
{
    wdt_queue = xQueueCreate(1, sizeof(uint32_t));

    timer_config_t config = { .alarm_en    = TIMER_ALARM_EN,
                              .counter_en  = TIMER_PAUSE,
                              .intr_type   = TIMER_INTR_LEVEL,
                              .counter_dir = TIMER_COUNT_UP,
                              .auto_reload = TIMER_AUTORELOAD_EN,
                              .divider     = 16 };

    const uint64_t alarm_val = 200.0 / 1000.0 * TIMER_BASE_CLK / config.divider;

    timer_init(TIMER_GROUP_1, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0ULL);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, alarm_val);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, tg1_isr, (void*)TIMER_0, 0);
    timer_start(TIMER_GROUP_1, TIMER_0);
}