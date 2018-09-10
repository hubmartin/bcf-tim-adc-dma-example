#include <application.h>
#include <bcl.h>

#include <mic.h>

bc_led_t led;
bc_button_t button;

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_PRESS)
    {
        mic_start_measure();
    }

    // Logging in action
    bc_log_info("Button event handler - event: %i", event);
}

void application_init(void)
{
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_OFF);

    bc_scheduler_disable_sleep();
    bc_system_hsi16_enable();

    mic_init();

    mic_start_measure();
}

volatile float rms = 0.0f;

void application_task()
{
    bc_log_debug("%f", mic_get_rms());

    // RED means RECORDING https://www.youtube.com/channel/UChnxLLvzviaR5NeKOevB8iQ
    bc_led_set_mode(&led, (mic_measure_is_done()) ? BC_LED_MODE_OFF : BC_LED_MODE_ON);

    bc_scheduler_plan_current_relative(50);
}
