#include "main.hpp"

#define LQ_APP_NONE          0
#define LQ_APP_CAR_RUNTIME   1

#define LQ_DEMO_NONE         0
#define LQ_DEMO_GPIO_OUTPUT  1
#define LQ_DEMO_GPIO_INPUT   2
#define LQ_DEMO_PWM          3
#define LQ_DEMO_GTIM_PWM     4
#define LQ_DEMO_ENCODER_PWM  5
#define LQ_DEMO_MOTOR        6
#define LQ_DEMO_IMAGE        7
#define LQ_DEMO_IMAGE_MOTOR  8

#ifndef LQ_APP_TARGET
#define LQ_APP_TARGET LQ_APP_CAR_RUNTIME  // eg: LQ_APP_CAR_RUNTIME / LQ_APP_NONE
#endif

#ifndef LQ_DEMO_TARGET
#define LQ_DEMO_TARGET LQ_DEMO_NONE  // eg: LQ_DEMO_PWM / LQ_DEMO_MOTOR / LQ_DEMO_NONE      
#endif

int main()
{
#if LQ_APP_TARGET == LQ_APP_CAR_RUNTIME
    (void)CarRuntime_RunCameraLoop(true, 160, 120, 120, 1000, 20 * 1000, 20 * 1000);
#elif LQ_DEMO_TARGET == LQ_DEMO_GPIO_OUTPUT
    lq_log_info("Run demo: GPIO output");
    lq_gpio_output_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_GPIO_INPUT
    lq_log_info("Run demo: GPIO input");
    lq_gpio_input_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_PWM
    lq_log_info("Run demo: PWM");
    lq_pwm_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_GTIM_PWM
    lq_log_info("Run demo: GTIM PWM");
    lq_gtim_pwm_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_ENCODER_PWM
    lq_log_info("Run demo: encoder PWM");
    lq_encoder_pwm_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_MOTOR
    lq_log_info("Run demo: motor");
    lq_motor_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_IMAGE
    lq_log_info("Run demo: image");
    lq_image_demo();
#elif LQ_DEMO_TARGET == LQ_DEMO_IMAGE_MOTOR
    lq_log_info("Run demo: image+motor");
    lq_image_motor_demo();
#else
    lq_log_info("No app or demo selected. Set LQ_APP_TARGET for the formal runtime, or LQ_DEMO_TARGET for testing.");

    while (1)
    {
        usleep(100 * 1000);
    }
#endif

    return 0;
}
