#include "motor.hpp"

#include <memory>

namespace
{
constexpr atim_pwm_pin_t kLeftMotorPwmPin  = ATIM_PWM0_PIN81;
constexpr atim_pwm_pin_t kRightMotorPwmPin = ATIM_PWM1_PIN82;
constexpr gpio_pin_t kLeftMotorDirPin  = PIN_21;
constexpr gpio_pin_t kRightMotorDirPin = PIN_22;
constexpr ls_enc_pwm_pin_t kLeftEncoderPin = ENC_PWM0_PIN64;
constexpr ls_enc_pwm_pin_t kRightEncoderPin = ENC_PWM1_PIN65;
constexpr gpio_pin_t kLeftEncoderDirPin  = PIN_72;
constexpr gpio_pin_t kRightEncoderDirPin = PIN_73;
constexpr uint32_t kMotorPwmFreqHz = 10000;
constexpr int kDefaultInitDuty = 1000;
constexpr int kBaseTargetSpeed = 160;
// 正向（小车前进方向）时，方向 GPIO 应输出的电平
constexpr bool kLeftForwardDir  = true;   
constexpr bool kRightForwardDir = true;  

std::unique_ptr<ls_atim_pwm> left_motor_pwm;
std::unique_ptr<ls_atim_pwm> right_motor_pwm;
std::unique_ptr<ls_gpio> left_motor_dir;
std::unique_ptr<ls_gpio> right_motor_dir;
std::unique_ptr<ls_encoder_pwm> left_motor_encoder;
std::unique_ptr<ls_encoder_pwm> right_motor_encoder;

bool motor_initialized = false;

int clamp_pwm(int value)
{
    if (value < 0)
    {
        return 0;
    }
    if (value > ATIM_PWM_DUTY_MAX)
    {
        return ATIM_PWM_DUTY_MAX;
    }
    return value;
}

int clamp_pid_output(int value, int min_value, int max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

void ensure_motor_ready()
{
    if (!motor_initialized)
    {
        Motor_Init1(kDefaultInitDuty);
    }
}
}

int16_t encoder_Left = 0;
int16_t encoder_Right = 0;
int16_t Speed_Encoder_l = 0;
float Speed_P_l = 0;
float Speed_I_l = 0;
float Speed_D_l = 0;
int Speed_Erro_l = 0;
int Speed_Goal_l = 0;
int Speed_PID_OUT_l = 0;
int Speed_Lasterro_l = 0;
int Speed_Preverro_l = 0;

int16_t Speed_Encoder_r = 0;
float Speed_P_r = 0;
float Speed_I_r = 0;
float Speed_D_r = 0;
int Speed_Erro_r = 0;
int Speed_Goal_r = 0;
int Speed_PID_OUT_r = 0;
int Speed_Lasterro_r = 0;
int Speed_Preverro_r = 0;

int PWM_Max = 3200;
int PWM_Min = -3200;
int16_t Speed_Begin = 80;
int16_t Speed_Expect = 0;
float Diff_Speed_error = 0;
int16_t Diff_SpeedL_expect = 0;
int16_t Diff_SpeedR_expect = 0;
float Diff_Kp = 10.242f;
float Diff_Kd = 10.274f;
uint8_t stop_flag = 0;

float Encoder_Left1(void)
{
    ensure_motor_ready();
    encoder_Left = -static_cast<int16_t>(left_motor_encoder->encoder_get_count());
    return encoder_Left;
}

float Encoder_Right1(void)
{
    ensure_motor_ready();
    encoder_Right = -static_cast<int16_t>(right_motor_encoder->encoder_get_count());
    return encoder_Right;
}

void Encoder_Test1(void)
{
    ensure_motor_ready();
    (void)Encoder_Left1();
    (void)Encoder_Right1();
}

void Motor_Argument(void)
{
    Speed_Goal_l = kBaseTargetSpeed;
    Speed_Goal_r = kBaseTargetSpeed;

    Speed_P_l = 18;
    Speed_I_l = 1.65f;
    Speed_D_l = 1;

    Speed_P_r = 18;
    Speed_I_r = 1.65f;
    Speed_D_r = 1;
}

void Motor_Init1(int duty)
{
    const uint32_t init_duty = static_cast<uint32_t>(clamp_pwm(duty));

    left_motor_pwm  = std::make_unique<ls_atim_pwm>(kLeftMotorPwmPin,  kMotorPwmFreqHz, init_duty, ATIM_PWM_POL_INV);
    right_motor_pwm = std::make_unique<ls_atim_pwm>(kRightMotorPwmPin, kMotorPwmFreqHz, init_duty, ATIM_PWM_POL_INV);
    left_motor_dir  = std::make_unique<ls_gpio>(kLeftMotorDirPin,  GPIO_MODE_OUT);
    right_motor_dir = std::make_unique<ls_gpio>(kRightMotorDirPin, GPIO_MODE_OUT);
    left_motor_encoder  = std::make_unique<ls_encoder_pwm>(kLeftEncoderPin,  kLeftEncoderDirPin);
    right_motor_encoder = std::make_unique<ls_encoder_pwm>(kRightEncoderPin, kRightEncoderDirPin);

    left_motor_dir->gpio_level_set(kLeftForwardDir  ? GPIO_HIGH : GPIO_LOW);
    right_motor_dir->gpio_level_set(kRightForwardDir ? GPIO_HIGH : GPIO_LOW);
    left_motor_pwm->atim_pwm_set_duty(0);
    right_motor_pwm->atim_pwm_set_duty(0);

    encoder_Left = 0;
    encoder_Right = 0;
    Speed_Encoder_l = 0;
    Speed_Encoder_r = 0;
    Speed_Erro_l = 0;
    Speed_Erro_r = 0;
    Speed_PID_OUT_l = 0;
    Speed_PID_OUT_r = 0;
    Speed_Lasterro_l = 0;
    Speed_Lasterro_r = 0;
    Speed_Preverro_l = 0;
    Speed_Preverro_r = 0;
    Diff_SpeedL_expect = 0;
    Diff_SpeedR_expect = 0;
    motor_initialized = true;
}

void Left_Motor_Pwm1(int duty, bool dir)
{
    ensure_motor_ready();
    left_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
    left_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(clamp_pwm(duty)));
}

void Right_Motor_Pwm1(int duty, bool dir)
{
    ensure_motor_ready();
    right_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
    right_motor_pwm->atim_pwm_set_duty(static_cast<uint32_t>(clamp_pwm(duty)));
}

void Motor_Disable1(void)
{
    if (!motor_initialized)
    {
        return;
    }
    left_motor_pwm->atim_pwm_set_duty(0);
    right_motor_pwm->atim_pwm_set_duty(0);
    left_motor_pwm->atim_pwm_disable();
    right_motor_pwm->atim_pwm_disable();
    motor_initialized = false;
}

void Motor_Control(void)
{
    ensure_motor_ready();

    static uint32_t debug_log_divider = 0;

    encoder_Left = -static_cast<int16_t>(left_motor_encoder->encoder_get_count());
    encoder_Right = static_cast<int16_t>(right_motor_encoder->encoder_get_count());

    if (stop_flag == 1)
    {
        Speed_Goal_l = 0;
        Speed_Goal_r = 0;
    }
    else
    {
        Speed_Goal_l = kBaseTargetSpeed;
        Speed_Goal_r = kBaseTargetSpeed;

        if (top_point < 15)
        {
            Speed_Goal_l = kBaseTargetSpeed;
            Speed_Goal_r = kBaseTargetSpeed;
        }
        else
        {
            Speed_Goal_l = kBaseTargetSpeed;
            Speed_Goal_r = kBaseTargetSpeed;
        }
    }

    Motor_Diff_Pid1();
    Motor_PID_Left();
    Motor_PID_Right();

    if (++debug_log_divider >= 30)
    {
        debug_log_divider = 0;
        const int center_error = ImageStatus.Det_True - ImageStatus.MiddleLine;
        printf("err=%4d  pidL=%6d  pidR=%6d  spdL=%4d  spdR=%4d\n",
               center_error,
               Speed_PID_OUT_l,
               Speed_PID_OUT_r,
               encoder_Left,
               encoder_Right);
    }
}

void Motor_PID_Left(void)
{
    ensure_motor_ready();

    Speed_Encoder_l = encoder_Left;
    Speed_Erro_l = Diff_SpeedL_expect - Speed_Encoder_l;

    Speed_PID_OUT_l += static_cast<int>(Speed_P_l * (Speed_Erro_l - Speed_Lasterro_l) +
                                        Speed_I_l * Speed_Erro_l +
                                        Speed_D_l * (Speed_Erro_l - 2 * Speed_Lasterro_l + Speed_Preverro_l));

    Speed_PID_OUT_l = clamp_pid_output(Speed_PID_OUT_l, PWM_Min, PWM_Max);

    Speed_Preverro_l = Speed_Lasterro_l;
    Speed_Lasterro_l = Speed_Erro_l;

    if (Speed_PID_OUT_l >= 0)
    {
        Left_Motor_Pwm1(Speed_PID_OUT_l, kLeftForwardDir);
    }
    else
    {
        Left_Motor_Pwm1(-Speed_PID_OUT_l, !kLeftForwardDir);
    }
}

void Motor_PID_Right(void)
{
    ensure_motor_ready();

    Speed_Encoder_r = encoder_Right;
    Speed_Erro_r = Diff_SpeedR_expect - Speed_Encoder_r;

    Speed_PID_OUT_r += static_cast<int>(Speed_P_r * (Speed_Erro_r - Speed_Lasterro_r) +
                                        Speed_I_r * Speed_Erro_r +
                                        Speed_D_r * (Speed_Erro_r - 2 * Speed_Lasterro_r + Speed_Preverro_r));

    Speed_PID_OUT_r = clamp_pid_output(Speed_PID_OUT_r, PWM_Min, PWM_Max);

    Speed_Preverro_r = Speed_Lasterro_r;
    Speed_Lasterro_r = Speed_Erro_r;

    if (Speed_PID_OUT_r >= 0)
    {
        Right_Motor_Pwm1(Speed_PID_OUT_r, kRightForwardDir);
    }
    else
    {
        Right_Motor_Pwm1(-Speed_PID_OUT_r, !kRightForwardDir);
    }
}

void Motor_Diff_Pid1(void)
{
    static float last_turn_error = 0;

    float turn_error = ImageStatus.Det_True - (float)ImageStatus.MiddleLine;
    if (turn_error > -2.0f && turn_error < 2.0f)
    {
        turn_error = 0;
    }

    const float abs_turn_error = my_abs(turn_error);
    float current_Kp = Diff_Kp;
    float current_Kd = Diff_Kd;
    if (abs_turn_error > 0.0f && abs_turn_error < 4.0f)
    {
        current_Kp = Diff_Kp * 0.9f;
    }
    else if (abs_turn_error < 18.0f)
    {
        current_Kp = Diff_Kp * 1.25f;
        current_Kd = Diff_Kd * 1.15f;
    }

    float turn_output = current_Kp * turn_error + current_Kd * (turn_error - last_turn_error);
    last_turn_error = turn_error;

    if (turn_output > 500.0f)
    {
        turn_output = 500.0f;
    }
    if (turn_output < -500.0f)
    {
        turn_output = -500.0f;
    }

    int current_base_speed = Speed_Goal_l - static_cast<int>(my_abs(turn_error) * 3.5f);
    if (current_base_speed < 80)
    {
        current_base_speed = 80;
    }

    Diff_SpeedL_expect = static_cast<int16_t>(current_base_speed + static_cast<int>(turn_output));
    Diff_SpeedR_expect = static_cast<int16_t>(current_base_speed - static_cast<int>(turn_output));

    if (Diff_SpeedL_expect < 0)
    {
        Diff_SpeedL_expect = 0;
    }
    if (Diff_SpeedR_expect < 0)
    {
        Diff_SpeedR_expect = 0;
    }
    if (Diff_SpeedL_expect > 1500)
    {
        Diff_SpeedL_expect = 1500;
    }
    if (Diff_SpeedR_expect > 1500)
    {
        Diff_SpeedR_expect = 1500;
    }
}
