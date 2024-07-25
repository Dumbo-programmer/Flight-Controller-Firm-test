#include "stm32f4xx_hal.h"      // STM32F4 HAL library
#include "mpu6050.h"            // IMU sensor library
#include "pid.h"                // PID control library

// Define constants
#define MOTOR1_PWM_CHANNEL TIM_CHANNEL_1
#define MOTOR2_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR3_PWM_CHANNEL TIM_CHANNEL_3
#define MOTOR4_PWM_CHANNEL TIM_CHANNEL_4

#define IMU_UPDATE_INTERVAL_MS 10  // 100 Hz update rate
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_NEUTRAL ((PWM_MAX + PWM_MIN) / 2)
// PID controllers for roll, pitch, and yaw
PID_Controller roll_pid, pitch_pid, yaw_pid;

// Motor control variables
uint16_t motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;

// Sensor data
float roll, pitch, yaw;
float roll_rate, pitch_rate, yaw_rate;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
void init_system(void);
void read_sensors(void);
void compute_pid(void);
void update_motors(void);
void init_system(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();  // Timer for PWM output
    MX_I2C1_Init();  // I2C for IMU
    MX_UART4_Init(); // UART for debugging

    // Initialize the IMU
    mpu6050_init();
    
    // Initialize PID controllers
    pid_init(&roll_pid, 1.0, 0.0, 0.0);  // Example PID constants
    pid_init(&pitch_pid, 1.0, 0.0, 0.0);
    pid_init(&yaw_pid, 1.0, 0.0, 0.0);
}
void read_sensors(void) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw data to angles (placeholder functions)
    roll_rate = (float)gx / 65.5;
    pitch_rate = (float)gy / 65.5;
    yaw_rate = (float)gz / 65.5;
    
    // Convert gyro data to angle (using simple integration)
    roll += roll_rate * (IMU_UPDATE_INTERVAL_MS / 1000.0);
    pitch += pitch_rate * (IMU_UPDATE_INTERVAL_MS / 1000.0);
    yaw += yaw_rate * (IMU_UPDATE_INTERVAL_MS / 1000.0);
}
void compute_pid(void) {
    // Example desired values (these should be from control inputs or mission parameters)
    float desired_roll = 0.0;
    float desired_pitch = 0.0;
    float desired_yaw = 0.0;
    
    // Compute PID outputs
    float roll_output = pid_compute(&roll_pid, desired_roll, roll);
    float pitch_output = pid_compute(&pitch_pid, desired_pitch, pitch);
    float yaw_output = pid_compute(&yaw_pid, desired_yaw, yaw);

    // Convert PID outputs to motor PWM signals
    motor1_pwm = PWM_NEUTRAL + roll_output + pitch_output + yaw_output;
    motor2_pwm = PWM_NEUTRAL - roll_output + pitch_output - yaw_output;
    motor3_pwm = PWM_NEUTRAL - roll_output - pitch_output + yaw_output;
    motor4_pwm = PWM_NEUTRAL + roll_output - pitch_output - yaw_output;

    // Constrain PWM values
    motor1_pwm = constrain_pwm(motor1_pwm);
    motor2_pwm = constrain_pwm(motor2_pwm);
    motor3_pwm = constrain_pwm(motor3_pwm);
    motor4_pwm = constrain_pwm(motor4_pwm);
}

uint16_t constrain_pwm(uint16_t pwm) {
    if (pwm < PWM_MIN) return PWM_MIN;
    if (pwm > PWM_MAX) return PWM_MAX;
    return pwm;
}
void update_motors(void) {
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR1_PWM_CHANNEL, motor1_pwm);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR2_PWM_CHANNEL, motor2_pwm);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR3_PWM_CHANNEL, motor3_pwm);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR4_PWM_CHANNEL, motor4_pwm);
}
int main(void) {
    init_system();
    
    while (1) {
        read_sensors();
        compute_pid();
        update_motors();
        
        // Delay for sensor update interval
        HAL_Delay(IMU_UPDATE_INTERVAL_MS);
    }
}
