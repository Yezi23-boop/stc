#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "zf_common_headfile.h"

/**
 * @anchor fault_codes
 * @name Fault codes
 * The symbols below define the codes associated to the faults that the
 * Motor Control subsystem can raise.
 * @{ */
#define  MC_NO_ERROR     ((uint16_t)0x0000) /**< @brief No error. */
#define  MC_NO_FAULTS    ((uint16_t)0x0000) /**< @brief No error. */
#define  MC_DURATION     ((uint16_t)0x0001) /**< @brief Error: FOC rate to high. */
#define  MC_OVER_VOLT    ((uint16_t)0x0002) /**< @brief Error: Software over voltage. */
#define  MC_UNDER_VOLT   ((uint16_t)0x0004) /**< @brief Error: Software under voltage. */
#define  MC_OVER_TEMP    ((uint16_t)0x0008) /**< @brief Error: Software over temperature. */
#define  MC_START_UP     ((uint16_t)0x0010) /**< @brief Error: Startup failed. */
#define  MC_SPEED_FDBK   ((uint16_t)0x0020) /**< @brief Error: Speed feedback. */
#define  MC_OVER_CURR    ((uint16_t)0x0040) /**< @brief Error: Emergency input (Over current). */
#define  MC_SW_ERROR     ((uint16_t)0x0080) /**< @brief Software Error. */
#define  MC_DP_FAULT     ((uint16_t)0x0400) /**< @brief Error Driver protection fault. */

/**
  * @brief  State_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  ICLWAIT = 12,         /**< The system is waiting for ICL deactivation. Is not possible
                          *  to run the motor if ICL is active. While the ICL is active
                          *  the state is forced to #ICLWAIT; when ICL become inactive the
                          *  state is moved to #IDLE. */
  IDLE = 0,             /**< The state machine remains in this state as long as the
                          *  application is not controlling the motor. This state is exited
                          *  when the application sends a motor command or when a fault occurs. */
  ALIGNMENT = 2,        /**< The encoder alignment procedure that will properly align the
                          *  the encoder to a set mechanical angle is being executed. */
  CHARGE_BOOT_CAP = 16, /**< The gate driver boot capacitors are being charged. */
  OFFSET_CALIB = 17,    /**< The offset of motor currents and voltages measurement cirtcuitry
                          *  are being calibrated. */
  START = 4,            /**< The motor start-up is procedure is being executed. */
  SWITCH_OVER = 19,     /**< Transition between the open loop startup procedure and
                          *  closed loop operation */
  RUN = 6,              /**< The state machien remains in this state as long as the
                          *  application is running (controlling) the motor. This state
                          *  is exited when the application isues a stop command or when
                          *  a fault occurs. */
  STOP = 8,             /**< The stop motor procedure is being executed. */
  FAULT_NOW = 10,       /**< The state machine is moved from any state directly to this
                          *  state when a fault occurs. The next state can only be
                          *  #FAULT_OVER. */
  FAULT_OVER = 11,       /*!< The state machine transitions from #FAULT_NOW to this state
                          *   when there is no active fault condition anymore. It remains
                          * in this state until either a new fault condition occurs (in which
                          * case it goes back to the #FAULT_NOW state) or the application
                          * acknowledges the faults (in which case it goes to the #IDLE state).
                          */
  WAIT_STOP_MOTOR = 20  /**< Temporisation to make sure the motor is stopped. */

} MCI_State_t;

typedef struct
{
 MCI_State_t State;
 uint16_t CurrentFaults;
 uint16_t PastFaults;
} MCI_Handle_t;

// 电机配置结构
typedef struct {
    uint8_t pwm_channel_left;     // 左电机PWM通道
    gpio_pin_enum dir_pin_left;         // 左电机方向引脚
    uint8_t pwm_channel_right;    // 右电机PWM通道
    gpio_pin_enum dir_pin_right;        // 右电机方向引脚
    uint16_t pwm_frequency;       // PWM频率(Hz)

    uint16_t deadband_left;            // 死区PWM值
    uint16_t deadband_right;           // 死区PWM值
    uint16_t frequency_left;           // 左PWM频率
    uint16_t frequency_right;          // 右PWM频率
    
} MotorConfig_t;

// 电机ID定义
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_ALL = 2
} MotorID_e;

// 电机方向
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1,
    MOTOR_DIR_BRAKE = 2,
    MOTOR_DIR_COAST = 3
} MotorDirection;

// 编码器驱动接口
typedef struct {
    // 初始化
    void (*init)(encoder_index_enum encoder_n, gpio_pin_enum dir_pin, encoder_channel_enum lsb_pin);
    
    // 数据读取
    int16 (*read)(encoder_index_enum encoder_n);   
    void (*clear)(encoder_index_enum encoder_n);
    // float (*read_velocity)(EncoderID encoder);  // RPM
  
    // 校准
    //void (*calibrate)(EncoderID encoder, uint32_t calibration_time_ms);
    
    void* private_data;
} EncoderDriver;

// 电机驱动接口结构
typedef struct {
    // 初始化函数
    void (*init)(MotorConfig_t MotorConfig);
    // 控制函数
    void (*set_pwm)( int16_t lpwm_value, int16_t rpwm_value);
    // 状态获取
    // 高级功能
    // void (*enable_brake)(bool enable);
    // void (*set_deadband)(uint16_t deadband);  // 设置死区
    // // 私有数据
    // void* private_data;
} MotorDriver;

// 全局电机驱动实例
extern MotorDriver g_motor_driver;
extern EncoderDriver g_encoder_driver;
// 简化API（推荐使用）

void hal_motor_set_pwm(MotorID_e motor, int16_t pwm);
void hal_motor_set_speed(MotorID_e left_speed, int16_t right_speed);


void hal_motor_Init(void);
void motor_output(int32 lpwm, int32 rpwm);
void lost_lines();
 void MC_StateMachine(void);
extern int8 lost_spto;
#endif
