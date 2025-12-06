#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "zf_common_headfile.h"

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
/**
 * @brief list of direct (unbffered) commands
 */
typedef enum
{
  MCI_NO_COMMAND = 0,   /**< No Command --- Set when going to IDLE */
  MCI_START,            /**< Start controling the Motor */
  MCI_ACK_FAULTS,       /**< Acknowledge Motor Control subsystem faults */
  MCI_MEASURE_OFFSETS,  /**< Start the ADCs Offset measurements procedure */
  /* Shouldn't we remove this command ? */
  MCI_ALIGN_ENCODER,    /**< Start the Encoder alignment procedure */
  MCI_STOP              /**< Stop the Motor and the control */
} MCI_DirectCommands_t;

typedef struct
{

 MCI_DirectCommands_t DirectCommand;
 MCI_State_t State;
 uint16_t CurrentFaults;
 uint16_t PastFaults;
} MCI_Handle_t;
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

void motor_Init();
void motor_output(int32 lpwm, int32 rpwm);
void lost_lines();
extern int8 lost_spto;
#endif
