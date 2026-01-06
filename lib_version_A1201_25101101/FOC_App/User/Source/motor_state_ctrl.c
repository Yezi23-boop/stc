/****************************************************************************
 * @file    : motor_state_ctrl.c
 * @author  : Aiwin MCU Team
 * @version : V1.0
 * @Date    : 2024/04/11
 * @brief   : FAN MOTOR STATE CONTROL
 * @note
 * Copyright (C) 2024 Aiwin All rights reserved.
 ****************************************************************************/
#include "Module_Define.h"
#include "testFun.h"
#include "global.h"

//停机转速，当低于该阈值时直接刹车停机不再降速
#define FOC_STOP_SPEED (FRAC16(600/MAX_RPM))
#define FOC_MIN_SPEED (FRAC16(MIN_RPM/MAX_RPM)) // 限制最小转速，防止运行在强托环节或是由闭环状态切回强托导致不稳定


extern void UpdateLinATCPStatus(void);
static Frac16_t ParseDirectSpeed(uint16_t u16DirSpeed);
extern Frac16_t ParsePWMSpeed(uint16_t u8Pwm);

extern uint8_t PWMIO_GetMotorState(void);
extern void PWMOUT_Set(uint32_t freq, uint32_t duty);
extern void PWMOUT_SetEnable(uint8_t enable);

//当前的通讯方式
uint8_t g_u8ComType;
uint16_t g_u16ComTypeConfirmCnt = 1; //通讯方式确认计数

//从指令中获取的转速指令
Frac16_t f16CmdTagSpeed = 0;

extern uint32_t g_u32DutyCycleConfirm;
extern uint8_t g_u8pwmOffineFlag;


// static Frac16_t f16TagSpeed = 0;
Frac16_t f16PhaseCurrentMaxFilter[3]; //相电流幅值滤波
Frac16_t f16PhaseCurrentMax[3]; //相电流幅值
//相电流有效值
Frac16_t f16PhaseCurrentRms[3] = {0, 0, 0};
//相电流幅值滤波器
GCF_Filter1_LPF16_t tPhaseCurrentMaxFilter[3] = {
    {0, 0, 0, 0, PHASE_CURRENT_MAX_FILTER_NPOINT, 0},
    {0, 0, 0, 0, PHASE_CURRENT_MAX_FILTER_NPOINT, 0},
    {0, 0, 0, 0, PHASE_CURRENT_MAX_FILTER_NPOINT, 0}
};

void HandleLinCmd(void)
{

#if ((COM_TYPE & 0x02 )== COM_TYPE_LIN)
    if (g_u8ComType != COM_TYPE_LIN) {
        return;
    }
    if (l_get_ctrl_update_cmd()) {
        l_data_buffer_trans_atcp_ctrl_struct(g_nodeConfigFrameTable[NO_ATCP_CONTROL_FRAME].var, &g_tATCPCtrl);
        if (g_tATCPCtrl.CW_b.CMDCFMAction== 0) { //ATCP Stop command
            f16CmdTagSpeed = 0;
        } else if (g_tATCPCtrl.CW_b.CMDCFMAction== 0x40) {  // target speed value command
            if (0x00 == g_tATCPCtrl.CW_b.CMDCFMDiretion) {
                f16CmdTagSpeed = ParseDirectSpeed(g_tATCPCtrl.CW_b.NTagRotCFM);
            } else if (0x01 == g_tATCPCtrl.CW_b.CMDCFMDiretion) {
                f16CmdTagSpeed = -ParseDirectSpeed(g_tATCPCtrl.CW_b.NTagRotCFM);
            }   
        } else if (g_tATCPCtrl.CW_b.CMDCFMAction== 0x80) {  // target PWM DC value command
          //  f16CmdTagSpeed = ParsePWMSpeed(g_tATCPCtrl.CW_b.DCTagPWMCFM);
        }
        // tDrvFoc.tPospeControl.f16wRotElReq = f16CmdTagSpeed;
        // if (tDrvFoc.tAppState.tStatus == FAULT) {
        //     tDrvFoc.tAppState.tEvent = E_FAULT_CLEAR;
        // }

        l_clr_ctrl_update_cmd();
    }
#endif

}

void HandlePwmCmd(void)
{

    static Frac16_t lastF16CmdTagSpeed = 0;
#if ((COM_TYPE &0x01) == COM_TYPE_PWM)
    if (g_u8ComType != COM_TYPE_PWM) {
        return;
    }
    f16CmdTagSpeed = ParsePWMSpeed(g_u32DutyCycleConfirm);
#if (PWM_RUN_FULL_SPEED_ON_DISCONNECT == 1) 
    if ((lastF16CmdTagSpeed >= FRAC16(PWM_MIN_RPM / MAX_RPM)) && (g_u8pwmOffineFlag == 1)) f16CmdTagSpeed = FRAC16(PWM_MAX_RPM / MAX_RPM);
#endif
    lastF16CmdTagSpeed = f16CmdTagSpeed;
#endif
}


void getMotorCmdSpeed(void)
{
   if (tDrvFoc.bInFaultDelay) {
        // 在故障重启延迟期间，不处理新的速度指令，或者将目标速度强制为0
        f16CmdTagSpeed = 0; 
        tDrvFoc.tPospeControl.f16wRotElReq = 0;
        // 确保电机保持停机状态，这通常由 StateFault() 处理
        if (tDrvFoc.tAppState.tStatus != FAULT) {
             tDrvFoc.tAppState.tEvent = E_FAULT; // 强制进入故障状态
        }
        return; // 提前返回，不执行后续的速度指令处理和降额
    }

    HandleLinCmd();

    HandlePwmCmd();

}

void MotorStateCtrl(void)
{
    Frac16_t cmdSpeed_  = 0;
    uint16_t n_max_for_conversion = tFocParas.N_MAX; 
    if (n_max_for_conversion == 0) {
        n_max_for_conversion = 3000; // Fallback if N_MAX is not set, adjust as needed
    }
    //f16CmdTagSpeed = ParseDirectSpeed(2100);
    //限制转速，防止运行在强托环节或是由闭环状态切回强托导致不稳定
    // NOTE:电机刚开始调试时，需要运行在强托环节，需要将该语句注释
#if (IS_RELEASE_VERSION != 0)
    if (AbsF16(f16CmdTagSpeed) < FOC_MIN_SPEED) {
        f16CmdTagSpeed = 0;
    }
#endif

    cmdSpeed_ = Derating_Apply(
        &g_derating_controller,
        f16CmdTagSpeed, // Current target speed
        tDrvFoc.i16IntMOSTemp,                 // Current temperature in Celsius
        tDrvFoc.f16DcBusFilt,               // Current filtered DC bus voltage (Frac16_t)
        n_max_for_conversion                // N_MAX for RPM <-> Frac16_t conversion
    );


#if (TEST_MODE == 1)
    MotorStartupTestTask();
   
#else

    if (tFaultMode.u32FaultMode !=0) {
        tDrvFoc.tPospeControl.f16wRotElReq = 0;
        tDrvFoc.tAppState.tEvent = E_FAULT;

        tDrvFoc.bInFaultDelay = true;
        tDrvFoc.u32FaultRestartDelayTimer = FAULT_RESTART_DELAY_MS; 
        PWMOUT_SetEnable(1); // 使能PWM输出
        PWMOUT_Set(1500,0); // PWM输出低电平


    } else {
        // 故障清除后，停机延迟结束后再开机
        if (!tDrvFoc.bInFaultDelay){
            PWMOUT_SetEnable(0); // 改为PWM输入状态
            
            // 获取当前速度绝对值
            Frac16_t f16AbsSpeed = (tDrvFoc.tPospeControl.f16wRotE1 > 0) ? tDrvFoc.tPospeControl.f16wRotE1 : -tDrvFoc.tPospeControl.f16wRotE1;
            // 判断电机是否处于运行状态
            bool bIsRunning = (f16AbsSpeed > FRAC16(100.0 / MAX_RPM)); // 100 RPM 作为运行阈值
            bool bDirectionChange = false;

            // 如果电机正在运行且有速度指令，检查是否反向
            if (bIsRunning && (cmdSpeed_ != 0)) {
                if ((tDrvFoc.tPospeControl.f16wRotE1 > 0 && cmdSpeed_ < 0) ||
                    (tDrvFoc.tPospeControl.f16wRotE1 < 0 && cmdSpeed_ > 0)) {
                    bDirectionChange = true;
                }
            }

            if (bDirectionChange) {
                // 如果需要反转，强制目标速度为0，先让电机停下来
                // 此时不清除故障，等待速度降到 FOC_STOP_SPEED 以下触发 E_FAULT
                tDrvFoc.tPospeControl.f16wRotElReq = 0;
            } else {
                // 正常逻辑：同向指令、电机已停机或指令为0
                // 如果在反转减速过程中收到同向指令，bDirectionChange为false，会进入这里直接更新速度
                if (cmdSpeed_ != 0){
                    tDrvFoc.tPospeControl.f16wRotElReq  = cmdSpeed_;

                    if (tDrvFoc.tAppState.tStatus == FAULT) {
                        tDrvFoc.tAppState.tEvent = E_FAULT_CLEAR;
                    }
                } else {
                    tDrvFoc.tPospeControl.f16wRotElReq = 0;
                }
            }
        }
    }

    if (tDrvFoc.tPospeControl.f16wRotElReq == 0) {
        // 使用绝对值判断停机，确保反转时也能正确触发FAULT状态
        Frac16_t f16AbsSpeed = (tDrvFoc.tPospeControl.f16wRotE1 > 0) ? tDrvFoc.tPospeControl.f16wRotE1 : -tDrvFoc.tPospeControl.f16wRotE1;
        if (f16AbsSpeed < FOC_STOP_SPEED) {
            tDrvFoc.tAppState.tEvent = E_FAULT;
        }
    }
#endif

}


/************************************************************
 * @brief: Parse direc speed into normalization speed
 * @return <None>
 ************************************************************/
static Frac16_t ParseDirectSpeed(uint16_t u16DirSpeed)
{   
    if (u16DirSpeed > MAX_RPM) u16DirSpeed = MAX_RPM;
    
    return u16DirSpeed * 32767 / (int16_t)tFocParas.N_MAX;
}

//传入U电流采样值 计算幅值最大值
void GetUMaxCurrent(int16_t i16Current)
{
    if (i16Current >  f16PhaseCurrentMax[0]) {
        f16PhaseCurrentMax[0] = i16Current;
    } 
}

void GetVMaxCurrent(int16_t i16Current)
{
    if (i16Current >  f16PhaseCurrentMax[1]) {
        f16PhaseCurrentMax[1] = i16Current;
    } 
}


void GetWMaxCurrent(int16_t i16Current)
{
    if (i16Current >  f16PhaseCurrentMax[2]) {
        f16PhaseCurrentMax[2] = i16Current;
    } 
}

//幅值计算成有效值
void calPhaseCurrentMaxFilter(void)
{
    f16PhaseCurrentRms[0] =  (f16PhaseCurrentMaxFilter[0] * FRAC16(0.707)) >> 15;
    f16PhaseCurrentRms[1] =  (f16PhaseCurrentMaxFilter[1] * FRAC16(0.707)) >> 15;
    f16PhaseCurrentRms[2] =  (f16PhaseCurrentMaxFilter[2] * FRAC16(0.707)) >> 15;

}



