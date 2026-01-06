/************************************************************
 * @file: To be add
 * @author: Young Leon
 * @version: V1.0
 * @data: 2022/04/22
 * @brief: To be add
 * @note:
 * @Copyright (C) 2022 Novosense All rights reserved.
 ************************************************************/
#include "state_machine.h"
#include "foc_paras.h"


#if (ENABLE_HFI == 1)
const pFunc_t pStateFuncTable[16][8] = {
    /* Actual State ->    'Init'     'Fault'    'Ready'    'Calib'    'Align'     'HFI'      'Run'     'Reset' */
    /* e_fault         */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_fault_clear   */ {StateFault, StateInit, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_init          */ {StateInit, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_init_done     */ {StateReady, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_ready         */ {StateFault, StateFault, StateReady, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_app_on        */ {StateFault, StateFault, StateCalib, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_calib         */ {StateFault, StateFault, StateFault, StateCalib, StateFault, StateFault, StateFault, StateFault},
    /* e_calib_done    */ {StateFault, StateFault, StateFault, StateAlign, StateFault, StateFault, StateFault, StateFault},
    /* e_align         */ {StateFault, StateFault, StateFault, StateFault, StateAlign, StateFault, StateFault, StateFault},
    /* e_align_done    */ {StateFault, StateFault, StateFault, StateFault, StateHFI,   StateRun,  StateFault, StateFault}, // 修改转换逻辑
    /* e_hfi           */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateHFI,  StateFault, StateFault},
    /* e_hfi_done      */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateRun,  StateFault, StateFault},
    /* e_run           */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateRun,   StateFault},
    /* e_app_off       */ {StateFault, StateFault, StateReady, StateInit,  StateInit,  StateInit,  StateInit,  StateFault},
    /* e_reset         */ {StateFault, StateFault, StateReset, StateFault, StateFault, StateFault, StateFault, StateReset},
    /* e_reset_done    */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateInit}
};
#else
const pFunc_t pStateFuncTable[14][7] = {
    /* Actual State ->      'Init'       'Fault'      'Ready'    'Calib'     'Align'      'Run'		'Reset' */
    /* e_fault          */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_fault_clear    */ {StateFault, StateInit, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_init          	*/ {StateInit, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_init_done      */ {StateReady, StateFault, StateFault, StateFault, StateFault, StateFault, StateFault},
    /* e_ready          */ {StateFault, StateFault, StateReady, StateFault, StateFault, StateFault, StateFault},
    /* e_app_on         */ {StateFault, StateFault, StateCalib, StateFault, StateFault, StateFault, StateFault},
    /* e_calib          */ {StateFault, StateFault, StateFault, StateCalib, StateFault, StateFault, StateFault},
    /* e_calib_done     */ {StateFault, StateFault, StateFault, StateAlign, StateFault, StateFault, StateFault},
    /* e_align          */ {StateFault, StateFault, StateFault, StateFault, StateAlign, StateFault, StateFault},
    /* e_align_done     */ {StateFault, StateFault, StateFault, StateFault, StateRun, StateFault, StateFault},
    /* e_run            */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateRun, StateFault},
    /* e_app_off        */ {StateFault, StateFault, StateReady, StateInit, StateInit, StateInit, StateFault},
    /* e_reset          */ {StateFault, StateFault, StateReset, StateFault, StateFault, StateFault, StateReset},
    /* e_reset_done     */ {StateFault, StateFault, StateFault, StateFault, StateFault, StateFault, StateInit}};
#endif