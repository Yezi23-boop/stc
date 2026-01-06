/****************************************************************************
 * @file    : lin_api_appl.c
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/4
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/


/*******************************************************************************
* Include files
*******************************************************************************/
#include "include.h"
#include <string.h>
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"
#include "global.h"

/*******************************************************************************
 * definitions
 *******************************************************************************/

/*******************************************************************************
 * global  variables
 *******************************************************************************/
l_u8 g_updateCtrlCmd = 0;
l_u8 g_confirmComTypeCmdUpdate = 0; // confirm com type update flag
ATCP_Status_t g_tATCPState;
ATCP_Ctrl_t g_tATCPCtrl;
ATCP_Version_t g_tATCPVersion = {
    .AS_b.CONTROLLER_TYPE = EAS_CONTROLLER_TYPE,
    .AS_b.CONTROLLER_TYPE1 = EAS_CONTROLLER_TYPE1,
    .AS_b.HW_VER = EAS_HW_VER,
    .AS_b.SW_VER = EAS_SW_VER,
    .AS_b.SOFTVER_YEAR = EAS_SOFTVER_YEAR,
    .AS_b.SOFTVER_MONTH = EAS_SOFTVER_MONTH,
    .AS_b.SOFTVER_DAY = EAS_SOFTVER_DAY,
    .AS_b.MCU_TYPE = EAS_MCU_TYPE,
};
ATCP_Fault_t g_tATCPFault;
ATCP_Debug_t g_tATCPDebug;

/*******************************************************************************
 * static  variables
 *******************************************************************************/
 

/*******************************************************************************
********************************************************************************
* implementation of global  functions
*
* Those functions  have to be completed/modified  according the requirements
* of the  application.
********************************************************************************
*******************************************************************************/

/************************************************************
 * @brief: This function is called from _RxTxStatusNotification() and sets the corresponding update flag,
           if the related signal has been changed.
 * @author: Novosns MCU Team
 * @param <l_u8> *frame_array, the pointer to l_tmp_data_array that contains the frame data
          (see section "static variables" in lin_api.c)
 * @return <None>
 ************************************************************/
void l_flg_set_update_flags(l_u8* frame_array)
{
#ifdef L_APPLICATION_EXAMPLE
    // Set update flags for the signals "LED" and "key_state"

    /* The frame PIDs (ID_13, ID_14, etc.) might be reconfigured meanwhile.
       Therefore the current l_PID_Table[] is used (and not the initial PIDs)
       for comparison with the PID received from the LIN master. */
    /* set flag only if the signal has changed */
    if (frame_array[0] == g_nodeFramePIDTable[NO_ATCP_CONTROL_FRAME]) {
        g_updateCtrlCmd = 1;
    }
    if (frame_array[0] == g_nodeFramePIDTable[NO_CONFIRM_COM_TYPE_FRAME]) {
        g_confirmComTypeCmdUpdate = 1;
    }

#endif
}

/************************************************************
 * @brief: fetch control command status
 * @return <None>
 ************************************************************/
l_bool l_get_ctrl_update_cmd(void)
{
    return g_updateCtrlCmd;
}

/************************************************************
 * @brief: fetch control command status
 * @return <None>
 ************************************************************/
void l_clr_ctrl_update_cmd(void)
{
    g_updateCtrlCmd = 0;
}

/************************************************************
 * @brief: confirm com type update flag
    ************************************************************/
l_bool l_get_confirm_com_type_cmd(void)
{
    return g_confirmComTypeCmdUpdate;
}

/************************************************************
 * @brief: clear confirm com type update flag
 * @return <None>
 ************************************************************/
void l_clr_confirm_com_type_cmd(void)
{
    g_confirmComTypeCmdUpdate = 0;
}



/************************************************************
 * @brief: transfer data buffer into ATCP ctrl strcut
 * @param <l_u8> *data_buffer
 * @param <ATCP_Ctrl_t> *ctrl
 * @return <None>
 ************************************************************/
void l_data_buffer_trans_atcp_ctrl_struct(const l_u8 *const data_buffer, ATCP_Ctrl_t *ctrl)
{
    *(l_u8 *)((l_u8 *)ctrl + 0) = data_buffer[0];
    *(l_u8 *)((l_u8 *)ctrl + 1) = data_buffer[1];
    *(l_u8 *)((l_u8 *)ctrl + 2) = data_buffer[2];
    *(l_u8 *)((l_u8 *)ctrl + 3) = data_buffer[3];
    *(l_u8 *)((l_u8 *)ctrl + 4) = data_buffer[4];
}

/************************************************************
 * @brief: transfer ATCP status struct into frame data buffer
 * @param <ATCP_Status_t> *fan_strcut
 * @param <l_u8> *data_buffer
 * @return <None>
 ************************************************************/
void l_atcp_struct_trans_data_buffer(const ATCP_Status_t *const exv_struct, l_u8 *data_buffer)
{
    __disable_irq();
    data_buffer[0] = *(l_u8 *)((l_u8 *)exv_struct + 0); 
    data_buffer[1] = *(l_u8 *)((l_u8 *)exv_struct + 1);
    data_buffer[2] = *(l_u8 *)((l_u8 *)exv_struct + 2);
    data_buffer[3] = *(l_u8 *)((l_u8 *)exv_struct + 3);
    data_buffer[4] = *(l_u8 *)((l_u8 *)exv_struct + 4);
    data_buffer[5] = *(l_u8 *)((l_u8 *)exv_struct + 5);
    data_buffer[6] = *(l_u8 *)((l_u8 *)exv_struct + 6);
    data_buffer[7] = *(l_u8 *)((l_u8 *)exv_struct + 7);
    __enable_irq();
}

void l_ver_trans_data_buffer(const ATCP_Version_t *const exv_struct, l_u8 *data_buffer)
{
    __disable_irq();
    data_buffer[0] = *(l_u8 *)((l_u8 *)exv_struct + 0); 
    data_buffer[1] = *(l_u8 *)((l_u8 *)exv_struct + 1);
    data_buffer[2] = *(l_u8 *)((l_u8 *)exv_struct + 2);
    data_buffer[3] = *(l_u8 *)((l_u8 *)exv_struct + 3);
    data_buffer[4] = *(l_u8 *)((l_u8 *)exv_struct + 4);
    data_buffer[5] = *(l_u8 *)((l_u8 *)exv_struct + 5);
    data_buffer[6] = *(l_u8 *)((l_u8 *)exv_struct + 6);
    data_buffer[7] = *(l_u8 *)((l_u8 *)exv_struct + 7);
    __enable_irq();
}

void l_fault_trans_data_buffer(const ATCP_Fault_t *const exv_struct, l_u8 *data_buffer)
{
    __disable_irq();
    data_buffer[0] = *(l_u8 *)((l_u8 *)exv_struct + 0); 
    data_buffer[1] = *(l_u8 *)((l_u8 *)exv_struct + 1);
    data_buffer[2] = *(l_u8 *)((l_u8 *)exv_struct + 2);
    data_buffer[3] = *(l_u8 *)((l_u8 *)exv_struct + 3);
    data_buffer[4] = *(l_u8 *)((l_u8 *)exv_struct + 4);
    data_buffer[5] = *(l_u8 *)((l_u8 *)exv_struct + 5);
    data_buffer[6] = *(l_u8 *)((l_u8 *)exv_struct + 6);
    data_buffer[7] = *(l_u8 *)((l_u8 *)exv_struct + 7);
    __enable_irq();
}

void l_data_buffer_trans_g_tATCPDebug_struct(const l_u8 *const data_buffer, ATCP_Debug_t *ctrl)
{
    *(l_u8 *)((l_u8 *)ctrl + 0) = data_buffer[0];
    *(l_u8 *)((l_u8 *)ctrl + 1) = data_buffer[1];
    *(l_u8 *)((l_u8 *)ctrl + 2) = data_buffer[2];
    *(l_u8 *)((l_u8 *)ctrl + 3) = data_buffer[3];
    *(l_u8 *)((l_u8 *)ctrl + 4) = data_buffer[4];
    *(l_u8 *)((l_u8 *)ctrl + 5) = data_buffer[5];
    *(l_u8 *)((l_u8 *)ctrl + 6) = data_buffer[6];
    *(l_u8 *)((l_u8 *)ctrl + 7) = data_buffer[7];
}



#ifdef L_EVENT_TRIGGERED_FRAME
/************************************************************
 * @brief: updates the signal "Counter" in the corresponding frame buffer
 * @author: Novosns MCU Team
 * @param <l_u8> v
 * @return <None>
 ************************************************************/
void l_u8_wr_Counter(l_u8 v)
{
    l_irqmask irqState = l_sys_irq_disable();  // disable all interrupts

    g_nodeConfigFrameTable[4].var[0] = ID_15;  // PID of unconditional frame that is carried in event triggered frame
    g_nodeConfigFrameTable[4].var[1] = v;      // update counter_value

    // update event triggered frame and set event flag
    g_nodeConfigFrameTable[5].var[0] = ID_15;
    g_nodeConfigFrameTable[5].var[1] = g_nodeConfigFrameTable[4].var[1];
    g_eventTable[5] = 1;
    l_sys_irq_restore(irqState);  // restore irqmask
}
#endif  // L_EVENT_TRIGGERED_FRAME


