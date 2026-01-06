/****************************************************************************
 * @file    : lin_api_appl.h
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#ifndef  LIN_API_APPL_H
#define  LIN_API_APPL_H

/*******************************************************************************
  type definitions
*******************************************************************************/

/*******************************************************************************
  definitions
*******************************************************************************/

/*******************************************************************************
  global variables                                  
*******************************************************************************/

/*******************************************************************************
  prototypes of LIN API application specific functions
*******************************************************************************/
#include "stdint.h"
#pragma anon_unions
typedef struct {
    union {
        uint64_t u64StateLongWord1;
        struct {
            uint32_t NMeaRotCFM : 16;  //转速  
            uint32_t TempCFM : 16;
            uint32_t SWVer : 8;
            uint32_t DCVolCFM : 8;
            uint32_t DCCurCFM:16;
        } AS_b;
    };
} ATCP_Status_t; // ATCP_STATUS_FRAME structure
    
typedef struct {
    union {
        uint32_t u32CtrlWord1;
        struct {
            uint32_t NTagRotCFM : 16;  //转速
            uint32_t DCTagPWMCFM : 8;
            uint32_t CMDCFMAction: 8;  //转速开机指令 
            uint32_t CMDCFMDiretion: 8;  //正反转
        } CW_b;
    };
} ATCP_Ctrl_t;  // ATCP_CONTROL_FRAME structure

//软件硬件版本号 年月日
typedef struct {
    union {
        uint64_t u64StateLongWord1;
        struct {
            uint32_t CONTROLLER_TYPE : 8; // 控制器类型
            uint32_t CONTROLLER_TYPE1 : 8; // 控制器类型1
            uint32_t HW_VER : 8; // 硬件版本
            uint32_t SW_VER : 8; // 软件版本
            uint32_t SOFTVER_YEAR : 8; // 年
            uint32_t SOFTVER_MONTH : 8; // 月
            uint32_t SOFTVER_DAY : 8; // 日
            uint32_t MCU_TYPE : 8; // MCU型号
        } AS_b;
    };
} ATCP_Version_t;

//故障
typedef struct {
    union {
        uint64_t u64StateLongWord1;
        struct {
            uint32_t u32FaultMode: 32; // 故障模式标志位
            uint32_t reserved: 32; // 保留位
        } AS_b;
    };
} ATCP_Fault_t; // ATCP_FAULT_FRAME structure

// FA FA 02 FA FA FA FA FA
typedef struct {
    union {
        uint64_t u64StateLongWord1;
        struct {
            uint16_t Debugflag : 16;  //fa fa
            uint8_t DeviceType : 8; //02
            uint8_t Debugflag2 : 8; //fa
            uint32_t Debugflag3 : 32; //fa fa fa fa
        } AS_b;
    };
} ATCP_Debug_t; // ATCP_DEBUG_FRAME structure

extern ATCP_Status_t g_tATCPState;
extern ATCP_Ctrl_t g_tATCPCtrl;
extern ATCP_Version_t g_tATCPVersion;
extern ATCP_Fault_t g_tATCPFault;
extern ATCP_Debug_t g_tATCPDebug;
extern l_u8 g_confirmComTypeCmdUpdate;


void l_flg_set_update_flags(l_u8* frame_array);
l_bool l_get_ctrl_update_cmd(void);
void l_clr_ctrl_update_cmd(void);
void l_data_buffer_trans_atcp_ctrl_struct(const l_u8 *const data_buffer, ATCP_Ctrl_t *ctrl);
void l_atcp_struct_trans_data_buffer(const ATCP_Status_t *const exv_struct, l_u8 *data_buffer);
void l_ver_trans_data_buffer(const ATCP_Version_t *const exv_struct, l_u8 *data_buffer);
void l_fault_trans_data_buffer(const ATCP_Fault_t *const exv_struct, l_u8 *data_buffer);
extern l_bool l_get_confirm_com_type_cmd(void);
extern void l_clr_confirm_com_type_cmd(void);
extern void l_data_buffer_trans_g_tATCPDebug_struct(const l_u8 *const data_buffer, ATCP_Debug_t *ctrl);

#endif   /* LIN_API_APPL_H  */
