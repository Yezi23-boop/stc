/****************************************************************************
 * @file    : lin_api.h
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#ifndef  LIN_API_H
#define  LIN_API_H
/*******************************************************************************
  type definitions
*******************************************************************************/
#include <stdint.h>
// LIN data types
typedef unsigned char   l_bool;
typedef unsigned char   l_u8;
typedef unsigned short  l_u16;

typedef unsigned char   l_ioctl_op;
typedef unsigned char   l_irqmask;
typedef unsigned char   l_ifc_handle;

// Types for Message Direction
typedef enum { 
    ID_DATARECEIVE, 
    ID_DATASEND,
    ID_DATASEND_EVENT                // data is only sent, if it has been changed
} MessageDirection;

// attributes of the message frame
typedef struct{
    MessageDirection direction;
    l_u8 length;
    l_u8 *var;
} ConfigFrame;

// Status of the LIN network
typedef struct {
    l_u8 networkState;
    l_u8 protocolState;
    l_u8 task;
} LINStatus;

/*******************************************************************************
* definitions
*******************************************************************************/

// possible values for "MyLinStatus"
#define L_INIT_STATE             0x00

// possible values for MyLinStatus.l_networkState
#define L_NT_SLEEP               0x01
#define L_NT_ACTIVE              0x02
#define L_NT_INIT                L_INIT_STATE

// possible values for MyLinStatus.l_protocolState
#define L_PROT_IDLE              0x01
#define L_PROT_IDENT             0x02
#define L_PROT_DATA              0x03  // waiting for DATA and CHKSUM
#define L_PROT_STOPBIT           0x04  
#define L_PROT_INIT              L_INIT_STATE

// possible values for MyLinStatus.l_task
#define L_TASK_IDLE              0x00
#define L_TASK_GET_DATA          0x01
#define L_TASK_SEND_DATA         0x02
#define L_TASK_WAKE              0x03

/* bit mask for l_ifc_status_MyLinIfc */
#define L_ERR_IN_RESPONSE_MASK   0x01
#define L_SUCC_TRANSFER_MASK     0x02
#define L_OVERRUN_MASK           0x04
#define L_GOTO_SLEEP_MASK        0x08

// definition of PIDs: 
#define ID_DEFAULT               0xFF  // reset value of the PID => PID is not valid
                                       // It is also possible to "unassign" a PID by a 
                                       // configuration frame setting the PID to 0x40

#define ID_00              0x80
#define ID_01              0xC1
#define ID_02              0x42
#define ID_03              0x03
#define ID_04              0xC4
#define ID_05              0x85
#define ID_06              0x06
#define ID_07              0x47
#define ID_08              0x08
#define ID_09              0x49
#define ID_10              0xCA
#define ID_11              0x8B
#define ID_12              0x4C
#define ID_13              0x0D
#define ID_14              0x8E
#define ID_15              0xCF
#define ID_16              0x50
#define ID_17              0x11
#define ID_18              0x92
#define ID_19              0xD3
#define ID_20              0x14
#define ID_21              0x55
#define ID_22              0xD6
#define ID_23              0x97
#define ID_24              0xD8
#define ID_25              0x99
#define ID_26              0x1A
#define ID_27              0x5B
#define ID_28              0x9C
#define ID_29              0xDD
#define ID_30              0x5E
#define ID_31              0x1F
#define ID_32              0x20
#define ID_33              0x61
#define ID_34              0xE2
#define ID_35              0xA3
#define ID_36              0x64
#define ID_37              0x25
#define ID_38              0xA6
#define ID_39              0xE7
#define ID_40              0xA8
#define ID_41              0xE9
#define ID_42              0x6A
#define ID_43              0x2B
#define ID_44              0xEC
#define ID_45              0xAD
#define ID_46              0x2E
#define ID_47              0x6F
#define ID_48              0xF0
#define ID_49              0xB1
#define ID_50              0x32
#define ID_51              0x73
#define ID_52              0xB4
#define ID_53              0xF5
#define ID_54              0x76
#define ID_55              0x37
#define ID_56              0x78
#define ID_57              0x39
#define ID_58              0xBA
#define ID_59              0xFB

#define ID_60              0x3C    // master request frame
#define ID_61              0x7D    // slave response frame
#define ID_62              0xFE    // reserved for future LIN extended format
#define ID_63              0xBF    // reserved for future LIN extended format

//======================================================================
// global variables                                  
//======================================================================


#define L_ERR_IN_RESPONSE_MASK  0x01            // possible values
#define L_SUCC_TRANSFER_MASK    0x02
#define L_OVERRUN_MASK          0x04
#define L_GOTO_SLEEP_MASK       0x08

#define LIN_OK                  0x00
#define LIN_BIT_ERROR           0x01
#define LIN_CHECKSUM_ERROR      0x02

#define BIT_MACRO(A, B) ((A >> B) & 0x01)

extern volatile LINStatus g_myLinStatus;

/*******************************************************************************
  prototypes of LIN API functions
*******************************************************************************/
// functions implemented according to LIN specification
l_bool l_sys_init(void);
l_irqmask l_sys_irq_disable(void);
void l_sys_irq_restore(l_irqmask previous);
void l_ifc_init_MyLinIfc(void);
l_bool l_ifc_connect_MyLinIfc(void);
l_bool l_ifc_disconnect_MyLinIfc(void);
l_u16 l_ifc_ioctl_MyLinIfc(l_ioctl_op op, void *pv);
void l_ifc_wake_up_MyLinIfc(void);
l_u16 l_ifc_read_status_MyLinIfc(void);
void l_ifc_rx_MyLinIfc(void);
void l_ifc_tx_MyLinIfc(void);
void l_ifc_aux_MyLinIfc(void);
//  function for LIN time measurement
void l_scheduler(void);
void ResetProtocolState(void);
void NotifyRxTxStatus(l_u8 status);
void SetLINResponseError(void);

#endif   /* LIN_API_H  */
