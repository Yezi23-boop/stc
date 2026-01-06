/****************************************************************************
 * @file    : lin_config.h
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/11
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2022 Novosense All rights reserved.
 ****************************************************************************/
#ifndef  LIN_CONFIG_H
#define  LIN_CONFIG_H

/*******************************************************************************
* Include files
*******************************************************************************/
#include "lin_api.h"

/*******************************************************************************
* definitions
*******************************************************************************/
#define INITIAL_NAD            0x01
#ifdef L_LIN_2_2
  #define FUNCTIONAL_NAD       0x7E
#endif  
#define WILDCARD_NAD           0x7F
#define WILDCARD_SUPPLIER_ID   0x7FFF
#define WILDCARD_FUNCTION_ID   0xFFFF

#define LIN_NVR_SECTOR         2

#ifdef L_NAD_NVR  
    #define NAD_NVR_OFFSET             0            // offset to NVR where the NAD is stored (NAD is stored to byte 0)
    #define NAD_VALID_PATTERN          0xA55Au      // pattern on byte 2 and 3 of NVR_NVR_ADDRESS, that indicates the NAD as valid
#endif

#ifdef L_PID_NVR  
    #define PID_NVR_OFFSET             8            // offset to NVR where the NAD is stored (PID is stored to byte 0)
#endif

#ifdef L_UDS_BOOTLOADER_UPDATE
    #define BOOT_INFO_NVR_OFFSET       16           // offset to NVR where the BootInfo is stored
#endif

#define RESPONSE_ERROR_FRAME   0       // number of frame (in l_frame[]) containing the Response_Error signal
#define RESPONSE_ERROR_BYTE    7       // number of byte carrying the Response_Error signal
#define RESPONSE_ERROR_BIT    BIT_0    // bit carrying the Response_Error signal

//-----------------------------
// Interrupt priorities
//-----------------------------
#define LINUART_IRQ_PRIO      0          // set priority for LINUART IRQ to 0 (default after reset) 
#define LIN_IRQ_PRIO          0          // set priority for LIN I/O IRQ to 0 (default after reset)
#define TIM0_IRQ_PRIO         0          // set priority for TIM0 IRQ to 0 (default after reset)


/*******************************************************************************
*  type definitions
*******************************************************************************/
 
#ifdef L_UDS_BOOTLOADER_UPDATE

typedef struct {
    uint8_t infoDataLen;            /*Exchange inforamtion length must N * 4.*/
    uint8_t requestEnterBootloader; /*Request enter bootloader mode flag*/
    uint8_t downloadAPPSuccessful;  /*downlaod APP successful flag*/
    uint32_t infoStartAddr;         /*exchange information start address*/
    uint32_t requestEnterBootloaderAddr; /*Request enter bootloader flag address */
    uint32_t downloadAppSuccessfulAddr; /*download APP successful flag address*/
} BootInfo;

#endif

// IDs for node configuration
typedef struct {
    l_u16 supplierID;
    l_u16 functionID;
    l_u8 variantID;
} ProductIdentifier;

#define L_SERIAL_NUMBER        0x12345678
#define L_SUPPLIER_ID          0x0041    /* 0x0041 Just For example, Novosns don't supply Supplier ID */
#define L_FUNCTION_ID          0x1234
#define L_VARIANT_ID           0xAB

#ifdef L_EVENT_TRIGGERED_FRAME
    #define L_ID_TABLE_SIZE      9     // number of supported IDs (including 0x3C and 0x3D) 
#else
    #define L_ID_TABLE_SIZE      5     // number of supported IDs (including 0x3C and 0x3D)
#endif

#define DIAGNOSIS_REQUEST_FRAME  L_ID_TABLE_SIZE - 2
#define DIAGNOSIS_RESPONSE_FRAME L_ID_TABLE_SIZE - 1

#define NO_ATCP_STATUE_FRAME 0
#define NO_ATCP_CONTROL_FRAME 1
#define NO_ATCP_FAULT_FRAME 2
#define NO_CONFIRM_COM_TYPE_FRAME 3
#define NO_VERSION_INFO_FRAME 4

/*******************************************************************************
*  externals
*******************************************************************************/
extern const l_u8 g_serialNumber[];
extern const ProductIdentifier g_productIdentifier;
#if !(defined L_LIN_2_2)
extern const l_u16 g_configMessageIDTable[];
#endif
extern l_u8 g_nodeFramePIDTable[];
#ifdef L_EVENT_TRIGGERED_FRAME
extern l_bool g_eventTable[];
#endif
extern const ConfigFrame g_nodeConfigFrameTable[];

#endif   /* LIN_CONFIG_H  */
