/****************************************************************************
 * @file    : lin_nvr.h
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#ifndef  LIN_NVR_H
#define  LIN_NVR_H
/*******************************************************************************
  type definitions
*******************************************************************************/
#include <stdint.h>

#define UPDATE_FIRMWARE_FIX_CRC_CODE 0x7692u
#define NVR_SECTOR_SIZE              (512)
#define NVR_PAGE_SIZE                (128)

#ifdef L_NAD_NVR  
void ReadNADFromNVR(void);
void WriteNADToNVR(void);
#endif 

#ifdef L_PID_NVR
void ReadPIDFromNVR(uint8_t *PIDTable, uint8_t len);
void WritePIDToNVR(uint8_t *PIDTable, uint8_t len);
#endif 

#ifdef L_UDS_BOOTLOADER_UPDATE
void SetRequestEnterBootloader(void);
#endif


#endif
