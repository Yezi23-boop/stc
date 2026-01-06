/****************************************************************************
 * @file    : lin_nvr.c
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#include "include.h"
#include <string.h>
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"
#include "lin_identification_and_configuration.h"
#include "lin_nvr.h"

#define EraseSector ((unsigned int (*)(unsigned int eraseAddr))0x01000085)
#define ProgramPage                                                                  \
    ((unsigned int (*)(unsigned int writeAddr, unsigned int *tmpData))0x0100011D)


extern l_u8 g_nad;   // slave node address

#ifdef L_UDS_BOOTLOADER_UPDATE
static const BootInfo g_stBootInfo = {
    .infoDataLen = 8u,
    .requestEnterBootloader = 0x5Au,
    .downloadAPPSuccessful = 0xA5u,
    .infoStartAddr = NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET,
    .requestEnterBootloaderAddr = NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET + 1,
    .downloadAppSuccessfulAddr = NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET
};
#endif

uint32_t g_NVRProgramDatabuffer[NVR_PAGE_SIZE / 4];

#ifdef L_UDS_BOOTLOADER_UPDATE
/************************************************************
 * @brief: set download app successful
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void SetRequestEnterBootloader(void)
{
    memset(g_NVRProgramDatabuffer, NVR_PAGE_SIZE, 0x00);
    g_NVRProgramDatabuffer[BOOT_INFO_NVR_OFFSET / 4] = (uint16_t)g_stBootInfo.requestEnterBootloader << 8;
    g_NVRProgramDatabuffer[BOOT_INFO_NVR_OFFSET / 4 + 1] = ((uint32_t)UPDATE_FIRMWARE_FIX_CRC_CODE) << 16;
#ifdef L_NAD_NVR
    g_NVRProgramDatabuffer[NAD_NVR_OFFSET / 4] = 
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + NAD_NVR_OFFSET));
#endif
#ifdef L_PID_NVR
    g_NVRProgramDatabuffer[PID_NVR_OFFSET / 4]  =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + PID_NVR_OFFSET));
    g_NVRProgramDatabuffer[PID_NVR_OFFSET / 4 + 1]  =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + PID_NVR_OFFSET + 4));
#endif
    FLASHCTRL->TMCTRL_b.UNIT = (48 - 1);
    EraseSector(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR);
    ProgramPage(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR, g_NVRProgramDatabuffer);

}
#endif

#ifdef L_NAD_NVR
/******************************************************************************
 * @brief: Read NAD from NVR (if valid) and store ist to g_nad.
 * @author: Novosns MCU Team
 * @return <None>
 *******************************************************************************/
void ReadNADFromNVR(void)
{
    uint32_t nvrCode = *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + NAD_NVR_OFFSET));
    if ((nvrCode >> 16) == NAD_VALID_PATTERN) {
        g_nad = nvrCode & 0xFF;
    } else {
        g_nad = INITIAL_NAD;
    }
}

/******************************************************************************
 * @brief: Write NAD and NAD pattern to NVR
 * @author: Novosns MCU Team
 * @return <None>
 *******************************************************************************/
void WriteNADToNVR(void)
{
    memset(g_NVRProgramDatabuffer, NVR_PAGE_SIZE, 0x00);
    g_NVRProgramDatabuffer[NAD_NVR_OFFSET / 4] = (NAD_VALID_PATTERN << 16) + g_nad;
#ifdef L_PID_NVR
    g_NVRProgramDatabuffer[PID_NVR_OFFSET / 4]  =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + PID_NVR_OFFSET));
    g_NVRProgramDatabuffer[PID_NVR_OFFSET / 4 + 1]  =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + PID_NVR_OFFSET + 4));
#endif
#ifdef L_UDS_BOOTLOADER_UPDATE
    g_NVRProgramDatabuffer[BOOT_INFO_NVR_OFFSET / 4]  =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET));
    g_NVRProgramDatabuffer[BOOT_INFO_NVR_OFFSET / 4 + 1]  =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET + 4));
#endif
    FLASHCTRL->TMCTRL_b.UNIT = (48 - 1);
    EraseSector(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR);
    ProgramPage(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR, g_NVRProgramDatabuffer);
}

#endif

#ifdef L_PID_NVR
/******************************************************************************
 * @brief: Read PID from NVR (if valid) and store ist to g_nad.
 * @author: Novosns MCU Team
 * @return <None>
 *******************************************************************************/
void ReadPIDFromNVR(uint8_t *PIDTable, uint8_t len)
{
    uint64_t nvrCode = *((uint64_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + PID_NVR_OFFSET));
    uint8_t lenCode = nvrCode & 0xFF;
    uint8_t lenInvertCode = (nvrCode >> 8) & 0xFF;
    uint8_t invertLen = ~len;
    if (len <= 6 && lenCode == len && lenInvertCode == invertLen) {
        uint8_t *pidAddr = (uint8_t *)&nvrCode + 2;
        for (int i = 0; i <len; i++) {
            PIDTable[i] = pidAddr[i];
        }
    }
}

/******************************************************************************
 * @brief: Write PID , PID len code and len invert code to NVR
 * @author: Novosns MCU Team
 * @return <None>
 *******************************************************************************/
void WritePIDToNVR(uint8_t *PIDTable, uint8_t len)
{
    memset(g_NVRProgramDatabuffer, NVR_PAGE_SIZE, 0x00);
    if (len > 6) {
        return;
    }
    g_NVRProgramDatabuffer[PID_NVR_OFFSET / 4] = len + ((uint16_t)(~len) << 8);
    uint8_t *pidAddr = (uint8_t *)&(g_NVRProgramDatabuffer[PID_NVR_OFFSET / 4]) + 2;
    for (int i = 0; i < len; i++) {
        pidAddr[i] = PIDTable[i];
    }
#ifdef L_UDS_BOOTLOADER_UPDATE
    g_NVRProgramDatabuffer[BOOT_INFO_NVR_OFFSET / 4] =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET));
    g_NVRProgramDatabuffer[BOOT_INFO_NVR_OFFSET / 4 + 1] =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + BOOT_INFO_NVR_OFFSET + 4));
#endif

#ifdef L_NAD_NVR
    g_NVRProgramDatabuffer[NAD_NVR_OFFSET / 4] =
        *((uint32_t *)(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR + NAD_NVR_OFFSET));
#endif

    FLASHCTRL->TMCTRL_b.UNIT = (48 - 1);
    EraseSector(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR);
    ProgramPage(NVR_BASE + NVR_SECTOR_SIZE * LIN_NVR_SECTOR, g_NVRProgramDatabuffer);

}
#endif

