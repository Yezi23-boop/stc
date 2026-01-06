/****************************************************************************
 * @file    : lin_config.c
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/

/*******************************************************************************
 * Include files
 *******************************************************************************/
#include "lin_define.h"
#include "lin_config.h"

/*******************************************************************************
 * definitions
 *******************************************************************************/

/*******************************************************************************
 * external variables and constants
 *******************************************************************************/

const l_u8 g_serialNumber[4] = {
    L_SERIAL_NUMBER & 0xFF, // low byte first
    (L_SERIAL_NUMBER >> 8) & 0xFF,
    (L_SERIAL_NUMBER >> 16) & 0xFF,
    (L_SERIAL_NUMBER >> 24) & 0xFF
};

const ProductIdentifier g_productIdentifier = {
    .supplierID = L_SUPPLIER_ID,
    .functionID = L_FUNCTION_ID,
    .variantID = L_VARIANT_ID
};

#if !(defined L_LIN_2_2)
const l_u16 g_configMessageIDTable[L_ID_TABLE_SIZE - 2] = { // -2 because there is no message ID for ID_60 and ID_61
    0x0101, // supported message IDs
    0x0102,
    0x0103,
    0x0104
#ifdef L_EVENT_TRIGGERED_FRAME
    , // separator to previous line
    0x0105,
    0x0106
#endif
};
#endif

// data buffer for the supported LIN frames

static l_u8 g_linFrameDataBufferOf_ID02[8] = {0, 0, 0, 0, 0, 0, 0, 0};  //ATCP_STATUS_FRAME 0x02
static l_u8 g_linFrameDataBufferOf_ID03[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // ATCP_CONTROL_FRAME 0x03
static l_u8 g_linFrameDataBufferOf_ID48[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // ATCP_FAULT_FRAME
static l_u8 g_linFrameDataBufferOf_ID58[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // frame for confirm com typr if receive this fram in 500ms ,confirm the com type is LIN
static l_u8 g_linFrameDataBufferOf_ID59[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // frame for version information
#ifdef L_EVENT_TRIGGERED_FRAME
static l_u8 g_linFrameDataBufferOf_ID15[2]; // send 2 byte to host (PID and counter value)
static l_u8 g_linFrameDataBufferOf_ID16[2]; // send 2 byte to host (PID and counter value)
#endif

static l_u8 g_linFrameDataBufferOf_ID60[8];
static l_u8 g_linFrameDataBufferOf_ID61[8];

l_u8 g_nodeFramePIDTable[L_ID_TABLE_SIZE] = { // list of supported PIDs
    ID_02, //02
    ID_03, //03
    ID_48, //0X30
    ID_58,  //3A
    ID_59, //3B
#ifdef L_EVENT_TRIGGERED_FRAME
    ID_15, // unconditional frame associated with event triggered frame ID_16
    ID_16, // event triggered frame that carries frame ID_15
    // Note: In this table the EVT must be placed directly
    // after the associated unconditional frame !!!
#endif
    ID_60, // ID_60 and ID_61 must be at the end of this table !!!
    ID_61
};

#ifdef L_EVENT_TRIGGERED_FRAME
l_bool g_eventTable[L_ID_TABLE_SIZE]; // this table is associated to l_PID_Table[]
// if the corresponding flag is set, the event triggered frame is sent
#endif

const ConfigFrame g_nodeConfigFrameTable[L_ID_TABLE_SIZE] = { // attributes of the supported LIN frames
    {.direction = ID_DATASEND,       .length = sizeof(g_linFrameDataBufferOf_ID02),  .var = g_linFrameDataBufferOf_ID02},
    {.direction = ID_DATARECEIVE,    .length = sizeof(g_linFrameDataBufferOf_ID03),  .var = g_linFrameDataBufferOf_ID03},
    {.direction = ID_DATASEND,    .length = sizeof(g_linFrameDataBufferOf_ID48),  .var = g_linFrameDataBufferOf_ID48},
    {.direction = ID_DATARECEIVE,    .length = sizeof(g_linFrameDataBufferOf_ID58),  .var = g_linFrameDataBufferOf_ID58},
    {.direction = ID_DATASEND,    .length = sizeof(g_linFrameDataBufferOf_ID59),  .var = g_linFrameDataBufferOf_ID59},
#ifdef L_EVENT_TRIGGERED_FRAME
    {.direction = ID_DATASEND,       .length = sizeof(g_linFrameDataBufferOf_ID15),  .var = g_linFrameDataBufferOf_ID15},
    {.direction = ID_DATASEND_EVENT, .length = sizeof(g_linFrameDataBufferOf_ID16),  .var = g_linFrameDataBufferOf_ID16},
#endif
    // ID_60 and ID_61 must be at the end of this table !!!
    {.direction = ID_DATARECEIVE,    .length = sizeof(g_linFrameDataBufferOf_ID60),  .var = g_linFrameDataBufferOf_ID60},
    {.direction = ID_DATASEND,       .length = sizeof(g_linFrameDataBufferOf_ID61),  .var = g_linFrameDataBufferOf_ID61},
};
