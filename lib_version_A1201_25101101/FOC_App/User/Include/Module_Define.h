#ifndef MODULE_DEFINE_H_
#define MODULE_DEFINE_H_

#ifdef __cplusplus
extern "C"
{
#endif


#define UPDOWNLIMIT(A,B,C) { A =((A)>=(B)?(B):(A)); A=((A)>=(C)?(A):(C));}
#define ABS(A) ((A)>0?(A):-(A))

#include "include.h"
#include "stdio.h"
#include "nvsns_foc.h"
#include "foc_init.h"
#include "foc_paras.h"
#include "fault_diagnose.h"
#include "fault_init.h"
#include "fault_handler.h"
#include "freemaster.h"
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"
#include "lin_nvr.h"
#include "lin_identification_and_configuration.h"
#include "curr_sample_1rps.h"
#include "foc_config.h"
#include "SysTick.h"
#include "SysTick_isr.h"

#include "config.h"
#include "global.h"
#include "derating_control.h"
#include "motor_state_ctrl.h"
#include "NTCdrive.h"


//-----------------


#ifdef __cplusplus
}
#endif



#endif


