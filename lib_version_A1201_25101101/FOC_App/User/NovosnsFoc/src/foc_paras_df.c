/************************************************************
 * @file: foc_paras.c
 * @author: Novosns MCU Team
 * @version: V1.0
 * @data: 2023/12/13
 * @brief: foc parameters for NSUC1602
 * @note: 
 * @Copyright (C) 2022 Novosense All rights reserved.
 ************************************************************/

 /************************************************************
 ************************************************************
 * @par Edition History
 * -V0.0  2023.12.13
 *        -Initial version for foc_paras.c of NSUC1602.
 *
 ************************************************************/

#include "foc_paras.h"
#include "fault_hw_config.h"

const FOC_Paras_t tFocParas = {

// Motor Parameters
// --------------------------------------------------------------------------------
// Stator resistance                            = 0.019 [Ohms]
// Pole-pair numbers                            = 4 [-]
// Direct axis inductance                       = 0.000047 [H]
// Quadrature axis inductance                   = 0.000047 [H]
// Back-EMF constant                            = 0.0030558 [V.sec/rad]
// Drive inertia                                = 0.0000005 [kg.m2]
// nominal current                              = 0.8 [A]
// nominal speed                                = 100000 [rpm]


.MOTOR_PP_GAIN                                 = FRAC16(0.625), 
.MOTOR_PP_SHIFT                                = (int16_t)3, 

// Application Scales
// --------------------------------------------------------------------------------
.I_MAX                                         = I_FULL_SCALE, 
.U_DCB_MAX                                     = V_FULL_SCALE, 
.N_MAX                                         = (float)MAX_RPM, 
.WEL_MAX                                       = (float)2094.4, 
.E_MAX                                         = (float)14.4, 

// Application Fault Triggers
// --------------------------------------------------------------------------------
.U_DCB_TRIP                                    = FRAC16(0.36), 
.U_DCB_UNDERVOLTAGE                            = FRAC16(UV_VOLTAGE_UPLIMT/V_FULL_SCALE), 
.U_DCB_OVERVOLTAGE                             = FRAC16((OV_VOLTAGE_UPLIMT/V_FULL_SCALE)), 
.I_PH_OVER                                     = FRAC16(0.016), 
.TEMP_OVER                                     = FRAC16(0.1704897706), 
// DC Bus voltage MA filter constants
// --------------------------------------------------------------------------------
// Cut-off frequency                            = 99.4718394324 [Hz]
// Sample time                                  = 0.0001 [sec]
.DC_BUS_FILTER_MA_NPOINT                       = (int16_t)4, 

// Mechanical alignment
// --------------------------------------------------------------------------------
.ALIGN_DURATION                                = (int16_t)30000, 
.ALIGN_VOLTAGE                                 = FRAC16(0.10), 





// Current Loop Control
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 30 [Hz]
// Loop Attenuation                             = 1.2 [-]
// Loop sample time                             = 0.0001 [sec]
// --------------------------------------------------------------------------------
// Current Controller Output Limit 
.CLOOP_LIMIT                                   = FRAC16(0.9), 
// D-axis Controller - Recurrent type
.D_NSHIFT                                      = (int16_t)0, 
.D_CC1SC                                       = FRAC16(0.0029322449), 
.D_CC2SC                                       = FRAC16(-0.0027235028), 
// Q-axis Controller - Recurrent type
.Q_NSHIFT                                      = (int16_t)0, 
.Q_CC1SC                                       = FRAC16(0.0029322449), 
.Q_CC2SC                                       = FRAC16(-0.0027235028), 













//.D_NSHIFT                                      = (int16_t)0, 
//.D_CC1SC                                       = FRAC16(0.1710801727), 
//.D_CC2SC                                       = FRAC16(-0.1648273798), 
//// Q-axis Controller - Recurrent type
//.Q_NSHIFT                                      = (int16_t)0, 
//.Q_CC1SC                                       = FRAC16(0.2300268669), 
//.Q_CC2SC                                       = FRAC16(-0.2217878928), 

// Speed Loop Control
// --------------------------------------------------------------------------------
// Ksp Manual                                   = 0.055 [Hz]
// Ksi Manual                                   = 0.00001 [-]
// --------------------------------------------------------------------------------
// Speed Controller - Parallel type
.SPEED_PI_PROP_GAIN                            = FRAC16(0.737227076), 
.SPEED_PI_PROP_SHIFT                           = (int16_t)1, 
.SPEED_PI_INTEG_GAIN                           = FRAC16(0.5490331097), 
.SPEED_PI_INTEG_SHIFT                          = (int16_t)-11, 
.SPEED_LOOP_HIGH_LIMIT                         = FRAC16(0.96), 
.SPEED_LOOP_LOW_LIMIT                          = FRAC16(-0.96), 
.SPEED_RAMP_UP                                 = FRAC16(0.000075), 
.SPEED_RAMP_DOWN                               = FRAC16(0.000125), 
.SPEED_LOOP_CNTR                               = (int16_t)10, 
.POSPE_SPEED_FILTER_MA_NPOINT                  = (int16_t)4, 







// Sensorless DQ BEMF Observer and Tracking Observer
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 300 [Hz]
// Loop Attenuation                             = 0.9 [-]
// --------------------------------------------------------------------------------
// DQ Bemf - plant coefficients 
.I_GAIN                                        = FRAC16(0.960375391), 
.U_GAIN                                        = FRAC16(0.8342022941), 
.E_GAIN                                        = FRAC16(0.2252346194), 
.WI_GAIN                                       = FRAC16(0.0821160124), 
.BEMF_SHIFT                                    = (int16_t)0, 
.BEMF_DQ_CC1_GAIN                              = FRAC16(0.6889672612), 
.BEMF_DQ_CC2_GAIN                              = FRAC16(-0.6116553601), 
.BEMF_DQ_NSHIFT                                = (int16_t)0, 





// Tracking Observer - PI controller parameters
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 15 [Hz]
// Loop Attenuation                             = 0.9 [-]
// --------------------------------------------------------------------------------
.TO_CC1SC                                      = FRAC16(0.318919004), 
.TO_CC2SC                                      = FRAC16(-0.3172535083), 
.TO_NSHIFT                                     = (int16_t)0, 
// Tracking Observer - Integrator  
.TO_THETA_GAIN                                 = FRAC16(0.0266666667), 
.TO_THETA_SHIFT                                = (int16_t)0, 


// Observer speed output filter 

// Open loop start-up
.OL_START_RAMP_INC                             = 1,//FRAC16(0.000125), 
.OL_START_I                                    = FRAC16(0.25), 
.MERG_SPEED_1_TRH                              = FRAC16(0.025), 
.MERG_SPEED_2_TRH                              = FRAC16(0.055), 

//HFI parameters
//运行阶段的Q轴电流
.HFI_ID_REF_RUN                                   = FRAC16(0.09),
//运行阶段的D轴电流
.HFI_IQ_REF_RUN                                   = FRAC16(0.15),
.HFI_SPEED_LOOP_CNTR                               = 8, 

// Current Loop Control
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 30 [Hz]
// Loop Attenuation                             = 1.2 [-]
// Loop sample time                             = 0.000125 [sec]
// --------------------------------------------------------------------------------


.HFI_CLOOP_LIMIT                             = FRAC16(0.9),
// D-axis Controller - Recurrent type
.HFI_D_NSHIFT                                = (int16_t)0,
.HFI_D_CC1SC                                 = FRAC16(0.0029583377),
.HFI_D_CC2SC                                 = FRAC16(-0.00269741),
// Q-axis Controller - Recurrent type
.HFI_Q_NSHIFT                                = (int16_t)0,
.HFI_Q_CC1SC                                 = FRAC16(0.0029583377),
.HFI_Q_CC2SC                                 = FRAC16(-0.00269741),

// Speed Loop Control
// --------------------------------------------------------------------------------
// Ksp Manual                                   = 0.055 [Hz]
// Ksi Manual                                   = 0.00001 [-]
// --------------------------------------------------------------------------------

.HFI_SPEED_PI_PROP_GAINL                            = FRAC16(0.737227076), 
.HFI_SPEED_PI_PROP_SHIFT                           = (int16_t)1,  //2
.HFI_SPEED_PI_INTEG_GAIN                           = FRAC16(0.5490331097), 
.HFI_SPEED_PI_INTEG_SHIFT                          = (int16_t)-11, 
.HFI_SPEED_LOOP_HIGH_LIMIT                         = FRAC16(0.96), 
.HFI_SPEED_LOOP_LOW_LIMIT                          = FRAC16(-0.96), 
.HFI_SPEED_LOOP_CNTR                                = 8,
};
