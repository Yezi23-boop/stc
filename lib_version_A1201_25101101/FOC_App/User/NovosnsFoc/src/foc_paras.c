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

FOC_Paras_t tFocParas = {

// Motor Parameters
// --------------------------------------------------------------------------------
// Stator resistance                            = 0.02 [Ohms]
// Pole-pair numbers                            = 7 [-]
// Direct axis inductance                       = 0.00002 [H]
// Quadrature axis inductance                   = 0.000025 [H]
// Back-EMF constant                            = 2.32 [V.sec/rad]
// Drive inertia                                = 0.0000005 [kg.m2]
// nominal current                              = 0.2 [A]
// nominal speed                                = 10000 [rpm]
.MOTOR_PP_GAIN                                 = FRAC16(0.875), 
.MOTOR_PP_SHIFT                                = (int16_t)3, 



// Application Scales
// --------------------------------------------------------------------------------
.I_MAX                                         = I_FULL_SCALE, 
.U_DCB_MAX                                     = V_FULL_SCALE, 
.N_MAX                                         = (float)MAX_RPM, 
.WEL_MAX                                       = (float)2932.15, 
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
.ALIGN_DURATION                                = (int16_t)FOC_TIME_MS_CALC(2000), 
.ALIGN_WIND_DETECT_TIME                         = (int16_t)FOC_TIME_MS_CALC(500),
.ALIGN_BRAKE_TIME                               = (int16_t)FOC_TIME_MS_CALC(500),
.ALIGN_VOLTAGE                                 = FRAC16(0.10), 
.ALIGN_ID_REF                                 = FRAC16(0.2),


// Current Loop Control
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 50 [Hz]
// Loop Attenuation                             = 2 [-]
// Loop sample time                             = 0.0001 [sec]
// --------------------------------------------------------------------------------
// Current Controller Output Limit 
.CLOOP_LIMIT                                   = FRAC16(0.9), 
// D-axis Controller - Recurrent type
.D_NSHIFT                                      = (int16_t)0, 
.D_CC1SC                                       = FRAC16(0.0051369894), 
.D_CC2SC                                       = FRAC16(-0.0048943361), 
// Q-axis Controller - Recurrent type
.Q_NSHIFT                                      = (int16_t)0, 
.Q_CC1SC                                       = FRAC16(0.014931802), 
.Q_CC2SC                                       = FRAC16(-0.0146125213), 






// Speed Loop Control
// --------------------------------------------------------------------------------
// Ksp Manual                                   = 0.055 [Hz]
// Ksi Manual                                   = 0.0001 [-]
// --------------------------------------------------------------------------------
// Speed Controller - Parallel type
.SPEED_PI_PROP_GAIN                            = FRAC16(0.6450736915), 
.SPEED_PI_PROP_SHIFT                           = (int16_t)2, 
.SPEED_PI_INTEG_GAIN                           = FRAC16(0), 
.SPEED_PI_INTEG_SHIFT                          = (int16_t)-7, 
.SPEED_LOOP_HIGH_LIMIT                         = FRAC16(0.96), 
.SPEED_LOOP_LOW_LIMIT                          = FRAC16(-0.96), 
.SPEED_RAMP_UP                                 = SPEED_ACCEL_PU_F16, 
.SPEED_RAMP_DOWN                               = SPEED_DECEL_PU_F16, 
.SPEED_LOOP_CNTR                               = (int16_t)SPEED_LOOP_DIV, 
.POSPE_SPEED_FILTER_MA_NPOINT                  = (int16_t)10,


// Sensorless DQ BEMF Observer and Tracking Observer
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 100 [Hz]
// Loop Attenuation                             = 0.7 [-]
// --------------------------------------------------------------------------------
// DQ Bemf - plant coefficients 
.I_GAIN                                        = FRAC16(0.9), 
.U_GAIN                                        = FRAC16(0.966), 
.E_GAIN                                        = FRAC16(0.42), 
.WI_GAIN                                       = FRAC16(0.0916297857), 
.BEMF_SHIFT                                    = (int16_t)1, 
.BEMF_DQ_CC1_GAIN                              = FRAC16(-0.0086657206), 
.BEMF_DQ_CC2_GAIN                              = FRAC16(0.0108981311), 
.BEMF_DQ_NSHIFT                                = (int16_t)0, 

// Tracking Observer - PI controller parameters
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 5 [Hz]
// Loop Attenuation                             = 0.7 [-]
// --------------------------------------------------------------------------------
.TO_CC1SC                                      = FRAC16(0.0471767627), 
.TO_CC2SC                                      = FRAC16(-0.0470710169), 
.TO_NSHIFT                                     = (int16_t)0, 

// Tracking Observer - Integrator  
.TO_THETA_GAIN                                 = FRAC16(0.0466666667), 
.TO_THETA_SHIFT                                = (int16_t)0, 


// Observer speed output filter 

// Open loop start-up
.OL_START_RAMP_INC                             = 1,//FRAC16(0.000125), 
.OL_START_I                                    = FRAC16(0.6), 
.MERG_SPEED_1_TRH                              = FRAC16(0.01), 
.MERG_SPEED_2_TRH                              = FRAC16(0.1), 

//HFI parameters
//运行阶段的Q轴电流
.HFI_ID_REF_RUN                                   = FRAC16(0.09),
//运行阶段的D轴电流
.HFI_IQ_REF_RUN                                   = FRAC16(0.15),
.HFI_SPEED_LOOP_CNTR                               = 8, 

// Current Loop Control
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 30 [Hz]
// Loop Attenuation                             = 1.5 [-]
// Loop sample time                             = 0.0001 [sec]
// --------------------------------------------------------------------------------
// Current Controller Output Limit 
.HFI_CLOOP_LIMIT                                   = FRAC16(0.9), 
// D-axis Controller - Recurrent type
.HFI_D_NSHIFT                                      = (int16_t)0, 
.HFI_D_CC1SC                                       = FRAC16(0.3925207397), 
.HFI_D_CC2SC                                       = FRAC16(-0.3899000839), 
// Q-axis Controller - Recurrent type
.HFI_Q_NSHIFT                                      = (int16_t)0, 
.HFI_Q_CC1SC                                       = FRAC16(0.6934755982), 
.HFI_Q_CC2SC                                       = FRAC16(-0.6889699093), 


.HFI_SPEED_PI_PROP_GAINL                            = FRAC16(0.737227076), 
.HFI_SPEED_PI_PROP_SHIFT                           = (int16_t)1,  //2
.HFI_SPEED_PI_INTEG_GAIN                           = FRAC16(0.5490331097), 
.HFI_SPEED_PI_INTEG_SHIFT                          = (int16_t)-11, 
.HFI_SPEED_LOOP_HIGH_LIMIT                         = FRAC16(0.96), 
.HFI_SPEED_LOOP_LOW_LIMIT                          = FRAC16(-0.96), 
.HFI_SPEED_LOOP_CNTR                                = 8,
};
