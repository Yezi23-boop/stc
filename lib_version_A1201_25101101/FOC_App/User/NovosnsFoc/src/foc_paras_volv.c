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
// Stator resistance                            = 0.02 [Ohms]
// Pole-pair numbers                            = 7 [-]
// Direct axis inductance                       = 0.00057 [H]
// Quadrature axis inductance                   = 0.00098 [H]
// Back-EMF constant                            = 2.32 [V.sec/rad]
// Drive inertia                                = 0.0000005 [kg.m2]
// nominal current                              = 35 [A]
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
.ALIGN_DURATION                                = (int16_t)60000, 
.ALIGN_VOLTAGE                                 = FRAC16(0.50), 




// Current Loop Control
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 30 [Hz]
// Loop Attenuation                             = 1.5 [-]
// Loop sample time                             = 0.0001 [sec]
// --------------------------------------------------------------------------------
// Current Controller Output Limit 
.CLOOP_LIMIT                                   = FRAC16(0.9), 
// D-axis Controller - Recurrent type
.D_NSHIFT                                      = (int16_t)0, 
.D_CC1SC                                       = FRAC16(0.3925207397), 
.D_CC2SC                                       = FRAC16(-0.3899000839), 
// Q-axis Controller - Recurrent type
.Q_NSHIFT                                      = (int16_t)0, 
.Q_CC1SC                                       = FRAC16(0.6934755982), 
.Q_CC2SC                                       = FRAC16(-0.6889699093), 





// Speed Loop Control
// --------------------------------------------------------------------------------
// Ksp Manual                                   = 0.075 [Hz]
// Ksi Manual                                   = 0.0001 [-]
// --------------------------------------------------------------------------------
// Speed Controller - Parallel type
.SPEED_PI_PROP_GAIN                            = FRAC16(0.879645943), 
.SPEED_PI_PROP_SHIFT                           = (int16_t)2, 
.SPEED_PI_INTEG_GAIN                           = FRAC16(0.6005049638), 
.SPEED_PI_INTEG_SHIFT                          = (int16_t)-7, 
.SPEED_LOOP_HIGH_LIMIT                         = FRAC16(0.96), 
.SPEED_LOOP_LOW_LIMIT                          = FRAC16(-0.96), 
.SPEED_RAMP_UP                                 = FRAC16(0.0002), 
.SPEED_RAMP_DOWN                               = FRAC16(0.000075), 
.SPEED_LOOP_CNTR                               = (int16_t)10, 
.POSPE_SPEED_FILTER_MA_NPOINT                  = (int16_t)4, 

 








// Sensorless DQ BEMF Observer and Tracking Observer
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 200 [Hz]
// Loop Attenuation                             = 0.7 [-]
// --------------------------------------------------------------------------------
// DQ Bemf - plant coefficients 
.I_GAIN                                        = FRAC16(0.9937694704), 
.U_GAIN                                        = FRAC16(0.1246105919), 
.E_GAIN                                        = FRAC16(0.0361370717), 
.WI_GAIN                                       = FRAC16(0.2557641371), 
.BEMF_SHIFT                                    = (int16_t)0, 
.BEMF_DQ_CC1_GAIN                              = FRAC16(0.6123271452), 
.BEMF_DQ_CC2_GAIN                              = FRAC16(-0.5578741554), 
.BEMF_DQ_NSHIFT                                = (int16_t)2, 

// Tracking Observer - PI controller parameters
// --------------------------------------------------------------------------------
// Loop Bandwidth                               = 25 [Hz]
// Loop Attenuation                             = 0.7 [-]
// --------------------------------------------------------------------------------
.TO_CC1SC                                      = FRAC16(0.236941271), 
.TO_CC2SC                                      = FRAC16(-0.234297627), 
.TO_NSHIFT                                     = (int16_t)0, 
// Tracking Observer - Integrator  
.TO_THETA_GAIN                                 = FRAC16(0.0466666667), 
.TO_THETA_SHIFT                                = (int16_t)0, 












// Observer speed output filter 

// Open loop start-up
.OL_START_RAMP_INC                             = 1,//FRAC16(0.000125), 
.OL_START_I                                    = FRAC16(0.3), 
.MERG_SPEED_1_TRH                              = FRAC16(0.05), 
.MERG_SPEED_2_TRH                              = FRAC16(0.2), 

//HFI parameters
//运行阶段的Q轴电流
.HFI_ID_REF_RUN                                   = FRAC16(0.09),
//运行阶段的D轴电流
.HFI_IQ_REF_RUN                                   = FRAC16(0.15),

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


};
