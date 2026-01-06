#ifndef __PLL_PI_H__
#define __PLL_PI_H__

#include "mm_lib.h"

typedef struct {
    int32_t  qdsum;             // 1.31 format
    int16_t  qkp;
    int16_t  qki;                               
    int16_t  qkc;
    int16_t  qoutmax;
    int16_t  qoutmin;
    int16_t  qinref; 
    int16_t  qinmeas;
    int16_t  qout;
} PLL_PI;

extern void init_pll_pi(PLL_PI *pParm,int16_t kp,int16_t ki,int16_t kc,int16_t max,int16_t min);
extern void run_parallel_pi(PLL_PI *pParm);

#endif