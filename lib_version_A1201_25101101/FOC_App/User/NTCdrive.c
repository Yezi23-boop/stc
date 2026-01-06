
#include "Module_Define.h"
#include "NTCdrive.h"
//#include "types.h"




// AD值 对应温度，从-40~200摄氏度，共241个点
const uint16_t Sensor1Table[] = {
	 3907,  3896,  3885,  3874,  3862,  3849,  3836,  3822,  3808,  3793,
	 3778,  3762,  3745,  3728,  3710,  3691,  3672,  3652,  3631,  3610,
	 3588,  3565,  3542,  3517,  3493,  3467,  3441,  3414,  3386,  3358,
	 3329,  3299,  3269,  3238,  3206,  3174,  3141,  3107,  3073,  3039,
	 3004,  2968,  2932,  2896,  2859,  2822,  2784,  2746,  2708,  2669,
	 2630,  2591,  2552,  2513,  2473,  2433,  2394,  2354,  2314,  2275,
	 2235,  2196,  2157,  2117,  2078,  2039,  2001,  1963,  1925,  1887,
	 1850,  1813,  1776,  1740,  1704,  1668,  1633,  1599,  1564,  1531,
	 1498,  1465,  1433,  1401,  1370,  1339,  1309,  1280,  1250,  1222,
	 1194,  1167,  1140,  1113,  1087,  1062,  1037,  1013,   989,   965,
	  943,   920,   899,   877,   856,   836,   816,   797,   777,   759,
	  741,   723,   706,   689,   672,   656,   640,   625,   610,   596,
	  581,   568,   554,   541,   528,   515,   503,   491,   480,   468,
	  457,   447,   436,   426,   416,   406,   397,   387,   379,   370,
	  361,   353,   345,   337,   329,   322,   315,   308,   301,   294,
	  287,   281,   275,   269,   263,   257,   251,   246,   241,   235,
	  230,   225,   220,   216,   211,   206,   202,   198,   193,   189,
	  185,   181,   177,   173,   170,   166,   163,   159,   156,   153,
	  150,   146,   143,   140,   138,   135,   132,   129,   127,   124,
	  122,   119,   117,   114,   112,   110,   108,   105,   103,   101,
	   99,    97,    95,    93,    91,    90,    88,    86,    84,    83,
	   81,    80,    78,    76,    75,    74,    72,    71,    69,    68,
	   67,    66,    64,    63,    62,    61,    60,    59,    57,    57,
	   55,    55,    53,    53,    51,    51,    50,    49,    48,    47,
	   46
};





#define SENSOR1_OFFSET   40     // 第1个是-40度
#define SENSOR2_OFFSET   40     // 第2个是-40度
#define SENSOR3_OFFSET   55     // 第3个是-55度

int16_t CalcSensorTemp(const uint16_t *table, const uint16_t len, const int16_t offset, const uint16_t ad)
{
    int16_t low, middle, high;
    int16_t fract;

    low = 0;
    high = len - 1;

    while (low <= high) {
        middle = (low + high) / 2;

        if ((middle == 0) || (middle == len - 1))
        {
        	break;
        }

        if ((ad <= table[middle]) && (ad >= table[middle + 1]))
        {
        	break;
        }
        else if (ad < table[middle])
        {
            low = middle + 1;
        }
        else if (ad > table[middle])
        {
            high = middle - 1;
        }
    }

    if (middle >= len - 1)
	{
    	fract = 0;
	}
    else
    {
    	fract = ((int16_t)table[middle] - (int16_t)ad) * 10 / ((int16_t)table[middle] - (int16_t)table[middle + 1]);
    }

	return (middle - offset) * 10 + fract;
}


int16_t CalcIgbtTemp(uint16_t ad)
{
	int16_t tmp;
	tmp = CalcSensorTemp(Sensor1Table, sizeof(Sensor1Table)/sizeof(uint16_t), SENSOR1_OFFSET, ad);

	if (tmp < -400)
	{
		tmp = -400;
	}
	else if (tmp > 2000)
	{
		tmp = 2000;
	}

	return tmp;

}
/*
    ntc1.temperature:    max:1500   min:-350   
*/
