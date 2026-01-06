#ifndef _NTCDRIVE_H_
#define _NTCDRIVE_H_

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct
{
	int16_t voltage;
	int16_t temperature;
	const uint16_t *ptr;
}NTC;

extern void NTCdriveCalc(NTC *v);
extern int16_t CalcIgbtTemp(uint16_t ad);
extern int16_t CalcNtc2Temp(uint16_t ad);

#ifdef __cplusplus
}
#endif

#endif

