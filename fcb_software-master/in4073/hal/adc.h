#ifndef ADC_H_
#define ADC_H_

#include <inttypes.h>

extern uint16_t bat_volt;

void adc_init(void);
void adc_request_sample(void);

#endif /* ADC_H_ */
