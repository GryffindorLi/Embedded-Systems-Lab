#ifndef TWI_H_
#define TWI_H_

#include <stdbool.h>
#include <inttypes.h>

#define TWI_SCL	4
#define TWI_SDA	2

void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);


#endif /* TWI_H_ */
