/*
 * I2C.h
 *
 *  Created on: 11 Haz 2014
 *      Author: bugra
 */
#include <msp430fr5994.h>
#ifndef I2C_H_
#define I2C_H_
void delay(unsigned int wait);
enum BUS {UCB1, UCB3};
namespace MSP430
{
class I2C
{
public:
	I2C(BUS UCxx);
	unsigned char transmit(unsigned char* txData, unsigned int size, unsigned char slaveAddress);
	unsigned char receive(unsigned char* txData, unsigned int size, unsigned char slaveAddress);
private:
	BUS CurrentBus;
};

} /* namespace MSP430 */
#endif /* I2C_H_ */
