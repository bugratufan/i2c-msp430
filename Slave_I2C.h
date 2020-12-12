/*
 * Slave_I2C.h
 *
 *  Created on: 16 Mar 2018
 *      Author: bugra
 */
#include <msp430fr5994.h>
#ifndef Slave_I2C_H_
#define Slave_I2C_H_
enum Slave_BUS {UCB2};
namespace MSP430{
    class Slave_I2C{
        public:
            Slave_I2C(Slave_BUS UCxx);
        	  unsigned char transmit(unsigned char* txData, unsigned int size, unsigned char ownAddress);
        	  unsigned char receive(unsigned char* rxData, unsigned int size, unsigned char ownAddress);
        private:
        	  Slave_BUS CurrentBus;
    };
} /* namespace MSP430 */
#endif /* Slave_I2C_H_ */
