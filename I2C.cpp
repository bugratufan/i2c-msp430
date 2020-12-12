/*
 * I2C.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: bugra
 */

#include "I2C.h"

namespace MSP430
{

bool UCB3_I2C_ACK = false;
unsigned char* UCB3TXPtr;
unsigned int UCB3TXLength;
unsigned char* UCB3RXPtr;
unsigned int UCB3RXLength;

bool UCB1_I2C_ACK = false;
unsigned char* UCB1TXPtr;
unsigned int UCB1TXLength;
unsigned char* UCB1RXPtr;
unsigned int UCB1RXLength;


I2C::I2C(BUS UCxx)
{
    CurrentBus = UCxx;
	switch(CurrentBus){
        case UCB3:
            P6SEL0 |= BIT4 | BIT5;
            P6SEL1 &= ~(BIT4 | BIT5);

            UCB3CTLW0 = UCSWRST;
            UCB3CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK;
            UCB3BRW = 0x8;
            UCB3CTLW1 |= UCASTP_2;
            UCB3CTLW1 |= 0x40;
            UCB3CTLW0 &= ~UCSWRST;
            UCB3IE |= UCNACKIE + UCALIE + UCSTPIE + UCSTTIE + UCTXIE0 + UCRXIE0 | UCBCNTIE | USCI_I2C_UCCLTOIFG;

            break;
        case UCB1:
            P5SEL0 |= BIT0 | BIT1;
            P5SEL1 &= ~(BIT0 | BIT1);

            UCB1CTLW0 = UCSWRST;
            UCB1CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK;
            UCB1BRW = 0x8;
            UCB1CTLW1 |= 0x40;
            UCB1CTLW1 |= UCASTP_2;
            UCB1CTLW0 &= ~UCSWRST;
            UCB1IE |= UCNACKIE + UCALIE + UCSTPIE + UCSTTIE + UCTXIE0 + UCRXIE0 | UCBCNTIE | USCI_I2C_UCCLTOIFG;

            break;

	}
}

unsigned char I2C::transmit(unsigned char* txData, unsigned int size, unsigned char slaveAddress){
    unsigned int TimeoutCounter;
	switch(CurrentBus){
        case UCB3:
            UCB3I2CSA = slaveAddress; // Slave Address
            UCB3TXPtr = txData; // Assign Pointer
            UCB3TXLength = size; // Assign Length
            TimeoutCounter = 500;
            while((UCB3CTLW0 & UCTXSTP) && TimeoutCounter){
                TimeoutCounter--;
                delay(1);
            }
            if(!TimeoutCounter) return 0;
            UCB3TBCNT = size;
            UCB3CTLW0 |= UCTR + UCTXSTT; // Start Condition
            __bis_SR_register(LPM0_bits );

            return UCB3_I2C_ACK;
        case UCB1:
            UCB1I2CSA = slaveAddress; // Slave Address
            UCB1TXPtr = txData; // Assign Pointer
            UCB1TXLength = size; // Assign Length
            TimeoutCounter = 500;
            while((UCB1CTLW0 & UCTXSTP) && TimeoutCounter){
                TimeoutCounter--;
                delay(1);
            }
            if(!TimeoutCounter) return 0;
            UCB1TBCNT = size;
            UCB1CTLW0 |= UCTR + UCTXSTT; // Start Condition
            __bis_SR_register(LPM0_bits );

            return UCB1_I2C_ACK;
    }
	return 1;
}

unsigned char I2C::receive(unsigned char* rxData, unsigned int size, unsigned char slaveAddress){
    unsigned int TimeoutCounter;
    switch(CurrentBus){
        case UCB3:
            UCB3I2CSA = slaveAddress; // Slave Address
            UCB3RXPtr = rxData; // Assign Pointer
            UCB3RXLength = size; // Assign Length
            TimeoutCounter = 500;
            while((UCB3CTLW0 & UCTXSTP) && TimeoutCounter){
                TimeoutCounter--;
                delay(1);
            }

            if(!TimeoutCounter) return 0;
            UCB3TBCNT = size;
            UCB3CTLW0 &= ~UCTR; // Read
            UCB3CTLW0 |= UCTXSTT; // Start Condition
            __bis_SR_register(LPM0_bits );
            return UCB3_I2C_ACK;
        case UCB1:
            UCB1I2CSA = slaveAddress; // Slave Address
            UCB1RXPtr = rxData; // Assign Pointer
            UCB1RXLength = size; // Assign Length
            TimeoutCounter = 500;
            while((UCB1CTLW0 & UCTXSTP) && TimeoutCounter){
                TimeoutCounter--;
                delay(1);
            }
            if(!TimeoutCounter) return 0;
            UCB1TBCNT = size;
            UCB1CTLW0 &= ~UCTR; // Read
            UCB1CTLW0 |= UCTXSTT; // Start Condition
            __bis_SR_register(LPM0_bits );
            return UCB1_I2C_ACK;
	}
	return 1;
}

#pragma vector = EUSCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){
    switch(__even_in_range(UCB1IV,USCI_I2C_UCBIT9IFG)){
        case 0:                                 // Vector  0: No interrupts
            break;
        case 2:                                 // Vector  2: ALIFG
            break;
        case USCI_I2C_UCNACKIFG:                                // Vector  4: NACKIFG
            UCB1_I2C_ACK = false;
            //UCB1IFG &= ~UCNACKIFG;
            //UCB1CTLW0 |= UCTXSTP; // Stop condition //UCTXSTP
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        case 6:                                 // Vector  6: STTIFG
            break;
        case 8:                                 // Vector  8: STPIFG
            break;
        case USCI_I2C_UCRXIFG0:     //10                            Vector 10: RXIFG
            UCB1RXLength--; // decrease counter
            if(UCB1RXLength){
                *UCB1RXPtr = UCB1RXBUF; // data to buffer
                UCB1RXPtr++;
                /*if(UCB1RXLength == 1)
                    UCB1CTL1 |= UCTXSTP; // Stop condition*/
            }
            else{
                *UCB1RXPtr = UCB1RXBUF; // data to buffer
                //UCB1IFG &= ~UCRXIFG0;
                //UCB1CTLW0 |= UCTXSTP;
                UCB1_I2C_ACK = true;
                __bic_SR_register_on_exit(LPM0_bits);
            }
            break;
        case USCI_I2C_UCTXIFG0: //12                                // Vector 12: TXIFG
            if(UCB1TXLength){
                UCB1TXBUF = *UCB1TXPtr; // data to buffer
                *UCB1TXPtr++;
                UCB1TXLength--; // decrease counter
            }
            if(!UCB1TXLength){
                UCB1_I2C_ACK = true;
                //UCB1IFG &= ~UCTXIFG0;
                //UCB1CTLW0 |= UCTXSTP;
                __bic_SR_register_on_exit(LPM0_bits);
            }
            break;
        case USCI_I2C_UCBCNTIFG:            // Vector 26: BCNTIFG
            break;
        case USCI_I2C_UCCLTOIFG:											// Vector 34: Timeout Interrupt Flag
        	UCB1_I2C_ACK = false;
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        default: break;
    }
}

#pragma vector = EUSCI_B3_VECTOR
__interrupt void USCI_B3_ISR(void){
	switch(__even_in_range(UCB3IV,USCI_I2C_UCBIT9IFG)){
        case 0:                           		// Vector  0: No interrupts
            break;
        case 2:                          		// Vector  2: ALIFG
            break;
        case USCI_I2C_UCNACKIFG:                           		// Vector  4: NACKIFG
            UCB3_I2C_ACK = false;
            //UCB3IFG &= ~UCNACKIFG;
            //UCB3CTLW0 |= UCTXSTP; // Stop condition //UCTXSTP
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        case 6:   		                		// Vector  6: STTIFG
            break;
        case 8:                           		// Vector  8: STPIFG
            break;
        case USCI_I2C_UCRXIFG0:     //10                            Vector 10: RXIFG
            UCB3RXLength--; // decrease counter
            if(UCB3RXLength){
                *UCB3RXPtr = UCB3RXBUF; // data to buffer
                UCB3RXPtr++;
                /*if(UCB3RXLength == 1)
                    UCB3CTL1 |= UCTXSTP; // Stop condition*/
            }
            else{
                *UCB3RXPtr = UCB3RXBUF; // data to buffer
                //UCB3IFG &= ~UCRXIFG0;
                //UCB3CTLW0 |= UCTXSTP;
                UCB3_I2C_ACK = true;
                __bic_SR_register_on_exit(LPM0_bits);
            }
            break;
        case USCI_I2C_UCTXIFG0: //12                                // Vector 12: TXIFG
            if(UCB3TXLength){
                UCB3TXBUF = *UCB3TXPtr; // data to buffer
                *UCB3TXPtr++;
                UCB3TXLength--; // decrease counter
            }
            if(!UCB3TXLength){
                UCB3_I2C_ACK = true;
                //UCB3IFG &= ~UCTXIFG0;
                //UCB3CTLW0 |= UCTXSTP;
                __bic_SR_register_on_exit(LPM0_bits);
            }
            break;
        case USCI_I2C_UCBCNTIFG:            // Vector 26: BCNTIFG
            break;
        case USCI_I2C_UCCLTOIFG:											// Vector 34: Timeout Interrupt Flag
        	UCB3_I2C_ACK = false;
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        default:
        	break;
	}
}
} /* namespace MSP430 */
