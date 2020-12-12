/*
 * Slave_I2C.cpp
 *
 *  Created on: 16 Mar 2018
 *      Author: bugra
 */

#include <Slave_I2C.h>

namespace MSP430{

    bool UCB2_I2C_ACK = false;
    unsigned char* UCB2TXPtr;
    unsigned int UCB2TXLength;
    unsigned char* UCB2RXPtr;
    unsigned int UCB2RXLength;

    Slave_I2C::Slave_I2C(Slave_BUS UCxx){
        CurrentBus = UCxx;
    	  switch(CurrentBus){
    	       case UCB2:
    	           P7SEL0 |= BIT0 | BIT1;
    	           P7SEL1 &= ~(BIT0 | BIT1);

    	           UCB2CTLW0 = UCSWRST;
    	           UCB2CTLW0 |= UCMODE_3 | UCSSEL__SMCLK;
    	           UCB2CTLW1 |= 0x40;
    	           UCB2BRW = 0x8;
    	           UCB2CTLW1 |= UCASTP_2;
    	           UCB2CTLW0 &= ~UCSWRST;
    	           UCB2IE |= UCNACKIE + UCALIE + UCSTPIE + UCSTTIE + UCTXIE0 + UCRXIE0 | UCBCNTIE | USCI_I2C_UCCLTOIFG;
    	           break;
        }
    }

    unsigned char Slave_I2C::transmit(unsigned char* txData, unsigned int size, unsigned char ownAddress){
        switch(CurrentBus){
            case UCB2:
                UCB2I2COA0 = ownAddress | UCOAEN;
                UCB2TXPtr = txData; // Assign Pointer
                UCB2TXLength = size; // Assign Length
                unsigned int TimeoutCounter = 500;
                while((UCB2CTLW0 & UCTXSTP) && TimeoutCounter){
                    TimeoutCounter--;
                    //delay(1);
                }
                if(!TimeoutCounter) return 0;
                UCB2TBCNT = size;
                UCB2CTLW0 |= UCTR + UCTXSTT; // Start Condition
                __bis_SR_register(LPM0_bits );
                return UCB2_I2C_ACK;
        }
    	  return 1;
    }

    unsigned char Slave_I2C::receive(unsigned char* rxData, unsigned int size, unsigned char ownAddress){
        switch(CurrentBus){
            case UCB2:
                UCB2I2COA0 = ownAddress | UCOAEN; // Own Address
                UCB2RXPtr = rxData; // Assign Pointer
                UCB2RXLength = size; // Assign Length
                unsigned int TimeoutCounter = 500;
                while((UCB2CTLW0 & UCTXSTP) && TimeoutCounter){
                    TimeoutCounter--;
                    //delay(1);
                }
                if(!TimeoutCounter) return 0;
                UCB2TBCNT = size;
                UCB2CTLW0 &= ~UCTR; // Read
                UCB2CTLW0 |= UCTXSTT; // Start Condition
                __bis_SR_register(LPM0_bits );
                return UCB2_I2C_ACK;
    	}
    	return 1;
    }

    #pragma vector = EUSCI_B2_VECTOR
    __interrupt void USCI_B2_ISR(void){
    	switch(__even_in_range(UCB2IV,USCI_I2C_UCBIT9IFG)){
            case 0:                           		                              // Vector  0: No interrupts
                break;
            case 2:                          	                                	// Vector  2: ALIFG
                break;
            case USCI_I2C_UCNACKIFG:                                        		// Vector  4: NACKIFG
                UCB2_I2C_ACK = false;
                __bic_SR_register_on_exit(LPM0_bits);
                break;
            case 6:   		                	                                   	// Vector  6: STTIFG
                break;
            case 8:                           		                              // Vector  8: STPIFG
                break;
            case USCI_I2C_UCRXIFG0:                                             // Vector 10: RXIFG
                UCB2RXLength--; // decrease counter
                if(UCB2RXLength){
                    *UCB2RXPtr = UCB2RXBUF; // data to buffer
                    UCB2RXPtr++;
                }
                else{
                    *UCB2RXPtr = UCB2RXBUF; // data to buffer
                    UCB2_I2C_ACK = true;
                    __bic_SR_register_on_exit(LPM0_bits);
                }
                break;
            case USCI_I2C_UCTXIFG0:                                             // Vector 12: TXIFG
                if(UCB2TXLength){
                    UCB2TXBUF = *UCB2TXPtr; // data to buffer
                    *UCB2TXPtr++;
                    UCB2TXLength--; // decrease counter
                }
                if(!UCB2TXLength){
                    UCB2_I2C_ACK = true;
                    __bic_SR_register_on_exit(LPM0_bits);
                }
                break;
            case USCI_I2C_UCBCNTIFG:                                            // Vector 26: BCNTIFG
                break;
            case USCI_I2C_UCCLTOIFG:											// Vector 34: Timeout Interrupt Flag
            	UCB2_I2C_ACK = false;
            	__bic_SR_register_on_exit(LPM0_bits);
            	break;
            default:
            	break;
    	}
    }

} /* namespace MSP430 */
