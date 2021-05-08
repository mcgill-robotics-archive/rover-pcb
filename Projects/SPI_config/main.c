#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"

int main(void)
{

    // SYSCTL->RCC != (0x2<<4);
    /** 15.4 Initialization and Configuration (PDF titled "Tiva TM4C123GH6PM Microcontroller") "**/
    // To enable and initialize the SSI, the following steps are necessary:

    //  1. Enable the SSI module using the RCGCSSI register (see page 346).
    //  SYSCTL is just a pointer name
    //  SSI module 2 is being used so, because it's easier to manipulate by left shifting..?
    //  0100 for the RCGCSSI LSBs
    SYSCTL->RCGCSSI = (1<<2);

    //  2. Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 340).
    //  To find out which GPIO port to enable, refer to Table 23-5 on page 1351.
    //  Port B on the TM4C corresponds to 8 pins so, depending on which pins you have your serial device physically connected to, you can choose ports A-F
    SYSCTL->RCGCGPIO = (1<<1);

    //  3. Set the CPI AFSEL bits for the appropriate pins (see page 671).
    //  To determine which GPIOs to configure, see Table 23-4 on page 1344.
    GPIOB->AFSEL |= (1<<4)|(1<<6)|(1<<7)|(1<<5);
    //GPIOB->AFSEL &= ~(1<<5);


    /** Questions **/
    //  1. What is SYSCTL, is it a register?
    //  2. Clarify whether we can test our code without another peripheral serial device, current is port specific
    //  3. Is GPIOB a keyword as well? How does the AFSEL know to enable PB4-7 rather than say, PBA4-7? Is it implied somehow by the line above (step 2)?
    //  4.

	return 0;
}
