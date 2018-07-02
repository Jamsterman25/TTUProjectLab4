/* 
  ECE 3362 project 6 UART Communication. This project will recieve an interrupt
    from a keyboard input and return the pressed key to the console.


   Target: TI LaunchPad development board with MSP430G2553 device with the
    a speaker with DC blocking capacitor installed at P2.3 with common side 
    of speaker grounded

       Date:           May 6, 2017
       Last Revision:  1.0

       Written by:     Jeremiah McCutcheon
       Adapted from:   Dr. Michael Helm, Simple UART example. 
       & Troy Davis: UART Interrupt Example.
       Assembler/IDE:  IAR Embedded Workbench 6.5

        HW I/O assignments:
        P1.0    LED1    (Active HIGH)RED
        P1.1    not used   UART RX serial data (input to MSP430)
        P1.2    not used   UART TX serial data (output from MSP430)
        P1.3    not used PushButton (Active LOW) (internal Pullup Enabled)
        P1.4    not used
        P1.5    not used
        P1.6    not used LED2    (Active HIGH)GREEN
        P1.7    not used
 
        P2.0    not used
        P2.1    not used
        P2.2    not used
        P2.3    not used
        P2.4    not used
        P2.5    not used
        P2.6    not used
        P2.7    not used



*/

// Device Library
#include <msp430.h>

// Program Constants

// UART
#define UART_RX 0x02    // P1.1
#define UART_TX 0x04    // P1.2
#define UART_BR 104     // UART Baud Rate = 9600 baud at 1 MHz

// Function prototypes
void uartSetup();
void timerSetup();

// Variables
int BYTES_TO_SEND = 16;
unsigned char txArray[16];
unsigned char txIndex = 0;
static char data;

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;    // Stop watchdog timer

    uartSetup();

    txIndex = 0;

    _BIS_SR(GIE);
    
    txArray[0] = 'Y';
    txArray[1] = 'O';
    txArray[2] = 'U';
    txArray[3] = ' ';
    txArray[4] = 'P';
    txArray[5] = 'R';
    txArray[6] = 'E';
    txArray[7] = 'S';
    txArray[8] = 'S';
    txArray[9] = 'E';
    txArray[10] = 'D';
    txArray[11] = ' ';
    txArray[12] = '"';
    txArray[13] = ' ';
    txArray[14] = '"';
    txArray[15] = '\n';

    while(1);

}


void uartSetup()
{
    // Set up UART
        UCA0CTL1 |= UCSWRST;            // Disable UART
        P1SEL |= UART_TX + UART_RX;     // Set port pins to UART mode
        P1SEL2 |= UART_TX + UART_RX;    // Set port pins to UART mode
        P1DIR |= UART_TX;               // Set TX pin as output
        UCA0CTL1 |= UCSSEL_2;           // Set UART to use SMCLK (1 MHz)
        UCA0BR0 = UART_BR & 0xFF;       // Set baud rate
        UCA0BR1 = (UART_BR >> 8) & 0xFF;
        UCA0MCTL = (UCBRF_0 << 4) + (UCBRS_1 << 1); // UART Modulation Control setup
        UCA0CTL1 &= ~UCSWRST;           // Enable UART
        IE2 |= UCA0RXIE;                  // Enable USCI_A0 RX interrupt
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void UART_TX_ISR()
{
    UCA0TXBUF = txArray[txIndex];
    txIndex++;

    if(txIndex == BYTES_TO_SEND)
    {
        IE2 &= ~UCA0TXIE;
    }

    return;
}


#pragma vector=USCIAB0RX_VECTOR  // Receive UART interrupt
__interrupt void USCI0RX_ISR(void)
{
    data = UCA0RXBUF;

    txArray[13] = data;
    txIndex = 1;
    UCA0TXBUF = txArray[0];
    IE2 |= UCA0TXIE;
      
}
