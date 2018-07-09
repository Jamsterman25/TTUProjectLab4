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
#define UART_BR 109     // UART Baud Rate = 9600 baud at 1 MHz

// Function prototypes
void uartSetup();
void timerSetup();
void Send(char * tx_data);
void delay(unsigned int sec);

// Variables
int BYTES_TO_SEND = 16;
unsigned char txArray[16];
unsigned char txIndex = 0;
static char data;
char* temp = " ";

void main(void) {
     WDTCTL = WDTPW | WDTHOLD;    // Stop watchdog timer

    uartSetup();

    txIndex = 0;

    P2DIR |= 0x03u;

    _BIS_SR(GIE);

    P2OUT = 0x00; // clear channels

    while(1){
        P2OUT = 0x00; // ch0
        Send("\r\nChannel 0 10s echo: ");
        delay(10);

        P2OUT = 0x01; // ch1
        Send("\r\nChannel 1 10s echo: ");
        delay(10);

        P2OUT = 0x02; // ch2
        Send("\r\nChannel 2 10s echo: ");
        delay(10);

        P2OUT = 0x03; // ch3
        Send("\r\nChannel 3 10s echo: ");
        delay(10);
    }

}


void uartSetup()
{
        DCOCTL=0;
        BCSCTL1 = CALBC1_1MHZ; // Set DCO
        DCOCTL = CALDCO_1MHZ;
    // Set up UART
        UCA0CTL1 |= UCSWRST;            // Disable UART
        P1SEL |= UART_TX + UART_RX;     // Set port pins to UART mode
        P1SEL2 |= UART_TX + UART_RX;    // Set port pins to UART mode
        P1DIR |= UART_TX;               // Set TX pin as output
        UCA0CTL1 |= UCSSEL_2;           // Set UART to use SMCLK (1 MHz)
        UCA0BR0 = 109;       // Set baud rate
        UCA0BR1 = 0;
        UCA0MCTL = UCBRS_0; // UART Modulation Control setup
        UCA0CTL1 &= ~UCSWRST;           // Enable UART
        IE2 |= UCA0RXIE;                  // Enable USCI_A0 RX interrupt
}

void delay(unsigned int sec){
    while(sec>0){
        __delay_cycles(1000000);
        sec--;
    }
}

void Send(char * tx_data) // Send Out TX
{
    unsigned int i=0;

    while(tx_data[i]) // Increment
    {
        while ((UCA0STAT & UCBUSY)); // Wait
        UCA0TXBUF = tx_data[i]; // Send out element
        i++;
    }
}

#pragma vector=USCIAB0RX_VECTOR  // Receive UART interrupt
__interrupt void USCI0RX_ISR(void)
{
    data = UCA0RXBUF;

    while ((UCA0STAT & UCBUSY)); // Wait
            UCA0TXBUF = data; // Send out element
    //temp = data;
    //Send(temp);
}
