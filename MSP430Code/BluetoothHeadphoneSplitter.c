/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430FR5x9x Demo - eUSCI_A3 UART echo at 9600 baud using BRCLK = 8MHz
//
//  Description: This demo echoes back characters received via a PC serial port.
//  SMCLK/ DCO is used as a clock source and the device is put in LPM3
//  The auto-clock enable feature is used by the eUSCI and SMCLK is turned off
//  when the UART is idle and turned on when a receive edge is detected.
//  Note that level shifter hardware is needed to shift between RS232 and MSP
//  voltage levels.
//
//  The example code shows proper initialization of registers
//  and interrupts to receive and transmit data.
//  To test code in LPM3, disconnect the debugger.
//
//  ACLK = VLO, MCLK =  DCO = SMCLK = 8MHz
//
//                MSP430FR5994
//             -----------------
//       RST -|     P6.0/UCA3TXD|----> PC (echo)
//            |                 |
//            |                 |
//            |     P6.1/UCA3RXD|<---- PC
//            |                 |
//
//   William Goh
//   Texas Instruments Inc.
//   October 2015
//   Built with IAR Embedded Workbench V6.30 & Code Composer Studio V6.1
//******************************************************************************
/* Adaptation for Texas Tech University ECE Project Lab 4
 *
 * Tony Kahumbu & Jeremiah McCutcheon Copyright (c) 2018
 *
 *  This project utilizes UART communication to control 4 Bluetooth modules
 * each module can connect to a device and should be completly configured using
 * this code.
 *
 *  NOTE: If switching to USB-UART (setup function included) you must adjust the interrupt vector from A3 to A0
 *
 *
 */
#include <msp430.h>
voic CLOCK_SETUP();
void USB_UART_SETUP();
void GPIO_UART_SETUP();


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog


    */
    //GPIO-UART


    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;



    /* USB-UART

    */
    // GPIO-UART



    __bis_SR_register(LPM3_bits | GIE);     // Enter LPM3, interrupts enabled
    __no_operation();                       // For debugger
}

#pragma vector=EUSCI_A0_VECTOR
__interrupt void USCI_A3_ISR(void)

{
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            while(!(UCA0IFG&UCTXIFG));
            UCA0TXBUF = UCA0RXBUF;
            __no_operation();
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

void CLOCK_SETUP(){
    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL0_H = 0;                           // Lock CS registers
}

void USB_UART_SETUP(){
    // Configure GPIO
    P2SEL1 |= (BIT0 | BIT1); // USCI_A0 UART operation
    P2SEL0 &= ~(BIT0 | BIT1);

    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 21-4: UCBRSx = 0x04
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA0BRW = 52;                           // 8000000/16/9600
    UCA0MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
    UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
    UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
}

void GPIO_UART_SETUP(){
    // Configure GPIO
    P6SEL1 &= ~(BIT0 | BIT1);
    P6SEL0 |= (BIT0 | BIT1);                // USCI_A3 UART operation

    // Configure USCI_A0 for UART mode
    UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 21-4: UCBRSx = 0x04
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA3BRW = 52;                           // 8000000/16/9600
    UCA3MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
    UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
    UCA3IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
}






























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
/*
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
unsigned char rxArray[256];
unsigned int rxIndex = 0;
static char data;
char* temp = " ";

void main(void) {
     WDTCTL = WDTPW | WDTHOLD;    // Stop watchdog timer

    uartSetup();

    rxIndex = 0;

    P2DIR |= 0x03u;

    _BIS_SR(GIE);

    P2OUT = 0x00; // Set to ch0

    while(1){
        Send("get name\r\n");
        delay(2);
        //delay(2);
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
        UCA0BR0 = 109;                  // Set baud rate: 9600
        UCA0BR1 = 0;
        UCA0MCTL = UCBRS_0;             // UART Modulation Control setup
        UCA0CTL1 &= ~UCSWRST;           // Enable UART
        IE2 |= UCA0RXIE;                // Enable USCI_A0 RX interrupt
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
    rxArray[rxIndex] = data; // Copy element to array
    rxIndex++;

}
*/
