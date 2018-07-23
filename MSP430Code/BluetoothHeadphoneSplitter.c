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


// ------------------------------------------------------------------------------------
// SECONDARY ADAPTED CODE:
// Author: Troy Davis
// Company: Texas Tech University
// Department: Pulsed Power and Power Electronics Lab (P3E)
// Status: Incomplete
//
// Date: 6/14/2017
// Assembler: Code Composer Studio 7.0.0
// Target Board: TI MSP430G2553 LaunchPad
//
// Flash Used: 2Kb
// RAM Used: .1Kb
// I/O Pins Used: 14
//
// Description: Show different things depending on the button pressed, etc, etc
//      You have to figure out which pin is which on the number pad.
//      You can test this with 3.3V and a multimeter. Just hook up 3.3 to a pin
//      and test each other pins for 3.3 when a button is pressed.
//
// I/O Assignments: Port 1 will mostly be used by the 16x2 LCD screen and some extras.
//
//      16x2 Screen + Extra:
//          P3.0: (output) LCD D4
//          P3.1: (output) LCD D5
//          P3.2: (output) LCD D6
//          P3.3: (output) LCD D7
//          P3.4: (output) LCD E
//          P3.5: (output) LCD RS
//          P3.6: (TBD)
//          P3.7: (TBD)
//

//
//      Extra Connections:
//          LCD VDD - 5V (TP1 on MSP430 LaunchPad)
//          LCD VSS - GND
//          LCD RW  - GND
//          LCD V0  - 3V through pot (adjusts contrast)
//          LCD K   - GND
//          LCD A   - 5V through pot (adjusts brightness)
//
//      Needed Connections:
//          Thermistor?   // external temp
//          PWM out?
//          Voltage?
//
// Comments: In order to do in-circuit programming of the MSP if the chip is on a breadboard,
//           one must follow these schematics: http://www.ti.com/lit/ug/slau318g/slau318g.pdf
//
// -------------------------------------------------------------------------------------
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

#define lcd_port        P3OUT
#define lcd_port_dir    P3DIR

#define LCD_EN      BIT4
#define LCD_RS      BIT5
#define top         0x80 // top line of LCD starts here and goes to 8F
#define bottom      0xC0 // bottom line of LCD starts here and goes to CF

#define a           0
#define b           1
#define c           2
#define d           3

struct BTDevice{
    char MAC[10];
    int RSSI;
    char * NAME;
};

void CLOCK_SETUP();
void USB_UART_SETUP();
void GPIO_UART_SETUP();
void Send(char * input);
void delay(int sec);
void Inquiry();

void lcd_reset();
void lcd_pos(char pos);
void lcd_setup();
void lcd_data(unsigned char dat);
void lcd_display_top(char *line);
void lcd_display_bottom(char * line);
void lcd_clear_top(char * line);
void lcd_clear_bottom(char * line);

struct BTDevice BTVector[10];

unsigned char RXED[1024]; // Received data
unsigned int RXI = 0;    // Received data increment

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    CLOCK_SETUP();
    GPIO_UART_SETUP();

    __bis_SR_register(GIE);     // Enter LPM3, interrupts enabled

    lcd_setup();

    while(1){
        lcd_display_top("Select Device:");
        lcd_display_bottom("Jamoji JK");
        delay(3);
        lcd_display_top("Select Device:");
        lcd_display_bottom("BEARDEDBLUE CLASSIC");
        delay(1);
    }

}

#pragma vector=EUSCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)

{
    switch(__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            RXED[RXI] = UCA3RXBUF;
            if(RXI > 1023)
                RXI = 0;
            RXI++;
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

void Inquiry(){
    RXI = 0;
    Send("Inquiry 5\r");
    delay(6);
    unsigned int i = 0;
    for(i=0; i=RXI; i++){

    }
}

void Send(char * input){
    int i=0;
    while (input[i]){
        while(!(UCA3IFG&UCTXIFG)); // Wait
        UCA3TXBUF = input[i];     // Send
        i++;
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

void delay(int sec){
    while (sec>0){
        __delay_cycles(8000000); // Delay 1s
        sec--;
    }
}

void lcd_reset()
{
    lcd_port_dir = 0xff;
    lcd_port = 0xff;
    __delay_cycles(160000);
    lcd_port = 0x03+LCD_EN;
    lcd_port = 0x03;
    __delay_cycles(80000);
    lcd_port = 0x03+LCD_EN;
    lcd_port = 0x03;
    __delay_cycles(8000);
    lcd_port = 0x03+LCD_EN;
    lcd_port = 0x03;
    __delay_cycles(8000);
    lcd_port = 0x02+LCD_EN;
    lcd_port = 0x02;
    __delay_cycles(8000);
}

void lcd_pos (char pos)
{
    lcd_port = ((pos >> 4) & 0x0F)|LCD_EN;
    lcd_port = ((pos >> 4) & 0x0F);

    lcd_port = (pos & 0x0F)|LCD_EN;
    lcd_port = (pos & 0x0F);

    __delay_cycles(32000);
}

void lcd_setup()
{
    lcd_reset();         // Call LCD reset
    lcd_pos(0x28);       // 4-bit mode - 2 line - 5x7 font.
    lcd_pos(0x0C);       // Display no cursor - no blink.
    lcd_pos(0x06);       // Automatic Increment - No Display shift.
    lcd_pos(0x80);       // Address DDRAM with 0 offset 80h.
    lcd_pos(0x01);       // Clear screen
}


void lcd_data (unsigned char dat)
{
    lcd_port = (((dat >> 4) & 0x0F)|LCD_EN|LCD_RS);
    lcd_port = (((dat >> 4) & 0x0F)|LCD_RS);

    lcd_port = ((dat & 0x0F)|LCD_EN|LCD_RS);
    lcd_port = ((dat & 0x0F)|LCD_RS);

    __delay_cycles(3200);
}

void lcd_display_top(char * line)
{
    lcd_pos(top);
    while (*line)
        lcd_data(*line++);
}

void lcd_clear_top(char * line)
{
    lcd_clear_top("                ");
    lcd_pos(top);
    while (*line)
        lcd_data(*line++);
}

void lcd_display_bottom(char * line)
{
    lcd_clear_bottom("                ");
    lcd_pos(bottom);
    while (*line)
        lcd_data(*line++);
}

void lcd_clear_bottom(char * line)
{
    lcd_pos(bottom);
    while (*line)
        lcd_data(*line++);
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
