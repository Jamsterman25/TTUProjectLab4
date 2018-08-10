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

/* Pins used exclusively in this version of the program:
 *
 * P5.0 MUX control 0
 * P5.1 MUX control 1
 *
 * P8.0 PB0 UP
 * P8.1 PB1 DOWN
 * P8.2 PB2 SELECT
 * P8.3 PB3 CANCEL
 *
 */
#include <msp430.h>
#include <string.h>

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
    char MAC[13];
    char NAME[17];
    char RSSI[3];
};
// All data is 1 byte larger than required to prevent overlap in pointer identification.

void CLOCK_SETUP();
void USB_UART_SETUP();
void GPIO_UART_SETUP();
void Send(char * input);
void delay(int sec);
void Inquiry();
void PB_SETUP();
void PowerOff();
void PowerOn();
void Pair();
void RePair();
void UnPair();

void lcd_reset();
void lcd_pos(char pos);
void lcd_setup();
void lcd_data(unsigned char dat);
void lcd_display_top(char *line);
void lcd_display_bottom(char * line);
void lcd_clear_top(char * line);
void lcd_clear_bottom(char * line);

struct BTDevice BTVector[10];

unsigned int DeviceConnected = 0x00;
unsigned int Device = 0; // Current device in vector
unsigned int mode = 0;   // Device mode
unsigned int pair = 0;
unsigned int MMmode = 0; // Main menu mode
unsigned int CANCEL = 0;
unsigned int DT = 0;  // Controls whether the top can be written to
unsigned int DB = 0;  // Controls whether the bottom can be written to

unsigned char RXED[1024]; // Received data
unsigned int RXI = 0;    // Received data increment

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    CLOCK_SETUP();     // Setup clock 8Mhz
    GPIO_UART_SETUP(); // Setup UART 9600 baud
    lcd_setup();       // Setup LCD screen on port 3
    PB_SETUP();        // Setup control push buttons

    P5DIR |= 0x03u;    // MUX Select pins for selection of BC127 Device

    __bis_SR_register(GIE);     // interrupts enabled

    PowerOff();

    while(1){
        switch(mode){
            case 0:
                if(!DT){
                    lcd_display_top("Main Menu:");
                    DT = 1;
                }
                switch(MMmode){
                case 0:
                    if(!DB){
                        lcd_display_bottom("Add New Device");
                        DB = 1;
                    }
                    break;
                case 1:
                    if(!DB){
                        lcd_display_bottom("Reconnect Device");
                        DB = 1;
                    }
                    break;
                case 2:
                    if(!DB){
                        lcd_display_bottom("Remove Devices");
                        DB = 1;
                    }
                    break;
                }
                break;
            case 1:
                Pair();
                break;
            case 2:
                RePair();
                break;
            case 3:
                lcd_display_top("Unpairing");
                lcd_display_bottom("");
                UnPair();
                lcd_display_top("Done");
                delay(5);
                DT = 0;
                DB = 0;
                mode = 0;
                break;
            default:
                mode = 0;
        }
        CANCEL = 0;
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

#pragma vector=PORT8_VECTOR
__interrupt void Port_8(void)
{
    delay(5);
    switch(P8IFG){
        case 0x01: // UP
            DB = 0; // Allow bottom writing
            switch(mode){
            case 0:
                if(MMmode == 0) MMmode = 2;
                else if(MMmode == 1) MMmode = 0;
                else if(MMmode == 2) MMmode = 1;
                break;
            case 1:
                if(Device == 0){
                    Device = 10;
                    while(!BTVector[Device].NAME[0]){
                        Device--;
                    }
                }
                else
                    Device--;
                break;
            default:
                break;
            }
            break;
        case 0x02: // DOWN
            DB = 0; // Allow bottom writing
            switch(mode){
                case 0:
                    if(MMmode == 0) MMmode = 1;
                    else if(MMmode == 1) MMmode = 2;
                    else if(MMmode == 2) MMmode = 0;
                    break;
                case 1:
                    if(Device == 9)
                        Device = 0;
                    else
                        Device++;
                    if(!BTVector[Device].NAME[0])
                        Device = 0;
                    break;
                default:
                    break;
                       }
            break;
        case 0x04: // SELECT
            switch(mode){
                case 0:
                    DT = 0;
                    if(MMmode == 0) mode = 1;
                    if(MMmode == 1) mode = 2;
                    if(MMmode == 2) mode = 3;
                    break;
                case 1:
                    pair = 1;
                    break;
                default:
                    break;
            }

        case 0x08: // CANCEL
            CANCEL = 1;
            break;
    }
    P8IFG = 0; // Clear the interrupt
}

void Inquiry(){
    RXI = 0;
    memset(RXED, '\0', 1024);
    Send("Inquiry 8\r");
    delay(110);
    unsigned int i=0;
    unsigned int new = 1;
    unsigned int BTVec = 0;
    for(i=8; i<RXI+1; i++){
        if(BTVec == 10) continue;
        struct BTDevice temp;
        memset(temp.NAME, '\0', 16);
        switch(RXED[i]){
            case 'I':
                i += 8;
                unsigned int iMAC = 0;
                for(iMAC=0; iMAC<12; iMAC++){
                    temp.MAC[iMAC] = RXED[i];
                    i++;
                }
                break;
            case '"':
                i++;
                unsigned int iNAME = 0;
                while(RXED[i] != '"'){
                    if(iNAME == 16){
                        i++;
                        continue;
                    }
                    temp.NAME[iNAME] = RXED[i];
                    iNAME++;
                    i++;
                }
                unsigned int iStruct = 0;
                for(iStruct = 0; iStruct < 10; iStruct++){
                    if(!strcmp(BTVector[iStruct].NAME,temp.NAME)){
                        new = 0;
                    }
                }
                if(new){
                    unsigned int iMAC = 0;
                    for(iMAC = 0; iMAC<12; iMAC++)
                        BTVector[BTVec].MAC[iMAC] = temp.MAC[iMAC];

                    unsigned int iNAME = 0;
                    while(temp.NAME[iNAME]){
                        BTVector[BTVec].NAME[iNAME] = temp.NAME[iNAME];
                        iNAME++;
                    }
                    i += 9;
                    BTVector[BTVec].RSSI[0] = RXED[i];
                    i++;
                    BTVector[BTVec].RSSI[1] = RXED[i];
                    i += 3;
                    BTVec++;
                    break;
                }
                else{
                    i += 13;
                    break;
                }
            default:
                break;
           }
       }
}

void Pair(){
    lcd_display_top("Searching...");
    lcd_display_bottom("");
    DB = 0;
    switch(DeviceConnected){
        case 0:
        case 2:
        case 4:
        case 6:
        case 8:
        case 10:
        case 12:
        case 14:
            P5OUT = 0x00; // 0000 0000 Select channel 0 (first BC127)
            //Send("power on\r");
            memset(BTVector, '\0', 320);
            delay(3);
            Inquiry();
            lcd_display_top("Select Device:");
            while(pair == 0){
                if(!DB){
                    lcd_display_bottom(BTVector[Device].NAME);
                    DB = 1;
                }
                if(CANCEL){
                    DB = 0;
                    mode = 0;
                    //Send("power off\r");
                    return;
                }
            }
            lcd_display_top("Pairing...");
            lcd_display_bottom("");
            Send("open ");
            Send(BTVector[Device].MAC);
            Send(" a2dp\r");
            delay(100);
            Send("music 10 play\r");
            delay(20);
            RXI = 0;
            memset(RXED, '\0', 100);
            Send("status\r");
            delay(4);
            if(RXED[63] == 'L')
                DeviceConnected |= 0x01;
            else{
                lcd_display_top("Connect Failed");
                delay(7);
            }
            delay(1);
            DT = 0;
            DB = 0;
            pair = 0;
            mode = 0;
            break;
        case 1:
        case 5:
        case 9:
        case 13:
            P5OUT = 0x01; // 0000 0001 Select channel 1 (second BC127)
            //Send("power on\r");
            memset(BTVector, '\0', 320);
            delay(3);
            Inquiry();
            lcd_display_top("Select Device:");
            while(pair == 0){
                if(!DB){
                    lcd_display_bottom(BTVector[Device].NAME);
                    DB = 1;
                }
                if(CANCEL){
                    DB = 0;
                    mode = 0;
                    //Send("power off\r");
                    return;
                }
            }
            lcd_display_top("Pairing...");
            lcd_display_bottom("");
            Send("open ");
            Send(BTVector[Device].MAC);
            Send(" a2dp\r");
            delay(100);
            Send("music 10 play\r");
            delay(20);
            RXI = 0;
            memset(RXED, '\0', 100);
            Send("status\r");
            delay(4);
            if(RXED[63] == 'L')
                DeviceConnected |= 0x02;
            else{
                lcd_display_top("Connect Failed");
                delay(7);
            }
            delay(1);
            DT = 0;
            DB = 0;
            pair = 0;
            mode = 0;
            break;
        case 3:
        case 11:
            P5OUT = 0x02; // 0000 0010 Select channel 2 (third BC127)
            //Send("power on\r");
            memset(BTVector, '\0', 320);
            delay(3);
            Inquiry();
            lcd_display_top("Select Device:");
            while(pair == 0){
                if(!DB){
                    lcd_display_bottom(BTVector[Device].NAME);
                    DB = 1;
                }
                if(CANCEL){
                    DB = 0;
                    mode = 0;
                    //Send("power off\r");
                    return;
                }
            }
            lcd_display_top("Pairing...");
            lcd_display_bottom("");
            Send("open ");
            Send(BTVector[Device].MAC);
            Send(" a2dp\r");
            delay(100);
            Send("music 10 play\r");
            delay(20);
            RXI = 0;
            memset(RXED, '\0', 100);
            Send("status\r");
            delay(4);
            if(RXED[63] == 'L')
                DeviceConnected |= 0x04;
            else{
                lcd_display_top("Connect Failed");
                delay(7);
            }
            delay(1);
            DT = 0;
            DB = 0;
            pair = 0;
            mode = 0;
            break;
        case 7:
            P5OUT = 0x03; // 0000 0011 Select channel 3 (fourth BC127)
            //Send("power on\r");
            memset(BTVector, '\0', 320);
            delay(3);
            Inquiry();
            lcd_display_top("Select Device:");
            while(pair == 0){
                if(!DB){
                    lcd_display_bottom(BTVector[Device].NAME);
                    DB = 1;
                }
                if(CANCEL){
                    DB = 0;
                    mode = 0;
                    //Send("power off\r");
                    return;
                }
            }
            lcd_display_top("Pairing...");
            lcd_display_bottom("");
            Send("open ");
            Send(BTVector[Device].MAC);
            Send(" a2dp\r");
            delay(100);
            Send("music 10 play\r");
            delay(20);
            RXI = 0;
            memset(RXED, '\0', 100);
            Send("status\r");
            delay(4);
            if(RXED[63] == 'L')
                DeviceConnected |= 0x01;
            else{
                lcd_display_top("Connect Failed");
                delay(7);
            }
            delay(1);
            DT = 0;
            DB = 0;
            pair = 0;
            mode = 0;
            break;
        case 15:
            lcd_display_top("Maximum Devices!");
            lcd_display_bottom("");
            delay(7);
            mode = 0;
            DT = 0;
            DB = 0;
            break;
    }
}

void RePair(){
    lcd_display_top("Pairing...");
    lcd_display_bottom("");
    //PowerOn();
    delay(50);

    P5OUT = 0x00; // 0000 0000 Select channel 0 (first BC127)
    RXI = 0;
    memset(RXED, '\0', 100);
    Send("status\r");
    delay(4);
    if(RXED[63] == 'L'){
        Send("music 10 play\r");
        delay(20);
        DeviceConnected |= 0x01;
    }
    delay(1);

    P5OUT = 0x01; // 0000 0001 Select channel 1 (second BC127)
    RXI = 0;
    memset(RXED, '\0', 100);
    Send("status\r");
    delay(4);
    if(RXED[63] == 'L'){
        Send("music 10 play\r");
        delay(20);
        DeviceConnected |= 0x02;
    }
    delay(1);

    P5OUT = 0x02; // 0000 0010 Select channel 2 (third BC127)
    RXI = 0;
    memset(RXED, '\0', 100);
    Send("status\r");
    delay(4);
    if(RXED[63] == 'L'){
        Send("music 10 play\r");
        delay(20);
        DeviceConnected |= 0x04;
    }
    delay(1);

    P5OUT = 0x03; // 0000 0011 Select channel 3 (fourth BC127)
    RXI = 0;
    memset(RXED, '\0', 100);
    Send("status\r");
    delay(4);
    if(RXED[63] == 'L'){
        Send("music 10 play\r");
        delay(20);
        DeviceConnected |= 0x08;
    }
    delay(1);

    PowerOff();
    DT = 0;
    DB = 0;
    mode = 0;
}

void UnPair(){
    P5OUT = 0x00; // 0000 0000 Select channel 0 (first BC127)
    Send("unpair\r");
    delay(1);
    P5OUT = 0x01; // 0000 0001 Select channel 1 (second BC127)
    Send("unpair\r");
    delay(1);
    P5OUT = 0x02; // 0000 0010 Select channel 2 (third BC127)
    Send("unpair\r");
    delay(1);
    P5OUT = 0x03; // 0000 0011 Select channel 3 (fourth BC127)
    Send("unpair\r");
    delay(1);
    DeviceConnected = 0;
}

void Send(char * input){
    int i=0;
    while (input[i]){
        while(!(UCA3IFG&UCTXIFG)); // Wait
        UCA3TXBUF = input[i];     // Send
        i++;
    }
}

void PowerOn(){
    P5OUT = 0x00; // 0000 0000 Select channel 0 (first BC127)
    Send("power on\r");
    delay(1);
    P5OUT = 0x01; // 0000 0001 Select channel 1 (second BC127)
    Send("power on\r");
    delay(1);
    P5OUT = 0x02; // 0000 0010 Select channel 2 (third BC127)
    Send("power on\r");
    delay(1);
    P5OUT = 0x03; // 0000 0011 Select channel 3 (fourth BC127)
    Send("power on\r");
    delay(1);
}

void PowerOff(){
    P5OUT = 0x00; // 0000 0000 Select channel 0 (first BC127)
    Send("discoverable off\r");
    delay(1);
    //Send("power off\r");
    delay(1);
    P5OUT = 0x01; // 0000 0001 Select channel 1 (second BC127)
    Send("discoverable off\r");
    delay(1);
    //Send("power off\r");
    delay(1);
    P5OUT = 0x02; // 0000 0010 Select channel 2 (third BC127)
    Send("discoverable off\r");
    delay(1);
    //Send("power off\r");
    delay(1);
    P5OUT = 0x03; // 0000 0011 Select channel 3 (fourth BC127)
    Send("discoverable off\r");
    delay(1);
    //Send("power off\r");
    delay(1);
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
        __delay_cycles(800000); // Delay .1s
        sec--;
    }
}

void PB_SETUP(){
    P8DIR &= ~0x0F; // 0000 1111 set PB 0-3 to input mode
    P8IE = 0x0F;    // 0000 1111 Allow interrupts PB 0-3
    P8IES = 0x0F;   // Select low edge for PB interrupt
    P8REN = 0x0F;   // Enable internal pullup resistor enable
    P8OUT = 0x0f;   // Pullup resistor activate
    P8IFG = 0;      // Clear interrupts
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
    lcd_clear_top("                ");
    lcd_pos(top);
    while (*line)
        lcd_data(*line++);
}

void lcd_clear_top(char * line)
{
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
