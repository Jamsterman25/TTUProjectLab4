// ------------------------------------------------------------------------------------
//
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
// I/O Assignments: Port 1 will mostly be used by the 16x2 LCD screen and some extras. Port 2 will be
//                  used completely by the 4x4 Number Pad. Additional connections are also needed to
//                  power the LCD screen.
//
//      16x2 Screen + Extra:
//          P1.0: (output) LCD D4
//          P1.1: (output) LCD D5
//          P1.2: (output) LCD D6
//          P1.3: (output) LCD D7
//          P1.4: (output) LCD E
//          P1.5: (output) LCD RS
//          P1.6: (TBD)
//          P1.7: (TBD)
//
//      4x4 Number Pad:
//          P2.0: (output) Number Pad Column 1
//          P2.1: (output) Number Pad Column 2
//          P2.2: (output) Number Pad Column 3
//          P2.3: (output) Number Pad Column 4
//          P2.4: (input)  Number Pad Row 1
//          P2.5: (input)  Number Pad Row 2
//          P2.6: (input)  Number Pad Row 3 (XIN)
//          P2.7: (input)  Number Pad Row 4 (XOUT)
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

#include <msp430.h>

#define lcd_port        P1OUT
#define lcd_port_dir    P1DIR

#define LCD_EN      BIT4
#define LCD_RS      BIT5
#define top         0x80 // top line of LCD starts here and goes to 8F
#define bottom      0xC0 // bottom line of LCD starts here and goes to CF

#define a           0
#define b           1
#define c           2
#define d           3

long    temp;
long    DegF;
long    DegC;

int mode = 0;

unsigned char symbol = 'F';  //default number pad selection (impossible)

void keypad_check_row1();
void keypad_check_row2();
void keypad_check_row3();
void keypad_check_row4();
void keypad_check();

void lcd_reset();
void lcd_pos(char pos);
void lcd_setup();
void lcd_data(unsigned char dat);
void lcd_display_top(char *line);
void lcd_display_tempC_bottom();
void lcd_display_tempF_bottom();

void adc_setup();
void adc_convert_temp();

void timer0_setup();

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    P2SEL &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);    // select io
    P2DIR |= (BIT0 + BIT1 + BIT2 + BIT3);   // tell output
    P2DIR &= ~(BIT4 + BIT5 + BIT6 + BIT7);  // tell input
    P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);  // Put output to low

    // P1 In and Outs are TBD
    // The LCD is declared as it is used

    lcd_setup();
    adc_setup();
    timer0_setup();

    while(1) {

        keypad_check();
        //lcd_display_top("Start Here:");

        if (mode == 9){
            lcd_display_top("Temperature(F):  ");
           // lcd_display_tempF_bottom();
            lcd_pos(0xCf);
            lcd_data(55);
        }
        else if (mode == 7){
            lcd_display_top("Temperature(C):  ");
           // lcd_display_tempC_bottom();
            lcd_pos(0xCf);
            lcd_data(56);
        }
        else if (mode == 3){
            lcd_display_top("Voltage:         ");
            lcd_pos(0xCf);
            lcd_data(42);
        }
        else if (mode == 1){
            lcd_display_top("Frequency:       ");
            lcd_pos(0xCf);
            lcd_data(48);
        }

    }
}

void keypad_check(){

    keypad_check_row1();
    keypad_check_row2();
    keypad_check_row3();
    keypad_check_row4();
}




void lcd_reset()
{
    lcd_port_dir = 0xff;
    lcd_port = 0xff;
    __delay_cycles(20000);
    lcd_port = 0x03+LCD_EN;
    lcd_port = 0x03;
    __delay_cycles(10000);
    lcd_port = 0x03+LCD_EN;
    lcd_port = 0x03;
    __delay_cycles(1000);
    lcd_port = 0x03+LCD_EN;
    lcd_port = 0x03;
    __delay_cycles(1000);
    lcd_port = 0x02+LCD_EN;
    lcd_port = 0x02;
    __delay_cycles(1000);
}

void lcd_pos (char pos)
{
    lcd_port = ((pos >> 4) & 0x0F)|LCD_EN;
    lcd_port = ((pos >> 4) & 0x0F);

    lcd_port = (pos & 0x0F)|LCD_EN;
    lcd_port = (pos & 0x0F);

    __delay_cycles(4000);
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

    __delay_cycles(400);
}

void lcd_display_top(char *line)
{
    lcd_pos(top);
    while (*line)
        lcd_data(*line++);
}

void lcd_display_tempC_bottom()
{
    adc_convert_temp();

    int seven = 32;
    char eight = (DegC / 10 % 10) + 48;    // might change these to chars
    int nine = (DegC / 1 % 10) + 48;
    int eleven = (DegC / 100 % 10) + 48;
    int twelve = (DegC / 1000 % 10) + 48;
    int thirteen = (DegC / 10000 % 10) + 48;

    lcd_pos(0xCf);
    lcd_data(67);   // display "C" on 15

    lcd_pos(0xCe);
    lcd_data(32);   // display " " on 14

    lcd_pos(0xCd);
    lcd_data(thirteen);   // display on 13

    lcd_pos(0xCc);
    lcd_data(twelve);   // display on 12

    lcd_pos(0xCb);
    lcd_data(eleven);   // display on 11

    lcd_pos(0xCa);
    lcd_data(46);   // display "." on 10

    lcd_pos(0xC9);
    lcd_data(nine);   // display on 9

    lcd_pos(0xC8);
    lcd_data(eight);   // display on 8

    lcd_pos(0xC7);
    lcd_data(seven);   // display on 7
}

void lcd_display_tempF_bottom()
{
    adc_convert_temp();

    int seven = 32;
    int eight = (DegF / 10 % 10) + 48;   // might change these to chars
    int nine = (DegF / 1 % 10) + 48;
    int eleven = (DegF / 100 % 10) + 48;
    int twelve = (DegF / 1000 % 10) + 48;
    int thirteen = (DegF / 10000 % 10) + 48;

    lcd_pos(0xCf);
    lcd_data(70);   // display "F" on 15

    lcd_pos(0xCe);
    lcd_data(32);   // display " " on 14

    lcd_pos(0xCd);
    lcd_data(thirteen);   // display on 13

    lcd_pos(0xCc);
    lcd_data(twelve);   // display on 12

    lcd_pos(0xCb);
    lcd_data(eleven);   // display on 11

    lcd_pos(0xCa);
    lcd_data(46);   // display "." on 10

    lcd_pos(0xC9);
    lcd_data(nine);   // display on 9

    lcd_pos(0xC8);
    lcd_data(eight);   // display on 8

    lcd_pos(0xC7);
    lcd_data(seven);   // display on 7
}

void adc_setup()   // Internal Temp Sensor
{
    ADC10CTL1 = INCH_10 + ADC10DIV_3;
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
    // ADC10MEM is collected here
}

void adc_convert_temp() // convert ADC10MEM to F and C
{
    ADC10CTL0 |= ENC + ADC10SC;
    __bis_SR_register(CPUOFF + GIE);

    temp = ADC10MEM;
    DegC = ((temp - 673) * 423) / 1024;

    temp = ADC10MEM;
    DegF = ((temp - 630) * 761) / 1024;
}

void timer0_setup()   // used for LCD display
{
    __enable_interrupt();
    TACCR0 = 30;
    TACCTL0 |= CCIE;
    TACTL = TASSEL_2 | MC_1;
    LPM0;
    TACCTL0 &= ~CCIE;
    __disable_interrupt();

}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
   __bic_SR_register_on_exit(CPUOFF);
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void ta0_isr(void)
{
    TACTL = 0;
    LPM0_EXIT;
}
