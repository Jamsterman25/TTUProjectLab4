/*
 HW I/O assignments:
 P1.0    Front Digital IR
 P1.1    BlueSMiRF RX
 P1.2    BlueSMiRF TX
 P1.3    Left Analog IR Sensor
 P1.4    Right Analog IR Sensor
 P1.5    MS CLK
 P1.6    MISO
 P1.7    MOSI
 P2.0    ENB (TA1 Right Track)
 P2.1    IN3 (Right FWD Side B)
 P2.2    IN1 (Left REV Side A)
 P2.3    IN2 (Left FWD Side A)
 P2.4    IN4 (Right REV Side B)
 P2.5    ENA (TA0 Left Track)
 P2.6    CS (SPI)
 P2.7    RESET Pushbutton
 */

#include  <msp430g2553.h>
#include <msp430.h>

unsigned char Rx_Data=0;
unsigned int side;
unsigned int direction;
int left[10] = {0};
int left_avg = 0;
int right[10] = {0};
int right_avg = 0;
int front[10] = {0};
int front_avg = 0;
unsigned char SERVO1_Data = '1';
static char data;
static char manual;
int r;
int l;

void ManualMode();

void FWD();
void Left();
void Man_Left();
void Veer_Left();
void Right();
void Man_Right();
void Veer_Right();
void REV();
void Stop();
void Send(char * tx_data);
void setSERVO(unsigned char dataOut);

void main(void)
{

    /*** Set-up system clocks ***/
    WDTCTL = WDTPW + WDTHOLD;               // Stop WDT
    DCOCTL=0;
    BCSCTL1 = CALBC1_1MHZ; // Set DCO
    DCOCTL = CALDCO_1MHZ;
   // BCSCTL2= DIVS_0 + DIVM_0; // divider=4 for SMCLK and 1 for MCLK

    //-----------------------Set-up USCI A---------------------------
    UCA0CTL1 |= UCSSEL_2;       // Use SMCLK
    UCA0BR0 = 109; //lower byte of UCBR0. 26dec
                        //(4MHz / 9600 baud)  see table 15-5
    UCA0BR1 = 0;    // 1MHz 9600
    UCA0MCTL = UCBRS_0 ;// +  UCBRF_1 + UCOS16;  //sets UCBRFx to 1,
                                              // UCBRSx to 0 , UCOS16=1
  //  UCA0CTL0 = 0; //UART mode, No parity, LSB first, 8 data, 1 stop

    UCA0CTL1 &= ~UCSWRST;       // SMCLK
    //UCB0CTL1 = UCSWRST;


    UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC;  // Setting up SPI master
    UCB0CTL1 |= UCSSEL_2;                     // SMCLK
    UCB0BR0 |= 0x02;                          // /2 is prescaler value for clock
    UCB0BR1 = 0;                              //
    UCB0CTL1 &= ~UCSWRST;                     // Initialize USCI

    UC0IE |= UCA0RXIE; // Enable USCI_A1 RX interrupt
    IE2 |= UCA0RXIE;            // Enable USCI_A0 RX interrupt

    //---------------------------Setup Timer A0------------------------
    TA0CCTL0 |= CCIE;           // enable CC interrupts
    TA0CCTL1 |= OUTMOD_7 + CCIE;  // Set/Reset mode and enable CC interrupts
    TA0CTL |= TASSEL_2 + MC_1;
    TA0CCR0 = 9999; // Count up to 10000

    //---------------------------Setup Timer A1------------------------
    TA1CCTL0 |= CCIE;           // enable CC interrupts
    TA1CCTL1 |= OUTMOD_7 + CCIE;  // Set/Reset mode and enable CC interrupts
    TA1CTL |= TASSEL_2 + MC_1;
    TA1CCR0 = 9999; // Count up to 10000


    //----------------------Set-up GPIO Pins------------------------
    P1SEL = BIT1 + BIT2 + BIT5 + BIT6 + BIT7;      // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 + BIT5 + BIT6 + BIT7;     // P1.1 = RXD, P1.2=TXD

    P2OUT = 0x00u;
    P2DIR |= BIT0;  // set P2.0 as output (ENB)
    P2DIR |= BIT1;  // set P2.1 as output (IN3) Right REV
    P2DIR |= BIT2;  // set P2.2 as output (IN1) Left FWD
    P2DIR |= BIT3;  // set P2.3 as output (IN2) Left REV
    P2DIR |= BIT4;  // set P2.4 as output (IN4) Right FWD
    P2DIR |= BIT5;  // set P2.5 as output (ENA)


    P2SEL |= BIT0 + BIT5;       // TA1CCR1 on P2.1
    P2SEL &= ~(BIT0 +BIT5);
    P2SEL &= ~0xC0; // Reconfigure P2.6 and P2.7 from XIN and XOUT

    P2DIR |= BIT6; // Set P2.6 for SPI Chip Select (CS) pin
    P2OUT |= BIT6; // Tell slave to turn on

    P2OUT = 0x21u; // Activate ENA + ENB

    //-----------------Setup P2.7 for Reset Pushbutton---------------
    P2REN|=BIT7;          // Enable internal pull-up/down resistors
    P2OUT|=BIT7;          // Select pull-up mode for P2.7
    P2IFG &= ~BIT7;       // P2.7 IFG cleared
    P2IE |= BIT7;         // P2.7 interrupt enabled
    P2IES &= ~BIT7;       // P2.7 lo/Hi edge



    __bis_SR_register(GIE); // Enable Interrupts

    side = 0;

    while(1)
    {
       if ( side == 0 )
        {
            ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;    // Sample & Hold Time + ADC10 ON + Interrupt Enable
            ADC10CTL1 = INCH_3 + CONSEQ_2;                         // input A0
            ADC10DTC1 = 0x0A;                                   // 10 conversions
            ADC10AE0 |= BIT3;                           // P1.0 ADC Analog Enable

            ADC10CTL0 &= ~ENC;             // Disable Conversion
            while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
            ADC10SA = (int)left;             // Transfers data to next array (DTC auto increments address)
            ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
            __bis_SR_register(CPUOFF + GIE);// Low Power Mode 0, ADC10_ISR

            left_avg = ((left[0]+left[1]+left[2]+left[3]+left[4]+left[5]+left[6]+left[7]+left[8]+left[9])/10);

            side = 1;
        }

        if ( side == 1 )
        {
            ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;    // Sample & Hold Time + ADC10 ON + Interrupt Enable
            ADC10CTL1 = INCH_0 + CONSEQ_2;                         // input A0
            ADC10DTC1 = 0x0A;                                   // 10 conversions
            ADC10AE0 |= BIT0;                           // P1.0 ADC Analog Enable

            ADC10CTL0 &= ~ENC;             // Disable Conversion
            while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
            ADC10SA = (int)front;             // Transfers data to next array (DTC auto increments address)
            ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
            __bis_SR_register(CPUOFF + GIE);// Low Power Mode 0, ADC10_ISR

            front_avg = ((front[0]+front[1]+front[2]+front[3]+front[4]+front[5]+front[6]+front[7]+front[8]+front[9])/10);

            side = 2;
        }

        if ( side == 2 )
        {
            ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;    // Sample & Hold Time + ADC10 ON + Interrupt Enable
            ADC10CTL1 = INCH_4 + CONSEQ_2;                         // input A0
            ADC10DTC1 = 0x0A;                                   // 10 conversions
            ADC10AE0 |= BIT4;                           // P1.0 ADC Analog Enable

            ADC10CTL0 &= ~ENC;             // Disable Conversion
            while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
            ADC10SA = (int)right;             // Transfers data to next array (DTC auto increments address)
            ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
            __bis_SR_register(CPUOFF + GIE);// Low Power Mode 0, ADC10_ISR

            right_avg = ((right[0]+right[1]+right[2]+right[3]+right[4]+right[5]+right[6]+right[7]+right[8]+right[9])/10);

            side = 0;
        }

        if((left_avg > 299) && (left_avg < 401) && (front_avg < 300))
            direction = 1;
        else if((left_avg < 300) && (front_avg < 300))
            direction = 2;
        else if((left_avg > 400) && (front_avg < 300))
            direction = 3;
        else if((left_avg > right_avg) && (front_avg > 299))
            direction = 4;
        else if((left_avg < right_avg) && (front_avg > 299))
            direction = 5;

        switch(direction)
        {
            case 1:
                FWD();
                break;
            case 2:
                Veer_Left();
                break;
            case 3:
                Veer_Right();
                break;
            case 4:
                Right();
                break;
            case 5:
                Left();
                break;
            default:
                Stop();
                break;
        }

        if(manual == 1){
            ManualMode();
        }
    }
}

#pragma vector=USCIAB0RX_VECTOR  // Receive UART interrupt
__interrupt void USCI0RX_ISR(void)
{
    //Rx_Data = UCA0RXBUF; // Assign received byte to Rx_Data
    //__bic_SR_register_on_exit(LPM0_bits);   // Wake-up CPU
    data = UCA0RXBUF;

    switch (data)
               {
                   case '1':
                       Send("SERVO1 \r\n");
                       setSERVO(SERVO1_Data);
                       //P2OUT &= ~(BIT0 + BIT5);
                       Stop();
                       __delay_cycles(8000000);
                       manual = 0;
                       break;

                   case 'w':
                       manual = 1;
                       break;
                   case 's':
                       manual = 1;
                       break;
                   case 'a':
                       manual = 1;
                       break;
                   case 'd':
                       manual = 1;
                       break;
                   case 'x':
                       manual = 1;
                       break;
                   case 'z':
                       manual = 0;
                       break;
                   default:
                       Send("Unknown Command \r\n");
                       break;
              }

}



#pragma vector = TIMER1_A0_VECTOR
__interrupt void captureCompareInt1 (void)
{
    P2OUT |= BIT0;                        //Turn on ENABLES
    TA1CCTL0 &= ~CCIFG;                //clear capture compare interrupt flag
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void captureCompareInt2 (void)
{
    P2OUT &= ~(BIT0);                        //Turn off ENABLES
    TA1CCTL1 &= ~CCIFG;                //clear capture compare interrupt flag
}


#pragma vector = TIMER0_A0_VECTOR
__interrupt void captureCompareInt3 (void)
{
    P2OUT |= BIT5;                        //Turn on ENABLES
    TA0CCTL0 &= ~CCIFG;                //clear capture compare interrupt flag
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void captureCompareInt4 (void)
{
    P2OUT &= ~(BIT5);                        //Turn off ENABLES
    TA0CCTL1 &= ~CCIFG;                //clear capture compare interrupt flag
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}



void ManualMode(){
    while(manual == 1){
       switch (data){
           case 'w':
               FWD();
               break;
           case 'a':
               Man_Left();
               break;
           case 's':
               REV();
               break;
           case 'd':
               Man_Right();
               break;
           case 'x':
               Stop();
               break;
           default:
               Stop();
               break;
       }
    }
    return;
}



void FWD()
{
    P2OUT &= ~(BIT2 + BIT4);   // Stop left FWD and right REV
    P2OUT |= BIT3 + BIT1;      // Enable left REV and right FWD

    TA0CCR1 = 4000;
    TA1CCR1 = 4000;
}

void Right()
{
    for(r = 30000; r > 0; --r)
    {
        P2OUT &= ~(BIT2 + BIT1);    // Stop left and right REV
        P2OUT |= BIT3 + BIT4;       // Enable left and right FWD

        TA0CCR1 = 7000;
        TA1CCR1 = 7000;

    }
}

void Man_Right()
{
    for(r = 1000; r > 0; --r)
       {
           P2OUT &= ~(BIT2 + BIT1);    // Stop left and right REV
           P2OUT |= BIT3 + BIT4;       // Enable left and right FWD

           TA0CCR1 = 7000;
           TA1CCR1 = 7000;
       }
}


void Veer_Right()
{
    P2OUT &= ~(BIT2 + BIT4);   // Stop left FWD and right REV
    P2OUT |= BIT3 + BIT1;      // Enable left REV and right FWD

    TA0CCR1 = 5000;
    TA1CCR1 = 2000;
}

void Left()
{
    for(l = 30000; l > 0; --l)
    {
        P2OUT &= ~(BIT3 + BIT4);    // Stop left and right FWD
        P2OUT |= BIT2 + BIT1;       // Enable left and right REV

        TA0CCR1 = 7000;
        TA1CCR1 = 7000;
    }
}

void Man_Left()
{
    for(l = 1000; l > 0; --l)
        {
            P2OUT &= ~(BIT3 + BIT4);    // Stop left and right FWD
            P2OUT |= BIT2 + BIT1;       // Enable left and right REV

            TA0CCR1 = 7000;
            TA1CCR1 = 7000;
        }
}


void Veer_Left()
{
    P2OUT &= ~(BIT2 + BIT4);   // Stop left FWD and right REV
    P2OUT |= BIT3 + BIT1;      // Enable left REV and right FWD

    TA0CCR1 = 2000;
    TA1CCR1 = 5000;
}

void REV()
{
    P2OUT &= ~(BIT3 + BIT1);      // Stop left REV and left FWD
    P2OUT |= BIT2 + BIT4;        // Enable left FWD and right REV

    TA0CCR1 = 4000;
    TA1CCR1 = 4000;
}


void Stop()
{
    P2OUT &= ~(BIT3 + BIT2);        // Enable left FWD and right REV
    P2OUT &= ~(BIT1 + BIT4);
    TA0CCR1 = 0;
    TA1CCR1 = 0;
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

// SPI function for sending data from Master MCU to MCU2 through P.3
void setSERVO(unsigned char dataOut)
{
    P2OUT &= ~BIT6;                // enable slave (CS to Low)
    UCB0TXBUF = 0;                 // Send command
    while (!(IFG2 & UCB0TXIFG));   // wait for TX buffer ready
    UCB0TXBUF = dataOut;           // Send wiper level
    while (!(IFG2 & UCB0TXIFG));   // wait for TX buffer ready
    while (UCB0STAT & UCBUSY);     // wait for the tx to complete
    P2OUT |= BIT6;                 // disable Slave (CS to High)
}
