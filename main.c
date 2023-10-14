#include <msp430.h> 
#include <stdint.h>

/** Trigger source period for sensor */
#define TRIGGER_PERIOD   (3300)         //  ~100.7ms , ACLK=32768 Hz

/** LED on time */
#define LED_ON_TIME   (328)             //  ~10ms , ACLK=32768 Hz

/** Initial LED period*/
#define INIT_LED_PERIOD   (3300)        //  ~100.7ms , ACLK=32768 Hz

/** Min. distance from sensor
 *  in clock edges of 1MHz*/
#define MIN_DISTANCE   (124)            //  ~118.25us, ~2cm, SMCLK=1048576 Hz

/** Max. distance from sensor
 *  in clock edges of 1MHz*/
#define MAX_DISTANCE   (15994)          //  ~15.25ms , ~2.59m, SMCLK=1048576 Hz


/**
 * @brief Package length
 */
#define PCKG_SIZE    (5)

/** Data buffer */
volatile uint8_t data[PCKG_SIZE] = {0}; //inicijalizacija niza i setovanje 0


/** The flag is set when active period
 * of echo signal is finished*/
volatile uint8_t flag = 0;  //Kad dodje echo signal && silazna ivica on se tad setuje

/** Active echo signal time in clock
 *  with 1MHz*/
static volatile uint32_t measured_sum = 0;  //Brojimo koliko taktova je trajao Echo impuls

/** Data counter */
static volatile uint8_t data_cnt = 0;


/** Distance from sensor in [mm] */
volatile uint32_t distance = 0;     //Prava distanca koju saljem na uart


/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // LED1
    P2OUT &= ~BIT4;             // clear P2.4
    P2DIR &= ~BIT4;             // configure P2.4 as input

    // LED2
    P2OUT &= ~BIT5;             // clear P2.5
    P2DIR &= ~BIT5;             // configure P2.5 as input


    // UART
    P4SEL |= BIT4 + BIT5;       // select P4.4 and P4.5 for USCI


    // initialize USCI
    UCA1CTL1 |= UCSWRST;              // put USCI in reset

    UCA1CTL0 = 0;                     // no parity, 8bit, 1 stop bit
    UCA1CTL1 |= UCSSEL__ACLK;         // use ACLK
    UCA1BR0 = 3;
    UCA1BR1 = 0;
    UCA1MCTL |= UCBRS_3 + UCBRF_0;    // BRS = 3 for 9600 bps
    UCA1CTL1 &= ~UCSWRST;             // release reset
    UCA1IE |= UCTXIE;                 // interrupt


    // Timer B0 as trigger signal source on P3.5
    P7SEL |= BIT4;                          // alternate function on P7.4 Biram timer na ovom pinu, tu ide trigger
    P7DIR |= BIT4;                          // TB0.2 OUT jer ide u senzor
    TB0CCR2 = TRIGGER_PERIOD>>11;           // (~30mS) je na '1' 70ms na 0 reset signala
    TB0CCTL2 = OUTMOD_7;                    // reset/set mode for signal |
    TB0CCR0 = TRIGGER_PERIOD;               // (~100mS) trigger signal period set signala
    TB0CTL = TBSSEL__ACLK | MC__UP;         // set TB clock source, 32768 Hz | biram mod rada i koliki je CLK


    // Timer A0 as signal source for LED1 and LED2
    // TA0.1 on LED1,  TA0.2 on LED2
    P1SEL |= BIT2;                      // alternate function on P1.2
    P1DIR |= BIT2;                      // TA0.1 OUT
    P1SEL |= BIT3;                      // alternate function on P1.3
    P1DIR |= BIT3;                      // TA0.2 OUT

    TA0CCR1 = LED_ON_TIME;              // LED ON(~10ms)
    TA0CCR2 = LED_ON_TIME;              //328/32768 = 10ms

    TA0CCTL1 = OUTMOD_7;                // reset/set mode for signal
    TA0CCTL2 = OUTMOD_7;                //timer registar za A0.2
    TA0CCR0 = INIT_LED_PERIOD;          // (~100ms) Init LED signal period
    TA0CTL = TBSSEL__ACLK | MC__UP;     // 32768 Hz, Up mode



    /* Timer A1 with input CCI1A P2.0 pin
     * to read echo signal from sensor
     */
    P2SEL |= BIT0;              // alternate function
    P2DIR &= ~BIT0;             // input CCI1A

    /* Capture compare 1 is in capture mode */
    TA1CCTL1 = CM_3  | SCS | CAP | CCIE;     // CM_3 capture on both edges
                                             // sync capture source
                                             // capture mode
                                             // enable interrupt for CCR1
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS; //free running counter wtih 1MHz clk

    __enable_interrupt();


    while(1)
        {
            while (!flag);      //ceka da se zavrsi ECHO, kad se zavrsi onda radi
            uint32_t measured_sum_led = measured_sum;
            if (measured_sum_led < MIN_DISTANCE){
                measured_sum_led = MIN_DISTANCE;
            }
            else if(measured_sum_led > MAX_DISTANCE){
                measured_sum_led = MAX_DISTANCE;
            }
            TA0CCR0 = (measured_sum_led*4 + 1557);
            TA1R = 0;       //reset echo signal timer
            flag = 0;       //vracamo za sl iteraciju
            data_cnt = 0;   //vracamo za sl iteraciju
            float measured_sum_float = (float)measured_sum;
            distance = (measured_sum_float*170000/1048576);

            // measured_sum/1048576 (1MHz) dobijemo pravo vreme
            //170000 * measured_sum s=v*t/2, v=340 000mm/s t=measured_sum_float/1MHz


            while (distance>0){
                data[data_cnt] = distance%10;
                distance /= 10;
                data_cnt++;
            }
            data[data_cnt] = 32;        // first send space in ASCII
            UCA1TXBUF = data[data_cnt];

        }

}


/**
 * @brief TIMERA1 Interrupt service routine
 *
 * ISR measures time between rising and falling edge
 */
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TA1CCR1ISR (void)
{

    static uint16_t curr = 0;
    static uint16_t last = 0;   //poslednja vrednost koju je timer imao pri uzlaznoj ivici ECHO signala

    switch (TA1IV)  //proveri od kog tajmera je dosao prekid
    {
    case TA1IV_TACCR1:

        if (P2IN & BIT0) {      // rising edge
            last = TA1R;        //koliko je izbrojano CLK
        }else{                  // falling edge, end of echo
            uint16_t delta;     //koliko CLK traje echo signal
            curr = TA1R;

            if (curr > last)
            {
                delta = curr - last;
            }
            else
            {
                delta = (0xffff - last) + curr + 1;
            }
            measured_sum = delta;
            flag = 1;       // end of measurement
        }
        break;
    default:
        break;
    }
}

/**
 * @brief USCIA1 ISR
 *
 * Implement packet transfers
 */
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCIA1 (void)
{
    switch (UCA1IV)
    {
    case USCI_UCTXIFG:
        if (data_cnt > 0)                   // if the transmit is not over
        {
            data_cnt -= 1;                  // decrement the counter, used as index
            UCA1TXBUF = data[data_cnt]+'0'; // send the data
        }                                   // if data_cnt == 0, skip
        break;
    }
}





