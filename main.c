#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "C:\ti\TivaWare_C_Series-2.1.4.178\inc\tm4c123gh6pm.h"
#include "C:\ti\TivaWare_C_Series-2.1.4.178\inc\hw_memmap.h"
#include "C:\ti\TivaWare_C_Series-2.1.4.178\inc\hw_types.h"
#include "C:\ti\TivaWare_C_Series-2.1.4.178\driverlib\sysctl.h"
#include "C:\ti\TivaWare_C_Series-2.1.4.178\driverlib\interrupt.h"
#include "C:\ti\TivaWare_C_Series-2.1.4.178\driverlib\gpio.h"
#include "C:\ti\TivaWare_C_Series-2.1.4.178\driverlib\timer.h"

#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define CS_NOT (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0 (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))

#define input_length 50
#define DC_MODE 1
#define SINE_MODE 2
#define VOLTAGE_SINE_MODE 3
#define VOLTAGE_DC_MODE 4

#define PI 3.1415

#define MAX_LIMIT 20

uint32_t phase = 0;
int dcmode;
char* voltsignl;
char* testsignl;
float input_frequency;
float input_frequency1;
float input_frequency2;
uint32_t duty = 0;
uint16_t sweep_freq = 0;
uint16_t raw = 0;
uint16_t inputamplitude;
float input_offset;
float input_amplitude;
uint32_t accr = 0;
char String_Entered[input_length];
float freqarray[100];
int floatarraypos = 0;
float instantarray[100];
int instantpos = 0;
float input_amplitude;
bool negative;
uint32_t time = 0;
char str[10];
uint32_t lookuptable[4096];
char stringfour[10];
uint32_t squarevol;
float input_voltage;
int sineflag = 0;
int sweepflag = 0;

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #7");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 7
    __asm("             CBZ  R1, WMS_DONE1");
    // 6+1*3
    __asm("             NOP");
    // 6

    __asm("             B    WMS_LOOP1");
    // 6*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             NOP");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        ;
    return UART0_DR_R & 0xFF;
}

void SetDACOutput(uint32_t data)
{
    SSI2_DR_R = data; // write data
    //while(SSI2_SR_R && SSI_SR_BSY);
}
uint32_t dc_out;
uint8_t mde = 5;
void Timer1Isr()
{
    if (mde == 0)
    {
        accr = accr + phase;
        SetDACOutput(lookuptable[accr >> 20]);
    }
    if (mde == 1)
    {
        SetDACOutput(dc_out);
        GREEN_LED ^= 1;

    }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // clear interrupt flag
}

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF
            | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= 0x0A; // make bit 1 an outputs
    GPIO_PORTF_DR2R_R |= 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x0A; // enable LED

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3; // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0; // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45; // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2; // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0xB0; // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0xB0; // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0xB0; // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK
            | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xB0; // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10; // must be enabled when SPO=1

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 100 KHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE; // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0; // select master mode
    SSI2_CC_R = 0; // select system clock as the clock source
    SSI2_CPSR_R = 20; // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE; // turn on SSI2

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (ctr down)
    TIMER1_TAILR_R = 0x190; // set load value to 40e6 for 100k Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16); // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    SYSCTL_RCGCADC_R |= 1;

}
void output_slow()
{
    // Configure AN0 as an analog input
    // turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x08;    // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x08;          // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x08;           // turn on analog operation on pin PE3
    ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;

}
void output_fast()
{
    // Configure AN1 as an analog input
    // turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x04;    // select alternative functions for AN1 (PE2)
    GPIO_PORTE_DEN_R &= ~0x04;          // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= 0x04;           // turn on analog operation on pin PE2
    ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 1;                               // set first sample to AN1
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY)
        ;           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                    // get single result from the FIFO
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = c;
}
void putnUart0(uint8_t n)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = n;
}
int n_tu(int number, int count)
{
    int result = 1;
    while (count-- > 0)
        result *= number;

    return result;
}

//converts an Integer to string (Courtesy: Stack over flow modified)
void float_to_string(float f, char r[])
{
    long long int length, length2, i, number, position, sign;
    float number2;

    sign = -1; // -1 == positive number
    if (f < 0)
    {
        sign = '-';
        f *= -1;
    }

    number2 = f;
    number = f;
    length = 0; // size of decimal part
    length2 = 0; // size of tenth

    /* calculate length2 tenth part*/
    while ((number2 - (float) number) != 0.0
            && !((number2 - (float) number) < 0.0))
    {
        number2 = f * (n_tu(10.0, length2 + 1));
        number = number2;

        length2++;
    }

    /* calculate length decimal part*/
    for (length = (f >= 1) ? 0 : 1; f >= 1; length++)
        f /= 10;

    position = length;
    length = length + 1 + length2;
    number = number2;
    if (sign == '-')
    {
        length++;
        position++;
    }

    for (i = length; i >= 0; i--)
    {
        if (i == (length))
            r[i] = '\0';
        else if (i == (position))
            r[i] = '.';
        else if (sign == '-' && i == 0)
            r[i] = '-';
        else
        {
            r[i] = (number % 10) + '0';
            number /= 10;
        }
    }
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}
void voltage_test()
{
    //uint16_t raw=0;
    output_slow();
    //float instantvalue;
    char str1[10];
    float instantvalue;
    raw = readAdc0Ss3();
    instantvalue = (3.3 * (raw + 0.5)) / 4096;
    //sprintf(str1, "%3.6f", instantvalue);
    float_to_string(instantvalue, str1);
    putsUart0(str1);
    putsUart0("\r\n");

}
float voltagesine()
{
    //uint16_t raw=0;
    output_slow();

    char str1[10];
    float instantvalue;
    raw = readAdc0Ss3();
    instantvalue = (3.3 * (raw + 0.5)) / 4096;
    instantarray[instantpos] = instantvalue;
    instantpos++;
    //sprintf(str1, "%3.6f", instantvalue);
    putsUart0(" ");
    float_to_string(instantvalue, str1);
    putsUart0(str1);
    putsUart0("\r\n");
    return instantvalue;

}

int main(void)
{

    initHw();
    RED_LED = 1;
    waitMicrosecond(1000);
    RED_LED = 0;

    while (1)
    {
        memset(String_Entered, NULL, input_length);

        putsUart0("\r\n************WELCOME**************************\r\n");

        const char *final[MAX_LIMIT];
        int len = 0;
        char name[MAX_LIMIT];
        char ch;
        int i = 0;
        char *inputarray[MAX_LIMIT];
        int inarraycounter = 0;
        char signalname[MAX_LIMIT];
        char input_frequencyname[MAX_LIMIT];
        char input_offsetname[MAX_LIMIT];
        char input_voltagename[MAX_LIMIT];
        char *input_str;
        char *in_str;
        input_str = String_Entered;
        uint8_t ctr = 0;
        //step 2
        //freqarray[]={NULL};
        //instantarray[]={NULL};
        //  memset ( freqarray, 0, 50 );
        // memset ( instantarray, 0, 50 );
        putsUart0("Enter your String:");

        while (ctr < input_length)
        {
            char c = getcUart0();
            if (c == '\r')
            {
                if (ctr == 0)
                {

                }
                else
                    break;
            }

            else if (c == 8)
            {
                if (ctr > 0)
                {
                    ctr--;
                    input_str[ctr] = NULL;

                }
                else if (ctr == 0)
                {
                    input_str[ctr] = NULL;
                }
                else
                {

                }

            }
            else if (c == '\b')
            {
                if (ctr > 0)
                    ctr--;
            }
            else if (c >= ' ')
            {
                input_str[ctr] = c;
                ctr++;
            }
            else
                ;
        }

        char *token = strtok(input_str, " ");
        printf(token[0]);

        while (token != NULL)
        {

            final[len] = token;
            len++;

            token = strtok(NULL, " ");
        }
        //  putsUart0("im here");
        putsUart0("\n");
        for (i = 0; i < len; i++)
        {
            //  printf("%s\n",final[i]);
            putsUart0(final[i]);
            putsUart0("\n");

        }
        //char* offsetname=final[3];
        //  name[i] = '\0';
        //  input_frequencyname[len2] = '\0';
        //  input_voltagename[len3] = '\0';
        //  input_offsetname[len4]= '\0';
        putsUart0("\n");

        // inserting null character at end
        //putsUart0( inputarray[inarraycounter-1]);
        //  putsUart0(input_frequencyname);
        //  putsUart0(input_voltagename);
        // putsUart0(offsetname);
        //checking if waveform name is valid
        char* signl = final[0];
        int length1 = strlen(signl);

        int k = 0;
        for (k = 0; k < length1; k++)
        {

            if (signl[k] >= 48 && signl[k] <= 57)
            {
                //printf("mistake waveform can contain nly alphabets\n");
                putsUart0("mistake waveform can contain only alphabets\n\r");
                // break;
            }
            else
            {
                signl[k] = toupper(signl[k]);

            }

        }
        //if it is a sine waveform, it can take two arguments for input_frequency and input_amplitude
        //or it can take input_frequency input_amplitude and input_offset
//****************************************************************************SINE*****************************************************************************
        if (strcmp(signl, "SINE") == 0)
        {
            int error = 0;  //no error in your program
            // printf("calling sine function\n");
            putsUart0("calling sine function\n\r");
            dcmode = 0;
            //if it contains three arguments
            //check freq , input_amplitude and input_offset
            if (len > 3)
            {

                //checking if input_frequency is valid

                char* freq = final[1];
                int l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers");
                        //break;
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                //inputamplitude=atoi(final[2]);
                int l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    // if(!((ampn[k]>=48 && ampn[k]<=57)||ampn[k]==46))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        // break;
                        error = 1;
                    }
                }

                //check if input_offset is valid
                //char* input_offsetn=offsetname;
                char* offsetname = final[3];
                int l4 = strlen(offsetname);
                // putsUart0("\n\r");
                //putsUart0(offsetname);
                k = 0;
                for (k = 0; k < l4; k++)
                {
                    if (!(offsetname[k] >= 48 && offsetname[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_offset can contain only numbers\n\r");
                        error = 1;
                    }
                }
                input_frequency = atoi(final[1]);
                input_offset = 0;
                input_offset = atof(offsetname);
                input_amplitude = atof(final[2]);

                if (error == 0)
                {
                    sineflag = 1;       //setting sine flag to 1
                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000
                                + (2070
                                        - (380
                                                * ((input_amplitude)
                                                        * sin(i * 2 * PI / 4096)
                                                        + input_offset)));
                    }

                    putsUart0("\r\n sine wave generated\n");
                }
                else
                {
                    putsUart0("\r\n Cannot generate sine wave \n");
                }

            }
            //if only two arguments are given
            else if (len == 3)
            {
                error = 0;
                //checking if input_frequency is valid

                char* freq = final[1];
                int l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers\n\r");
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                int l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!(ampn[k]>=48 && ampn[k]<=57))
                    //if(!((ampn[k]>=48 && ampn[k]<=57 )||ampn[k]==46 ))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        //   break;
                        error = 1;
                    }
                }

                input_frequency = atoi(final[1]);

                input_amplitude = atoi(final[2]);
                //inputamplitude=atoi(final[2]);
                putsUart0("calling sine function\n\r");
                if (error == 0)
                {

                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000
                                + (2070
                                        - ((input_amplitude) * 380
                                                * (sin(i * 2 * PI / 4096))));
                    }

                    //TIMER1_CTL_R |= TIMER_CTL_TAEN;

                    putsUart0("\r\nsine wave generated\n");
                }
                else
                {
                    putsUart0("\r\nsine wave cannot be generated\n");
                }

            }
            else
            {
                putsUart0("your sine function parameters are wrong\n\r");
            }

        }
//*********************************************************************************************************************************************************

//    else if(strcmp(signl,"INVOKE")==0)
//             {
//
//             }
//****************************************************************SWEEP***********************************************************************************
        else if (strcmp(signl, "SWEEP") == 0)
        {
            putsUart0("Calling Sweep function");
            uint16_t error = 0;
            if (dcmode == 1)
            {
                error = 1;
                sweepflag = 0;
                dcmode = 0;
            }
            if (sineflag == 0)
            {
                error = 1;
            }
            if (error == 0)
            {
                sweepflag = 1;
                char* freq1 = final[1];
                char* freq2 = final[2];
                uint16_t l2 = strlen(freq1);
                uint16_t l3 = strlen(freq2);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq1[k] >= 48 && freq1[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers\n\r");
                        uint16_t error = 1;
                    }
                }
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    if (!(freq2[k] >= 48 && freq2[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers\n\r");
                        uint16_t error = 1;
                    }
                }

                uint32_t range1 = atoi(final[1]);
                uint32_t range2 = atoi(final[2]);
                uint32_t new_freq = 0;

                uint16_t q = 0;

                putsUart0("\r\n The signal is: sweep \r\n");
                floatarraypos = 0;
                instantpos = 0;
                uint32_t step = 0;
                step = ((range2 - range1) / 10);
                float_to_string(range1, str);

                putsUart0(str);
                putsUart0("\r\t");

                freqarray[floatarraypos] = range1;
                floatarraypos++;
                new_freq = range1;
                if (range1 < range2)
                {
                    float_to_string(range1, str);
                    putsUart0(str);
                    putsUart0("\r\t");

                    voltagesine();
                    freqarray[floatarraypos] = new_freq;

                    for (q = 0; q < (range2 - range1); q += step)
                    {
                        new_freq = new_freq + step;
                        freqarray[floatarraypos] = new_freq;
                        floatarraypos++;
                        phase = new_freq * pow(2, 32) / 100000;
                        waitMicrosecond(30);
                        //sprintf(str, "Frequency = %d  \r \n", new_freq);

                        float_to_string(new_freq, str);

                        putsUart0(str);
                        putsUart0("\r\t");
                        // TIMER1_CTL_R |= TIMER_CTL_TAEN;
                        voltagesine();
                    }
                    if (new_freq < 1000)
                    {
                        output_slow();
                        waitMicrosecond(30);

                        //sprintf(str, "Frequency = %d  \r \n", new_freq);
                        float_to_string(new_freq, str);
                        putsUart0(str);
                        putsUart0("\r\n");

                    }
                    if (new_freq > 1000)
                    {
                        output_fast();
                        waitMicrosecond(10);
                        voltagesine();
                        //sprintf(str, "Frequency = %d  \r \n", new_freq);
                        float_to_string(new_freq, str);
                        putsUart0(str);
                        putsUart0("\r\n");

                    }
                }
                if (range1 > range2)
                {

                    putsUart0(" error \r \n");

                }
            }
            else
            {
                putsUart0("\r\n failed to call sweep \r \n");
            }
//*************************************************************************VOLTAGE TEST*******************************************************************
        }
        else if (strcmp(signl, "VOLTAGE") == 0)

        {
            dcmode = 0;
            putsUart0("IM HERE\n\r");
            voltage_test();
            putsUart0("calling voltage test function\n\r");

        }
        else if (strcmp(signl, "VOLTAGETEST") == 0)

        {
            dcmode = 0;
            putsUart0("IM HERE\n\r");
            //voltagesine();
            voltage_test();
            putsUart0("calling voltage test function\n\r");

        }
//******************************************************************TRIANGLE******************************************************************************

        else if (strcmp(signl, "TRIANGLE") == 0)
        {
            uint16_t error = 0;       //no error in your program
            // printf("calling sine function\n");
            putsUart0("calling triangle function\n\r");
            dcmode = 0;
            //if it contains three arguments
            //check freq , input_amplitude and input_offset
            if (len > 3)
            {

                //checking if input_frequency is valid

                char* freq = final[1];
                uint16_t l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers");
                        //break;
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                //inputamplitude=atoi(final[2]);
                uint16_t l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!((ampn[k]>=48 && ampn[k]<=57)||ampn[k]==46))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        // break;
                        error = 1;
                    }
                }

                //check if input_offset is valid
                //char* input_offsetn=offsetname;
                char* offsetname = final[3];
                uint16_t l4 = strlen(offsetname);
                // putsUart0("\n\r");
                //putsUart0(offsetname);
                k = 0;
                for (k = 0; k < l4; k++)
                {
                    if (!(offsetname[k] >= 48 && offsetname[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_offset can contain only numbers\n\r");
                        error = 1;
                    }
                }
                input_frequency = atoi(final[1]);
                input_offset = 0;
                input_offset = atof(offsetname);
                input_amplitude = atof(final[2]);

                if (error == 0)

                {
                    sineflag = 1;                  //setting sine flag to 1
                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 1024; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                - (380 * input_amplitude * i / 1024);
                    }
                    for (i = 1024; i < 3072; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                - (380 * input_amplitude * (2048 - i) / 1024);
                    }
                    for (i = 3072; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                + (380 * input_amplitude * (4096 - i) / 1024);
                    }
                    if (input_amplitude <= 5 && negative == 0)
                    {

                        //TIMER1_CTL_R |= TIMER_CTL_TAEN;
                    }
                    putsUart0("\r\n triangle wave generated\n");
                }
                else
                {
                    putsUart0("\r\n Cannot generate triangle wave \n");
                }

            }
            // if only two arguments are given
            else if (len == 3)
            {
                error = 0;
                //checking if input_frequency is valid

                char* freq = final[1];
                uint16_t l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers\n\r");
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                uint16_t l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!(ampn[k]>=48 && ampn[k]<=57))
                    //if(!((ampn[k]>=48 && ampn[k]<=57 )||ampn[k]==46 ))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        //   break;
                        error = 1;
                    }
                }

                input_frequency = atoi(final[1]);

                input_amplitude = atoi(final[2]);
                //inputamplitude=atoi(final[2]);
                putsUart0("calling triangle function\n\r");
                if (error == 0)
                {

                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 1024; i++)
                    {
                        lookuptable[i] = 0x3000 + 2070
                                - (380 * input_amplitude * i / 1024);
                    }
                    for (i = 1024; i < 3072; i++)
                    {
                        lookuptable[i] = 0x3000 + 2070
                                - (380 * input_amplitude * (2048 - i) / 1024);
                    }
                    for (i = 3072; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 + 2070
                                + (380 * input_amplitude * (4096 - i) / 1024);
                    }
                    //TIMER1_CTL_R |= TIMER_CTL_TAEN;

                    putsUart0("\r\nTriangle wave generated\n");
                }
                else
                {
                    putsUart0("\r\nTriangle wave cannot be generated\n");
                }

            }
            else
            {
                putsUart0("Triangle function parameters are wrong\n\r");
            }

        }
//***********************************************************GAIN***********************************************************************************
        else if (strcmp(signl, "GAIN") == 0)

        {
            int error = 0;
            if (sweepflag == 0)
                error = 1;
            dcmode = 0;
            putsUart0("IM HERE\n\r");
            float gain;
            int r;
            char str1[20];
            char str2[20];
            float v;
            if (error == 0)
            {
                r = instantpos - 1;
                int h;

                for (h = 0; h < r; h++)
                {
                    float_to_string(freqarray[h], str1);
                    putsUart0(str1);
                    putsUart0(" ");
                    //calculate gain value for each freq value in freqarray;
                    float g;
                    g = instantarray[h];

                    gain = 20 * log(input_amplitude / g);
                    //sprintf(str1, "%3.6f", gain);
                    float_to_string(gain, str1);
                    putsUart0(str1);
                    putsUart0("\r\n");

                }

            }
            if (error == 1)
            {
                putsUart0("\r\n failed to call gain \r\n");
            }
        }
//******************************************************************************SQUARE DUTY****************************************************
        else if (strcmp(signl, "SQUAREDUTY") == 0)
        {
            int error = 0;                  //no error in your program
            // printf("calling sine function\n");
            putsUart0("Calling Square Duty function\n\r");
            dcmode = 0;
            //if it contains three arguments
            //check freq , input_amplitude and input_offset
            if (len > 3)
            {

                //checking if input_frequency is valid

                char* freq = final[1];
                int l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers");
                        //break;
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                //inputamplitude=atoi(final[2]);
                int l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!((ampn[k]>=48 && ampn[k]<=57)||ampn[k]==46))
                    // if(!((ampn[k]>=48 && ampn[k]<=57 )||ampn[k]==46 ||ampn[k]==45))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        // break;
                        error = 1;
                    }
                }

                //check if input_offset is valid
                //char* input_offsetn=offsetname;
                char* offsetname = final[3];
                int l4 = strlen(offsetname);
                // putsUart0("\n\r");
                //putsUart0(offsetname);
                k = 0;
                for (k = 0; k < l4; k++)
                {
                    if (!(offsetname[k] >= 48 && offsetname[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_offset can contain only numbers\n\r");
                        error = 1;
                    }
                }
                input_frequency = atoi(final[1]);
                input_offset = 0;
                input_offset = atof(offsetname);
                input_amplitude = atof(final[2]);
                duty = atoi(final[4]);

                if (error == 0)
                {
                    sineflag = 1;       //setting sine flag to 1
                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    memset(&lookuptable[0], 0, sizeof(lookuptable));
                    for (i = 0; i < 4096 - (4096 * duty / 100); i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                + 380 * input_amplitude * 1;
                    }
                    for (i = 4096 - (4096 * duty / 100); i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                + 380 * input_amplitude * (-1);
                    }

                    putsUart0("\r\n Square Duty generated\n");
                }
                else
                {
                    putsUart0("\r\n Cannot generate Square Duty \n");
                }

            }
            //if only two arguments are given

        }
        //***********************************************SAWTOOTH***********************************************************************************
        else if (strcmp(signl, "SAWTOOTH") == 0)
        {
            int error = 0;       //no error in your program
            // printf("calling sine function\n");
            putsUart0("calling sawtooth function\n\r");
            dcmode = 0;
            //if it contains three arguments
            //check freq , input_amplitude and input_offset
            if (len > 3)
            {

                //checking if input_frequency is valid

                char* freq = final[1];
                int l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers");
                        //break;
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                //inputamplitude=atoi(final[2]);
                int l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!((ampn[k]>=48 && ampn[k]<=57)||ampn[k]==46))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        // break;
                        error = 1;
                    }
                }

                //check if input_offset is valid
                //char* input_offsetn=offsetname;
                char* offsetname = final[3];
                int l4 = strlen(offsetname);
                // putsUart0("\n\r");
                //putsUart0(offsetname);
                k = 0;
                for (k = 0; k < l4; k++)
                {
                    if (!(offsetname[k] >= 48 && offsetname[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_offset can contain only numbers\n\r");
                        error = 1;
                    }
                }
                input_frequency = atoi(final[1]);
                input_offset = 0;
                input_offset = atof(offsetname);
                input_amplitude = atof(final[2]);

                if (error == 0)
                {
                    sineflag = 1;          //setting sine flag to 1
                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset
                                + (382 * input_amplitude + 2070)
                                - (2 * 382 * input_amplitude * i) / 4096;
                    }
                    if (input_amplitude <= 5 && negative == 0)
                    {

                        //TIMER1_CTL_R |= TIMER_CTL_TAEN;
                    }
                    putsUart0("\r\n sawtooth wave generated\n");
                }
                else
                {
                    putsUart0("\r\n Cannot generate sawtooth wave \n");
                }

            }
            //if only two arguments are given
            else if (len == 3)
            {
                error = 0;
                //checking if input_frequency is valid

                char* freq = final[1];
                int l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers\n\r");
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                int l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!(ampn[k]>=48 && ampn[k]<=57))
                    //if(!((ampn[k]>=48 && ampn[k]<=57 )||ampn[k]==46 ))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        //   break;
                        error = 1;
                    }
                }

                input_frequency = atoi(final[1]);

                input_amplitude = atoi(final[2]);
                //inputamplitude=atoi(final[2]);
                putsUart0("calling sawtooth function\n\r");
                if (error == 0)
                {

                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 + (382 * input_amplitude + 2070)
                                - (2 * 382 * input_amplitude * i) / 4096;
                    }
                    //TIMER1_CTL_R |= TIMER_CTL_TAEN;

                    putsUart0("\r\nSawtooth wave generated\n");
                }
                else
                {
                    putsUart0("\r\nSawtooth wave cannot be generated\n");
                }

            }
            else
            {
                putsUart0("your sawtooth function parameters are wrong\n\r");
            }

        }

        //************************************************************************SQUARE*********************************************************************
        if (strcmp(signl, "SQUARE") == 0)
        {
            uint16_t error = 0;          //no error in your program
            // printf("calling sine function\n");
            putsUart0("calling square function\n\r");
            dcmode = 0;
            //if it contains three arguments
            //check freq , input_amplitude and input_offset
            if (len > 3)
            {

                //checking if input_frequency is valid

                char* freq = final[1];
                uint16_t l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers");
                        //break;
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                //inputamplitude=atoi(final[2]);
                uint16_t l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    //if(!((ampn[k]>=48 && ampn[k]<=57)||ampn[k]==46))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        // break;
                        error = 1;
                    }
                }

                //check if input_offset is valid
                //char* input_offsetn=offsetname;
                char* offsetname = final[3];
                uint16_t l4 = strlen(offsetname);
                // putsUart0("\n\r");
                //putsUart0(offsetname);
                k = 0;
                for (k = 0; k < l4; k++)
                {
                    if (!(offsetname[k] >= 48 && offsetname[k] <= 57))
                    {
                        putsUart0(
                                "Mistake input_offset can contain only numbers\n\r");
                        error = 1;
                    }
                }
                input_frequency = atoi(final[1]);
                input_offset = 0;
                input_offset = atof(offsetname);
                input_amplitude = atof(final[2]);

                if (error == 0)
                {
                    sineflag = 1;             //setting sine flag to 1
                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 2048; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                + 380 * input_amplitude * 1;

                    }
                    for (i = 2048; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 - 380 * input_offset + 2070
                                + 380 * input_amplitude * (-1);

                    }

                    putsUart0("\r\n square wave generated\n");
                }
                else
                {
                    putsUart0("\r\n Cannot generate square wave \n");
                }

            }
            //if only two arguments are given
            else if (len == 3)
            {
                error = 0;
                //checking if input_frequency is valid

                char* freq = final[1];
                int l2 = strlen(freq);
                k = 0;
                for (k = 0; k < l2; k++)
                {
                    if (!(freq[k] >= 48 && freq[k] <= 57))
                    {
                        putsUart0(
                                "mistake input_frequency can contain only numbers\n\r");
                        error = 1;
                    }
                }

                //checking if input_amplitude is valid

                char* ampn = final[2];
                int l3 = strlen(ampn);
                k = 0;
                for (k = 0; k < l3; k++)
                {
                    // if(!(ampn[k]>=48 && ampn[k]<=57))
                    //if(!((ampn[k]>=48 && ampn[k]<=57 )||ampn[k]==46 ))
                    if (!((ampn[k] >= 48 && ampn[k] <= 57) || ampn[k] == 46
                            || ampn[k] == 45))
                    {
                        putsUart0(
                                "mistake input_amplitude can contain only numbers\n\r");
                        //   break;
                        error = 1;
                    }
                }

                input_frequency = atoi(final[1]);

                input_amplitude = atoi(final[2]);
                //inputamplitude=atoi(final[2]);
                putsUart0("calling Square wave function\n\r");
                if (error == 0)
                {

                    phase = input_frequency * pow(2, 32) / 100000;
                    mde = 0;
                    uint16_t i;
                    for (i = 0; i < 2048; i++)
                    {
                        lookuptable[i] = 0x3000 + 2070
                                + 380 * input_amplitude * 1;

                    }
                    for (i = 2048; i < 4096; i++)
                    {
                        lookuptable[i] = 0x3000 + 2070
                                + 380 * input_amplitude * (-1);

                    }
                    //TIMER1_CTL_R |= TIMER_CTL_TAEN;

                    putsUart0("\r\nSquare wave generated\n");
                }
                else
                {
                    putsUart0("\r\nSquare wave cannot be generated\n");
                }

            }
            else
            {
                putsUart0("Square wave function parameters are wrong\n\r");
            }

        }
//************************************************************************RESET*************************************************************************
        else if (strcmp(signl, "RESET") == 0)
        {
            putsUart0("\r\n reset \r\n");

            __asm("    .global _c_int00\n"
                    "    b.w     _c_int00");

        }

//********************************************************************DC************************************************************************************
        //to check DC input_offset value
        else if (strcmp(signl, "DC") == 0)
        {
            dcmode = 1;
            putsUart0("calling dc function\n\r");
            int error = 0;
            //it can consider only one parameter

            if (len < 2)
            {
                putsUart0("your dc parameters format is wrong\n\r");
                //   break;
                int error = 1;
            }
            //checking if input_voltage is valid
            else
            {
                char* input_voltagen = final[1];

                int l4 = strlen(input_voltagen);
                k = 0;
                for (k = 0; k < l4; k++)
                {
                    if (!((input_voltagen[k] >= 48 && input_voltagen[k] <= 57)
                            || input_voltagen[k] == 46
                            || input_voltagen[k] == 45))
                    {
                        // printf("mistake input_voltage can contain nly numbers\n");
                        putsUart0(
                                "mistake input_voltage can contain only numbers\n\r");
                        // break;
                        int error = 1;
                    }
                }

                if (error == 0)
                {
                    input_voltage = atof(final[1]);

                    putsUart0("\r\nEntered dc mode\n");
                    dc_out = 0x3000 + 2070 + (input_voltage * -380);
                    GREEN_LED = 1;
                    waitMicrosecond(100);
                    GREEN_LED = 0;
                    mde = 1;
                }
                else
                {
                    putsUart0("\r\nCannot enter dc mode\n");
                }

            }
        }
    }
}

