#include "adc.h"
#include "timer.h"

void adc_init(void){


    SYSCTL_RCGCGPIO_R |= 0b00000010;
    SYSCTL_RCGCADC_R |= 0b01;
    timer_waitMillis(1);
    GPIO_PORTB_AFSEL_R |= 0b00010000;
    GPIO_PORTB_DIR_R &= 0b11101111;
    GPIO_PORTB_DEN_R   &= 0b11101111;
    GPIO_PORTB_AMSEL_R |= 0b00010000;
    GPIO_PORTB_ADCCTL_R = 0x00;

    ADC0_ACTSS_R &= 0xFFFFFFFE;
    ADC0_EMUX_R &= 0xFFF0;
    ADC0_SSMUX0_R |= 0xA;
    ADC0_SSCTL0_R &= 0xFFFFFFF0;
    ADC0_SSCTL0_R |=0x00000006;
    ADC0_ACTSS_R |= 0x00000001;


}

int adc_read(void){

        int data = 0;
        ADC0_PSSI_R = ADC_PSSI_SS0;

        while (!(ADC0_RIS_R & ADC_RIS_INR0)){

        }

//        data = (int)(ADC0_SSFIFO0_R & ADC_SSFIFO0_DATA_M);
////        ADC0_ISC_R &= ADC_ISC_IN0;
    timer_waitMillis(50);
    return ADC0_SSFIFO0_R;
}
