/* 
 * File:   main.c
 * Author: Silvano
 *
 * Created on 25 luglio 2018, 15.50
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>

#pragma config FOSC = INTOSC
#pragma config WDTE = OFF 
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF

#pragma config WRT = OFF
#pragma config PLLEN = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOREN = OFF
#pragma config LVP = ON

#define _XTAL_FREQ 31000

/*
 * RA0 -> pb turn on signal (open drain)
 * RA1 -> panel-to-load mosfet command
 * RA2 -> Vpan (analog input)
 * RA4 -> charge mosfet command (output)
 * RA5 -> on_cmd (input)
 */

// On and off thresholds expressed in ADC counts. To go from voltage to ADC
// counts apply the following formula cnts = (V * k * 1024) / Vref.
// Where k is the partition ratio of the external voltage divider and Vref is the
// ADC reference voltage value.
// In this case we have k = 10/43 and Vref = 2,048V
// Real ratio: k = 0.233255813953

#define V_ON 571   // 4.9V in ADC counts
#define V_CHG 525  // 4.5V
#define V_OFF 519  // 4.45V


#define CHG_WAIT 30000

uint16_t ADCread()
{
    ADCON0bits.ADON = 1;
    ADCON0bits.ADGO = 1;
    while(ADCON0bits.nDONE == 1)
    {

    }
    uint16_t value = (ADRESH << 8) | ADRESL;
    ADCON0bits.ADON = 0;
    return value;
}

void waitMs(uint16_t ms)
{
    uint16_t ofl_value = 0xFFFF - ms;
    TMR1H = ofl_value >> 8;
    TMR1L = ofl_value & 0xFF;

    while(PIR1bits.TMR1IF == 0) ;

    PIR1bits.TMR1IF = 0;
}

void pbOn()
{
    LATAbits.LATA4 = 0;     // unconnect power bank charge
    __delay_ms(100);
    LATAbits.LATA0 = 0;     // turn on pules, from hi-z to low
    __delay_ms(100);
    LATAbits.LATA0 = 1;    
}

enum
{
    PAN_LOW = 0x01,
    PAN_OK  = 0x02
}Pstat;

uint8_t pan_stat = PAN_LOW;
uint16_t adcRes = 0;

int main(int argc, char** argv) {

    // oscillator setup
    OSCCON = 0b00000010;
    
    // I/O setup
    ANSELA = 0b00010000;    // only RA2 analog input
    TRISA = 0b00101100;     // RA0, RA1 and RA4 as output
    WPUA = 0x00;
    ODCONA = 0x00;
    ODCONAbits.ODA0 = 1;    // set RA0 as open drain output
    SLRCONA = 0x00;
    LATAbits.LATA0 = 1;      // RA0 hi-z
    LATAbits.LATA4 = 0;      // recharge disconnected
    LATAbits.LATA1 = 0;      // load disconnected from panel

    // Fixed voltage reference settings
    FVRCONbits.FVREN = 1;
    FVRCONbits.ADFVR1 = 1;  // ADC Vref is 2 x FVR = 2.048V

    // ADC settings
    ADCON0 = 0b00001000;    // select channel 2 (RA2)
    ADCON1 = 0b10000011;    // right justified result, Vref attached to FVR

    // TMR1 settings
    T1CON = 0b00110100;     // clock at fosc/32, this gives approx 1.03 ms/tick
    T1GCON = 0;
    PIE1bits.TMR1IE = 1;    // enable interrupt flag
    T1CONbits.TMR1ON = 1;

    while(1)
    {
        // begin ADC conversion
        adcRes = ADCread();

        if((adcRes > V_ON) && (pan_stat == PAN_LOW))
        {
            LATAbits.LATA1 = 1;  // load connected to panel
            __delay_ms(100);
            LATAbits.LATA4 = 1;  // mosfet on, charge connected to panel
            waitMs(1000);
            if(ADCread() < V_CHG)
            {
                // Panel voltage is not sufficient to initiate the recharge,
                // connect back load to power bank and disconnect recharge
                pbOn();
                __delay_ms(300);
                LATAbits.LATA1 = 0;  // disconnect load from panel
                pan_stat = PAN_LOW;
                waitMs(CHG_WAIT);
            }
            else
            {
                // voltage OK, keep power bank and load connected to panel
                pan_stat = PAN_OK;
            }
        }

        if((adcRes < V_OFF) && (pan_stat == PAN_OK))
        {
            // Panel voltage too low for recharge, disconnect power bank from panel
            // and connect load to power bank
            pbOn();
            __delay_ms(300);
            LATAbits.LATA1 = 0;     // disconnect load from panel
            pan_stat = PAN_LOW;
            waitMs(CHG_WAIT);       // anti-glitch
        }

        if((pan_stat == PAN_OK) && (PORTAbits.RA5 == 1))
        {
            pbOn();
            __delay_ms(300);
            LATAbits.LATA1 = 0;      // disconnect load from panel
        }

        if((pan_stat == PAN_OK) && (PORTAbits.RA5 == 0))
        {
            LATAbits.LATA1 =  1;     // connect load to panel
            __delay_ms(100);
            LATAbits.LATA4 = 1;      // disconnect recharge
        }

        __delay_ms(30000);
    }
    return (EXIT_SUCCESS);
}

