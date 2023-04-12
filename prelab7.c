/*
 * File:   prelab7.c
 * Author: Rodrigo Garc�a
 *
 * Created on 10 de abril de 2023, 09:03 AM
 */
// CONFIG1
#pragma config FOSC = EXTRC_CLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>



//Constantes
#define _XTAL_FREQ 8000000      // Oscilador de 8 MHz


//Funciones
void setup (void);


//Interrupciones
void __interrupt() isr (void)
{
    if (PIR1bits.ADIF)  // Interrupci�n del ADC
    {
        CCPR1L = (ADRESH>>1)+124;
        CCP1CONbits.DC1B1 = ADRESH & 0b01;
        CCP1CONbits.DC1B0 = ADRESL >> 7;
        PIR1bits.ADIF = 0;
        
    }
    
}
//MAIN
void main(void) {
    setup();
    while(1){
    
    if (ADCON0bits.GO == 0) {
        __delay_us(1000);
        ADCON0bits.GO = 1;
    } 
        
    }        
    return;
}

//Configuraciones
void setup (void){
    
    // Configuraci�n de los puertos
    ANSEL = 0b00000001;          //El primer bit como entrada anal�gica
    ANSELH = 0;
    
    TRISA = 0b00000001;         // PORTA como salida, RA0 como entrada
    PORTA = 0;                  // Limpiamos PORTA 
    

    
    // Configuraci�n del oscilador
    OSCCONbits.IRCF = 0b0111;    // IRCF <2:0> -> 111 8 MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    
    //COnfiguraci�n del ADC
    ADCON1bits.ADFM = 0;        
    ADCON1bits.VCFG0 = 0;       
    ADCON1bits.VCFG1 = 0;       
    
    ADCON0bits.ADCS = 0b10;     // ADCS <1:0> -> 10 FOSC/32
    ADCON0bits.CHS = 0b0000;    // CHS  <3:0> 
    
    ADCON0bits.ADON = 1;        // Encender ADC
    __delay_us(50);

    
    // Configuraci�n del PWM
    
    TRISCbits.TRISC2 = 1;       // Entrada del PWM
    CCP1CONbits.P1M = 0;        // Salida simple
    CCP1CONbits.CCP1M = 0b1100; // asignaci�n del modo a PWM
    
            
    CCPR1L = 0x0F;              // Valor inicial del duty cycle
    CCP1CONbits.DC1B = 0;       // CONFIG bits menos significativos
    
    
    // Configuraci�n del TIMER2
    
    PR2 = 255;                  // Periodo del TIMER2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TIMER2
    PIR1bits.TMR2IF = 0;        // Flag del TIMER2 en apagado
    
    while (PIR1bits.TMR2IF == 0); // Esperamos una interrupci�n del TIMER2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;           //salida del PWM
    
    //Configuraci�n de las interrupciones
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de los puertos
    
    PIE1bits.ADIE = 1;          // Habilitamos interrupciones del ADC
    
    
    PIR1bits.ADIF = 0;          // Flag del ADC en 0
    
    
   
}
