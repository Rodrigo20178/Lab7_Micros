/*
 * File:   Lab7.c
 * Author: Rodrigo Garc�a
 *
 * Created on 11 de abril de 2023, 11:14 AM
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


//Variables
uint8_t contador;
uint8_t Pot3_valor;

//Funciones
void setup (void);


//Interrupci�n
void __interrupt() isr (void)
{
    if (PIR1bits.ADIF)  // Interrupci�n del ADC
    {
         if (ADCON0bits.CHS == 0b0000){
            CCPR1L = (ADRESH>>1)+124;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = ADRESL >> 7;
         }
           
        else if (ADCON0bits.CHS == 0b0001) {
             CCPR2L = (ADRESH>>1)+124;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = ADRESL >> 7;
        }
        else
            Pot3_valor = ADRESH;
        PIR1bits.ADIF = 0;   
    }
    if (INTCONbits.T0IF)    // Interrupci�n del TIMER0
    {
        contador++;
        if (contador < Pot3_valor)  // Si el valor del contador es menor al de la conversi�n del Pot, encender LED
            PORTCbits.RC3 = 1;
        else
            PORTCbits.RC3 = 0;
        TMR0 = 6;
        INTCONbits.T0IF = 0;
    }    
}
//MAIN
void main(void) {
    setup();
    while(1){
    
    if (ADCON0bits.GO == 0) {
        if (ADCON0bits.CHS == 0b0000)
            ADCON0bits.CHS = 0b0001;
        else if (ADCON0bits.CHS == 0b0001)
            ADCON0bits.CHS = 0b0010;
        else
            ADCON0bits.CHS = 0b0000;
        
        __delay_us(1000);
        ADCON0bits.GO = 1;
        
    } 
        
    }        
    return;
}

//Configuraciones
void setup (void){
    
    // Configuraci�n de los puertos
    ANSEL = 0b00000111;          //AN0, AN1, AN2 como entradas anal�gicas
    ANSELH = 0;
    
    TRISA = 0b00000111;         // PORTA como salida, RA0 & RA1 como entradas 
    PORTA = 0;                  // Limpiamos PORTA 
    
    TRISC = 0;                  // PORTD como salida anal�gica
    PORTC = 0;                  // Limpiamos PORTD
    
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
    
    TRISCbits.TRISC2 = 1;       // RC2 como entrada
    TRISCbits.TRISC1 = 1;       // RC1 como entrada
    
    CCP1CONbits.P1M = 0;        // Salida simple
    CCP1CONbits.CCP1M = 0b1100; // asignaci�n del modo a PWM1
    
    CCP2CONbits.CCP2M = 0b1100; // asignaci�n del modo a PWM2
            
    CCPR1L = 0x0F;              // Valor inicial del duty cycle
    CCP1CONbits.DC1B = 0;       // CONFIG bits menos significativos
    
    CCPR2L = 0x0F;              // Valor inicial del duty cycle
    CCP2CONbits.DC2B1 = 0;       // CONFIG bits menos significativos
    CCP2CONbits.DC2B0 = 0;       // CONFIG bits menos significativos
    
    // Configuraci�n del TMR0
    OPTION_REGbits.T0CS = 0;    // TIMER0 como temporizador
    OPTION_REGbits.PSA = 0;     
    OPTION_REGbits.PS = 0b011;  // PS<2:0> -> 101 Prescaler 1:16
    TMR0 = 6;                  
    
    // Configuraci�n del TIMER2
    
    PR2 = 255;                  // Periodo del TIMER2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TIMER2
    PIR1bits.TMR2IF = 0;        // Flag del TIMER2 apagado
    
    while (PIR1bits.TMR2IF == 0); // Esperamos una interrupci�n del TIMER2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;       // RC2 como salida del PWM
    TRISCbits.TRISC1 = 0;       // RC1 como salida
    
    //Configuraci�n de las interrupciones
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de los puertos
    
    PIE1bits.ADIE = 1;          // Habilitamos interrupciones del ADC
    INTCONbits.TMR0IE = 1;      // Habilitamos interrupciones del TIMER0
    
    PIR1bits.ADIF = 0;          // Flag del ADC en 0
    INTCONbits.T0IF = 0;        // Flag del TIMER0 en 0
    
   
}


