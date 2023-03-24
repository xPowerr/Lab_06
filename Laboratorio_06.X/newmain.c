/*
 * File:   newmain.c
 * Autor: Gabriel Carrera 21216
 *
 * Creado el 24 de marzo, 2023, 9:38 AM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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

#include <xc.h>
#define _tmr0_value 200 // valor de tmr0 para la interupcion 
#define _XTAL_FREQ 8000000 
#define LEDS_1 PORTAbits.RA0 /// usar e0 como salida del led

// --------------- Rutina de  interrupciones --------------- 
void __interrupt() isr(void) {
    if (INTCONbits.T0IF) { // Ssi se produce overflow en el TMR0 y se activa la bandera
        LEDS_1 = ~LEDS_1; // alternar el estado del led entre encendido y apagado
        INTCONbits.T0IF = 0; // limpiar bandera de interrupcion del TMR0
        TMR0 = 216;           // reestablecer el valor necesario para el TMR0
    }
}
// --------------- Prototipo del Setup --------------- 
void setup(void);

// --------------- main --------------- 
void main(void) {
    setup ();
    while(1){
            
    }
    return;
}

// --------------- Setup General --------------- 
void setup(void){
    // --------------- Definir como digitales --------------- 
    ANSEL = 0;
    ANSELH = 0;
    
    // --------------- Configura el puerto A --------------- 
    TRISAbits.TRISA0 = 0; // Configura RA0 como salida para el LED
    LEDS_1 = 0;    // Inicializa RE0 en estado bajo 

    // --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b111 ; // establecerlo en 8 mHz
    OSCCONbits.SCS = 1; // utilizar oscilador interno
    

    // --------------- TMR0 --------------- 
    OPTION_REGbits.T0CS = 0; // utilizar el reloj interno (fosc/4)
    OPTION_REGbits.PSA = 0; // asignar el prescaler a Timer0
    OPTION_REGbits.PS2 = 1; // utilizar prescaler de 256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;  
    TMR0 = 216; ///VALOR INICIAL DEL TMR0
    
    // --------------- Banderas e interrupciones --------------- 
    INTCONbits.T0IF = 0; // establece la bandera de la interrupcion del TMR0 apagada
    INTCONbits.T0IE = 1; // habilitar iinterrupcion del TMR0
    INTCONbits.GIE = 1; // habilitar interrupciones globales
   
    
}
