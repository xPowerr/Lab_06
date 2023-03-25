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
#include <stdint.h>
#define _tmr0_value 200 // establecer valor inicial para el TMR0
#define _XTAL_FREQ 8000000 // establecer oscilador intero a 8MHz
#define LEDS_1 PORTAbits.RA0 /// usar RA0 como salida del led

unsigned char display [10] = {
    // TABLA PARA CATODO COMUN
    // BITS DP G F E D C B A
    0b00111111, //0
    0b00000110, //1
    0b01011011, //2
    0b01001111, //3
    0b01100110, //4
    0b01101101, //5
    0b01111101, //6
    0b00000111, //7
    0b01111111, //8
    0b01100111, //9
    //RETLW 01110111B ;A
    //RETLW 01111100B ;B
    //RETLW 00111001B ;C
    //RETLW 01011110B ;D
    //RETLW 01111001B ;E
    //RETLW 01110001B ;F 
};

unsigned char display_dot [10] = {
    // TABLA PARA CATODO COMUN
    // BITS DP G F E D C B A
    0b10111111, //0.
    0b10000110, //1.
    0b11011011, //2.
    0b11001111, //3.
    0b11100110, //4.
    0b11101101, //5.
    0b11111101, //6.
    0b10000111, //7.
    0b11111111, //8.
    0b11100111, //9.
    //RETLW 01110111B ;A
    //RETLW 01111100B ;B
    //RETLW 00111001B ;C
    //RETLW 01011110B ;D
    //RETLW 01111001B ;E
    //RETLW 01110001B ;F 
};

unsigned int volt=0;
unsigned int volt_map=0;
unsigned int uni=0;
unsigned int dec=0;
unsigned int cen=0;

int map(int value, int inputmin, int inputmax, int outmin, int outmax){ //función para mappear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;};

// --------------- Rutina de  interrupciones --------------- 
void __interrupt() isr(void) {
    if (INTCONbits.T0IF) { // si se produce overflow en el TMR0 y se activa la bandera
        LEDS_1 = ~LEDS_1; // alternar el estado del led entre encendido y apagado
        INTCONbits.T0IF = 0; // limpiar bandera de interrupcion del TMR0
        TMR0 = 216;           // reestablecer el valor necesario para el TMR0
    }
    if (PIR1bits.ADIF) { // si se activa la bandera de interrupcion del ADC
        if (ADCON0bits.CHS == 0b0000){ // si está en ADC AN0
            PORTB = ADRESH; // asignar el PORTB como el potenciometro de PORTA0
            ADCON0bits.CHS = 0b0100; // cambiar a ADC AN5
        }
        else if (ADCON0bits.CHS == 0b0100){ // si está en ADC AN5
            volt = ADRESH; // asignar el display en la variable volt como el potenciometro de PORTA5
            //PORTD = volt;
            ADCON0bits.CHS = 0b0000; // cambiar a ADC AN0
        }
        PIR1bits.ADIF = 0; // limpiar la bandera de la interrupcion
    }
}

// --------------- Definiendo los Setups --------------- 
void setup(void);
void setupADC(void);
void displays(void);

// --------------- main --------------- 
void main(void) {
    setup ();
    setupADC ();
    while(1){
        __delay_ms(10);
        if (ADCON0bits.GO == 0) { // si la lectura del ADC se desactiva
            ADCON0bits.GO = 1;
        }
        displays ();
    }
    return;
}

// --------------- Setup General --------------- 
void setup(void){
    // --------------- Definir como digitales --------------- 
    ANSELbits.ANS0 = 1;
    ANSELbits.ANS4 = 1;
    ANSELH = 0;
    
    // --------------- Configura puertos --------------- 
    //TRISA = 0; // Configura PORTA como salida
    TRISB = 0; // Configura PORTB como salida
    TRISC = 0; // Configura PORTC como salida
    TRISD = 0; // Configura PORTD como salida
    TRISE = 0; // Configura PORTE como salida
    LEDS_1 = 0;    // Inicializa RA0 en estado bajo 

    // --------------- limpiar puertos --------------- 
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    // --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b111 ; // establecerlo en 8 MHz
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
    INTCONbits.PEIE = 1; // habilitar interrupciones perifericas
    PIE1bits.ADIE = 1; // habilitar interrupciones de ADC
    PIR1bits.ADIF = 0; // limpiar la bandera de interrupcion del ADC
    
}

// --------------- Setup del ADC --------------- 
void setupADC(void){
    // --------------- Configura el canal --------------- 
    ADCON0bits.CHS = 0b0000; // seleccionar AN0
    
            
    // --------------- Seleccion voltaje referencia --------------- 
    ADCON1bits.VCFG1 = 0; // Voltaje de referencia de 0V
    ADCON1bits.VCFG0 = 0; // Voltaje de referencia de 5V
            
    // --------------- Seleccion de reloj ---------------
    ADCON0bits.ADCS = 0b10; // Fosc/32
            
    // --------------- Habilitar interrupciones del ADC ---------------
    
            
    // --------------- Asignar 8 bits, justificado izquierda ---------------
    ADCON1bits.ADFM = 0;        
            
    //--------------- Iniciar el ADC ---------------
    ADCON0bits.ADON = 1;  
    __delay_ms(1);
}

// --------------- Setup del MUX Displays --------------- 
void displays(void){
    volt_map = map(volt, 0, 255, 0, 100); // mappear el valor del voltaje de 0 a 500 y no 255
    uni = (volt_map*5)/100; // enviar las unidades del voltaje a la variable unidades
    dec = ((volt_map*5)/10)%10; // enviar las decimas del valor del voltaje a la variable decimas
    cen = ((volt_map*5))%10; // enviar las centesimas del voltaje a la variable centesimas
    
    PORTC = display_dot[uni]; // enviar el valor de las unidades al PORTC
    PORTDbits.RD0 = 0; // MUX COMO EN ASSEMBLER
    PORTDbits.RD1 = 0;
    PORTDbits.RD2 = 1;
    __delay_ms(5);     // DELAY DE 5ms
    
    PORTC = display[dec]; // enviar el valor de las decimas al PORTC
    PORTDbits.RD0 = 0; // MUX COMO EN ASSEMBLER
    PORTDbits.RD1 = 1;
    PORTDbits.RD2 = 0;
    __delay_ms(5);     // DELAY DE 5ms
    
    PORTC = display[cen]; // enviar el valor de las centesimas al PORTC
    PORTDbits.RD0 = 1; // MUX COMO EN ASSEMBLER
    PORTDbits.RD1 = 0;
    PORTDbits.RD2 = 0;
    __delay_ms(5);     // DELAY DE 5ms
}