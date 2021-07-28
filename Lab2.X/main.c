/*
 * File:   Lab 2
 * Author: Pablo Herrarte
 * Curso: Electrónica Digital 2 
 * Fecha: 27/07/2021
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

/**************************PALABRAS DE CONFIGURACIÓN**************************/
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
/******************************LIBRERÍAS***************************************/
#include <xc.h>
#include <stdint.h>
#include <pic16f887.h>
#include "LCD_libreria.h"       //LCD
#include "ADC.h"                //ADC
#include "ASCII.h"              //NUM ASCII
#include "USART.h"              //CONFIGURACIÓN USART
#define _XTAL_FREQ 4000000      //Frecuencia a trabajar
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/****************************VARIABLES***************************************/
uint8_t nowadc = 0; //Variables para controlar adc
uint8_t anlec = 0;
uint8_t v1 = 0; //Valores de los potenciómetros
uint8_t v2 = 0;
char Cen1 = 0;  //Dígito de potenciómetros y contador
char Cen2 = 0;
char Cen3 = 0;
char Dec1 = 0;
char Dec2 = 0;
char Dec3 = 0;
char Un1 = 0;   
char Un2 = 0;
char Un3 = 0;
char AC1 = 0;
char AC2 = 0;
char AC3 = 0;
char AD1 = 0;
char AD2 = 0;
char AD3 = 0;
char AU1 = 0;
char AU2 = 0;
char AU3 = 0;
uint8_t T = 1;  //Toggle para controlar dos señales analógicas
uint8_t toggleTX = 0;   //contador para mandar datos a la terminal virtual
uint8_t signo = 0;      //Control del signo para transmitir datos desde la terminal virtual
uint8_t sum = 0;        //Suma del contador
uint8_t res = 0;        //Resta del contador
uint8_t CONT = 0;       //Contador

//PROTOTIPO FUNCIONES
void Setup(void);   //Setup
void ADCL(void);    //Lectura ADC
void LECT1(void);   //Separación de dígitos y lectura del potenciómetro 1
void LECT2(void);   //Separación de dígitos y lectura del potenciómetro 2
void LECT3(void);   //Separación de digitos y lectura del contador
void envio(void);   //Datos que recibirá la terminal de lectura
void CONTADOR(void);    //Antirrebote del contador
const char* conver(char A, char B, char C); //Datos que recibirá la LCD

/*****************************INTERRUPCIONES***********************************/
void __interrupt() ISR(void){
    if(INTCONbits.TMR0IF == 1){ //Interrupción del timer 0
        INTCONbits.TMR0IF = 0;
        TMR0 = 236;
        nowadc++; //Se va sumando el valor de la variable de la ADC cada vez que el timer 0 hace un ciclo
    }
    if (PIR1bits.ADIF == 1){ //Si la conversión AD fue realizada se regresa la bandera a 0
        PIR1bits.ADIF = 0;
        anlec = ADRESH; //Señal analógica        
    }
    if (PIR1bits.RCIF == 1){    //El recieve buffer del EUSART está lleno
        signo = RCREG; //El signo será el dato recivido
    }
    if (PIR1bits.TXIF == 1){    //El transmit buffer del EUSART está vacío
        envio();    //Se mandará caracter por caracter con esta función
        PIE1bits.TXIE = 0;
    }
}    
/******************************CICLO*******************************************/
void main(void) {
    Setup();    //Setup
    USARTcon(); //Configuración de EUSART
    Lcd_Init(); //Inicialización de 8 bits para LCD
    while(1){
        Lcd_Set_Cursor(1,1);    //Cursor en primera línea
        Lcd_Write_String("S1    S2    S3"); //Escribir S1 S2 S3
        Lcd_Set_Cursor(2,1);    //Cursor en segunda línea
        Lcd_Write_String(conver(AC3, AD3, AU3));    //Escribir los datos para el LCD con esa función
        ADCL(); //Lectura analógica
        if(signo != 13 && signo != 43 && signo != 45){  //Si el caracter ingresado no es + - o enter, no se sumará ni restará
            sum = 0;
            res = 0;
        }
        CONTADOR(); //Contar leer y hacer conversión de datos
        LECT1();    
        LECT2();
        LECT3();
        
    }
}

/***************************************FUNCIONES******************************/
void Setup(void){
    //CONFIG I&0
    ANSEL = 0;
    ANSELH = 0;
    ANSEL = 0b00000011; //Puertos analógicos y digitales
    TRISA = 0b00000011; //Inputs para las señales analógicas
    TRISB = 0;  //Outputs
    TRISC = 0b10000000; //Input para RX
    TRISD = 0; //Outputs
    TRISE = 0; //Outputs
    PORTA = 0; //Potenciometros
    PORTB = 0; 
    PORTC = 0; //RX y TX
    PORTD = 0; //Pines de LCD. (D0 a D7)
    PORTE = 0; //Pines de LCD. (RS, En, RW)
    INTCONbits.TMR0IF = 0; //Interrupciones
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1; //Habilitar interrupciones
    INTCONbits.PEIE = 1; 
    INTCONbits.T0IE = 1; //Interrupción del timer 0
    INTCONbits.RBIE = 0;
    PIR1bits.ADIF = 0; //Función AD lisa para comenzar
    OSCCONbits.IRCF0 = 0; //Configuración del oscilador (4MHz)
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    ADC_init(20, 20);   //Se escoge velocidad y canal para ADC
    ADCON1 = 0;
    TMR0 = 236; //Donde comienza el timer 0
    OPTION_REG = 0b01010111; //Configuración de timer 0 y pull ups
    PIE1bits.ADIE = 1;  //Habilitar ADC
    PIE1bits.RCIE = 1;  //Habilita EUSART Recieve Interrupt
    PIE1bits.TXIE = 1;  //Habilita EUSART Transmit Interrupt
}

void ADCL(void){ //Función de ADC
    if (nowadc > 5){ //Si la variable nowadc es mayor a 5 regresará a 0 y comienza la converción AD
     nowadc = 0;
     ADCON0bits.GO_nDONE = 1;
        if (T==0){ //Se crea un toggle para leer ambos potenciómetros
         ADC_init(1, 20); //Se escoge el canal
         v1 = anlec;  //Se lee señal analógica
         __delay_us(100);
         T = 1;
    }
        else{
         ADC_init(1, 1); //Se escoge el otro canal
         v2 = anlec; //Se lee señal analógica
         __delay_us(100);
         T = 0;
        }    
    }
}
void LECT1(void){ //Para el primer puerto analógica
    Cen1 = v1/51; //Se crea bit de centena de 5V a 0V
    Dec1 = ((10*v1)/51-Cen1*10); //Se crea el bit de decena
    Un1  = (100*v1)/51-(Cen1*100+Dec1*10); //Se crea el bit de unidad
    AC1 = num_ascii(Cen1); //Se hace conversión de números a su forma ascii para que lea la LCD
    AD1 = num_ascii(Dec1);
    AU1 = num_ascii(Un1);
    
}
void LECT2(void){ //Se hace lo mismo para el segundo potenciómetro
    Cen2 = v2/51;
    Dec2 = (((100*v2)/51-(Cen2*100))/10);
    Un2 = (100*v2)/51-(100*Cen2+Dec2*10);
    AC2 = num_ascii(Cen2);
    AD2 = num_ascii(Dec2);
    AU2 = num_ascii(Un2);
}
void LECT3(void){
    Cen3 = CONT/100;    //Se separan los bits del contador
    Dec3 = (CONT-Cen3*100)/10;
    Un3 = (CONT-Cen3*100-Dec3*10);
    AC3 = num_ascii(Cen3);
    AD3 = num_ascii(Dec3);
    AU3 = num_ascii(Un3);
}
const char* conver(char AC3, char AD3, char AU3){   //Datos que recivirá la LCD
    char temporal[16];
    temporal[0] = AC1;
    temporal[1] = 0x2E;
    temporal[2] = AD1;
    temporal[3] = AU1;
    temporal[4] = 0x76;
    temporal[5] = 0x20;
    temporal[6] = AC2;
    temporal[7] = 0x2E;
    temporal[8] = AD2;
    temporal[9] = AU2;
    temporal[10] = 0x76;
    temporal[11] = 0x20;
    temporal[12] = AC3;
    temporal[13] = 0x2E;
    temporal[14] = AD3;
    temporal[15] = AU3;
    return temporal;
}
void envio(void){   //Lectura de la terminal virtual
    toggleTX++;     //Siempre se irá aumentando este contador para que mande datos uno por uno
    if (toggleTX == 1){ //centena potenciómetro 1
        TXREG = AC1;
    }
    if (toggleTX == 2){ //.
        TXREG = 0x2E;
    }
    if (toggleTX == 3){//decena potenciómetro 1
        TXREG = AD1;
    }
    if (toggleTX == 4){ //u P1
        TXREG = AU1;
    }
    if (toggleTX == 5){ //v
        TXREG = 0x76;
    }
    if (toggleTX == 6){ //,
        TXREG = 0x2C;
    }
    if (toggleTX == 7){ //spc
        TXREG = 0x20;
    }
    if (toggleTX == 8){ //cent P2
        TXREG = AC2;
    }
    if (toggleTX == 9){ //.
        TXREG = 0x2E;
    }
    if (toggleTX == 10){ //dec P2
        TXREG = AD2;
    }
    if (toggleTX == 11){ //un P2
        TXREG = AU2;
    }
    if (toggleTX == 12){ //v
        TXREG = 0x76;
    }
    if (toggleTX == 13){ //brk
        TXREG = 13;
        toggleTX = 0;
    }
}
void CONTADOR(void){
    if (signo == 43){   //Si el signo es "+" (43 en código ascii)
        sum = 1;    //Se levantará la bandera de suma
    }
    
    if (signo == 13 && sum == 1){   //Si se apacha enter al estar la bandera de suma levantada
            sum = 0;
            CONT++;                 //El contador se le sumará 1
    }
    
    if (signo == 45){  //Si el signo es -
        res = 1;
    }

    if(signo ==13 && res == 1){ //Y se apacha enter
        res = 0;
        CONT--; //Se restará
    }
}