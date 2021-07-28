#include "LCD_libreria.h"

void Lcd_Port(char a) {
    PORTD = a;
}

void Lcd_Cmd(char a) {
    Lcd_Port(a);
    RS = 0;
    RW = 0;
    EN = 1; // => E = 1
    __delay_ms(5);
    EN = 0; // => E = 0
}

void Lcd_Clear(void) {
    Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b) {
    char temp;
    if (a == 1) {  //Move cursor to display
        temp = 0x80 + b - 1;
        Lcd_Cmd(temp);
    } else if (a == 2) {  //Move cursor into CGRAM
        temp = 0xC0 + b - 1;
        Lcd_Cmd(temp);
    }
}

void Lcd_Init(void) { //Inicialización
    //Lcd_Port(0x030);
    //__delay_ms(5);
    //Lcd_Cmd(0x030);
    //__delay_us(160);
    //Lcd_Cmd(0x030);
    //__delay_us(160);
    /////////////////////////////////////////////////////
    Lcd_Cmd(0x38);  //Interface length
    Lcd_Cmd(0x06);
    Lcd_Cmd(0x0c);
    Lcd_Cmd(0x80);
}

void Lcd_Write_Char(char a) {
    Lcd_Port(a); //Data transfer
    RS = 1; // => RS = 1
    RW = 0;
    EN = 1;
    __delay_us(40);
    EN = 0;
}

void Lcd_Write_String(char *a) {
    int i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right(void) {
    Lcd_Cmd(0x1C);
}
void Lcd_Shift_Left(void) {
    Lcd_Cmd(0x18);
}