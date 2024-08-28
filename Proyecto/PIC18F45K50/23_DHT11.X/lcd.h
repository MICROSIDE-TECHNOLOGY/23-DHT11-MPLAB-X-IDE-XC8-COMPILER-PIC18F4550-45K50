#include "mcc_generated_files/device_config.h"
#include "mcc_generated_files/i2c1_master.h"
#include <xc.h>     // include processor files - each processor file is guarded.

#include <stdint.h> //Permite el uso de variables tipo uint8_t

/*******************************************************************************
Definiciones utilizadas en el protocolo I2C y el modulo PCF8574
********************************************************************************/
// Acciones I2C en modo 8 bits
#define I2C_WRITE 0x00
#define I2C_READ 0x01

// Mapa de pines E-RS-LIGHT en el modulo LCD
#define LCD_EN 0b00000100    // LCD En pin @ P2 bit of PCF8574
#define LCD_RS 0b00000001    // LCD Rs pin @ P0 bit of PCF8574
#define LCD_LIGHT 0b00001000 // LCD K pin  @ P3 bit of PCF8574

/*******************************************************************************
Instrucciones que controlan el comportamiento de la pantalla LCD
********************************************************************************/

#define LCD_FunctionSet4bit 0b00101000         // Establece el modo de 4-bit
#define LCD_Reset 0b00110000                   // Instruccion de reinicio
#define LCD_Clear 0b00000001                   // Borra todos los caracteres de la LCD
#define LCD_Home 0b00000010                    // Mueve el cursor a la posicion inicial
#define LCD_EntryMode 0b00000110               // Mueve el cursor de izquierda a derecha
#define LCD_DisplayOff 0b00001000              // Apaga la pantalla LCD
#define LCD_DisplayOn 0b00001100               // Enciende la pantalla LCD
#define LCD_SetCursor 0b10000000               // Indica que se va a mover al cursor
#define LCD_Shift_Left 0b00011000              // Desplaza los caracteres a la izquierda
#define LCD_Shift_Right 0b00011100             // Desplaza los caracteres a la derecha
#define LCD_LineOne 0x00                       // Establece el cursor en la linea 1
#define LCD_LineTwo 0x40                       // Establece el cursor en la linea 2

uint8_t __ADD = 0x27;                          // Direccion predeterminada del modulo PCF8574
uint8_t __LIGHT = 0x08;                        // Pin P3 del PCF8574, controla la luz del LCD

void lcd_i2c_init( uint8_t address );
void lcd_i2c_clear( void );
void lcd_i2c_write( uint8_t byte );
void lcd_i2c_write_instruction( uint8_t byte );
void lcd_i2c_write_byte( char byte );
void lcd_i2c_write_str( char theString[] );
void lcd_i2c_write_int( int value );
void lcd_i2c_write_double( double value, uint8_t decimals );
void lcd_i2c_Backlight( void );
void lcd_i2c_noBacklight( void );
void lcd_i2c_setCursor( uint8_t col, uint8_t row );

static i2c1_operations_t wr1RegCompleteHandler( void *ptr )
{
    I2C1_SetBuffer( ptr, 1 );
    I2C1_SetDataCompleteCallback( NULL, NULL );
    return I2C1_CONTINUE;
}

uint8_t I2C1_Write_Reg( i2c1_address_t address, uint8_t reg, uint8_t data )
{
    while ( !I2C1_Open( address ) )
        ;                                      // sit here until we get the bus..
    I2C1_SetDataCompleteCallback( wr1RegCompleteHandler, &data );
    I2C1_SetBuffer( &reg, 1 );
    I2C1_SetAddressNackCallback( NULL, NULL ); // NACK polling?
    I2C1_MasterWrite();
    while ( I2C1_BUSY == I2C1_Close() )
        ;                                      // sit here until finished.
    return 0;
}

/*******************************************************************************
 Funcion: lcd_i2c_init
 ----------------------
 Inicializa la LCD para funcionar en modo de 4 bits, mediante el expansor de
 puertos I2C de 8 bits PCF8574

 address: Direccion del modulo en formato de 7 bits (0x27)

 retorna: no parametros
********************************************************************************/
void lcd_i2c_init( uint8_t address )
{
    __ADD = address;
    // Secuencia de reinicio de la LCD de acuerdo con la hoja de datos
    lcd_i2c_write( LCD_Reset );
    __delay_us( 4500 );

    lcd_i2c_write( LCD_Reset );
    __delay_us( 4500 );

    lcd_i2c_write( LCD_Reset );
    __delay_us( 150 );

    // Inicia la LCD en modo de 8 bits
    lcd_i2c_write( LCD_FunctionSet4bit );
    __delay_ms( 80 ); // Esta accion requiere de al menos 40 ms

    lcd_i2c_write_instruction( LCD_FunctionSet4bit );
    __delay_ms( 80 ); // Esta accion requiere de al menos 80 ms

    // Parte 2 de la secuencia de inicio
    lcd_i2c_write_instruction( LCD_DisplayOff ); // Apaga la pantalla LCD

    // Limpia la pantalla LCD
    lcd_i2c_clear();

    // Modo de entrada de caracteres
    lcd_i2c_write_instruction( LCD_EntryMode ); // Establece escritura de izquierda a derecha

    // Con esto se termina la secuencia de inicio, pero deja la
    // pantalla de la LCD apagada

    lcd_i2c_write_instruction( LCD_DisplayOn ); // Enciende la pantalla LCD

    return;
}

/*******************************************************************************
 Funcion: lcd_i2c_clear
 -----------------------
 Envia la instruccion LCD_Clear y espera al menos 1.6ms mientras se eliminan
 los caracteres de la memoria RAM de la LCD

 retorna: no parametros
********************************************************************************/
void lcd_i2c_clear()
{
    lcd_i2c_write_instruction( LCD_Clear );
    __delay_ms( 4 ); // Delay de 4 ms
}

/*******************************************************************************
 Funcion: lcd_i2c_noBacklight
 -----------------------------
 Escribe un 0 en el pin P3, lo que deshabilita el pin catodo de la LCD,
 apagando el LED de retroiluminacion

 retorna: no parametros
********************************************************************************/
void lcd_i2c_noBacklight()
{
    __LIGHT = 0;
}

/*******************************************************************************
 Funcion: lcd_i2c_Backlight
 ---------------------------
 Escribe un 1 en el pin P3, lo que habilita el pin Katodo de la LCD,
 encendiendo el LED de retroiluminacion

 retorna: no parametros
********************************************************************************/
void lcd_i2c_Backlight()
{
    __LIGHT = LCD_LIGHT;
}

/*******************************************************************************
 Funcion: lcd_i2c_write
 -----------------------
 Conmuta el pin EN de la LCD, esto permite leer los 4 bits enviados

 retorna: no parametros
********************************************************************************/
void lcd_i2c_write( uint8_t byte )
{
    // Prepara el estado de los pines que se van a leer
    uint8_t payload = byte | __LIGHT;
    I2C1_Write_Reg( __ADD, payload, payload );
    __delay_us( 1 );

    // Escribe 1 en el pin EN de la LCD para comenzar la lectura de los 4 bits
    payload = byte | LCD_EN | __LIGHT;
    I2C1_Write_Reg( __ADD, payload, payload );
    __delay_us( 1 );

    // Escribe un 0 en el pin EN, para prepararla para la siguiente instruccion
    payload = ( byte & ~LCD_EN ) | __LIGHT;
    I2C1_Write_Reg( __ADD, payload, payload );
    __delay_us( 1 );
}

/*******************************************************************************
 Function: lcd_i2c_write_instruction
 -----------------------------------
 Divide 1 byte de la instruccion en un medio-byte mas significativo y un
 medio-byte menos significativo, despues envia uno despues del otro en formato
 de 4 bits

 byte: El byte a enviar

 retorna: no parametros
********************************************************************************/
void lcd_i2c_write_instruction( uint8_t byte )
{
    uint8_t MSHB = ( byte & 0xF0 );
    uint8_t LSHB = ( ( byte << 4 ) & 0xF0 );

    lcd_i2c_write( MSHB );
    lcd_i2c_write( LSHB );

    __delay_us( 80 );
}

/*******************************************************************************
 Function: lcd_i2c_write_byte
 ----------------------------
 Divide 1 byte en un medio-byte mas significativo y un medio-byte menos
 significativo, despues envia uno despues del otro en formato de 4 bits, despues
 escribe un 1 en el pin CS de la LCD, para indicar que se va a enviar un
 caracter en formato ASCII que debe escribirse en la pantalla de la LCD

 byte: El byte a enviar

 retorna: no parametros
********************************************************************************/
void lcd_i2c_write_byte( char byte )
{
    uint8_t MSHB = ( byte & 0xF0 ) | LCD_RS;
    uint8_t LSHB = ( ( byte << 4 ) & 0xF0 ) | LCD_RS;

    lcd_i2c_write( MSHB );
    lcd_i2c_write( LSHB );

    __delay_us( 80 ); // 40 uS delay (min)
}

/*******************************************************************************
 Function: lcd_i2c_write_str
 ---------------------------
 Escribe caracteres en la LCD hasta encontrar un caracter de terminacion ('/0')

 theString: La cadena de caracteres a escribir en la LCD

 retorna: no parametros
********************************************************************************/
void lcd_i2c_write_str( char theString[] )
{
    int i = 0; // contador de caracteres
    while ( theString[i] != 0 ) {
        lcd_i2c_write_byte( theString[i] );
        i++;
    }
}

/*******************************************************************************
 Function: lcd_i2c_write_double
 ---------------------------
 Escribe un valor como flotante en la LCD en su representación como cadena

 value: El valor para escribirse en la pantalla LCD
 decimals: La cantidad de decimales para escribirse en la pantalla

 retorna: no parametros
********************************************************************************/
void lcd_i2c_write_double( double value, uint8_t decimals )
{
    int integer = 0;
    int decimal = 0;
    int decimal_shif = 1;

    for ( uint8_t e_10 = 0; e_10 < decimals; e_10++ ) {
        decimal_shif *= 10;
    }

    integer = (int)value;
    decimal = (int)( ( value - (float)integer ) * decimal_shif );
    if ( decimal < 0 )
        decimal = decimal * -1;

    lcd_i2c_write_int( integer );
    lcd_i2c_write_byte( '.' );
    lcd_i2c_write_int( decimal );
}

/*******************************************************************************
 Function: lcd_i2c_write_int
 ---------------------------
 Escribe un valor entero en la LCD en su representación como cadena

 value: El valor para escribirse en la pantalla LCD

 retorna: no parametros
********************************************************************************/
void lcd_i2c_write_int( int value )
{
    char buf[16] = { 0 };
    uint8_t index = 0;

    if ( value < 0 ) {
        lcd_i2c_write_byte( '-' );
        value = value * -1;
    }

    do {
        buf[index] = '0' + ( value % 10 );
        index++;
        value /= 10;
    } while ( value > 0 );

    for ( uint8_t i = 1; i <= index; i++ ) {
        lcd_i2c_write_byte( buf[index - i] );
    }

    return;
}

/*******************************************************************************
 Function: lcd_i2c_setCursor
 ---------------------------
 Coloca el cursor en la posición columna y fila. Empezando con 1.

 col: Columna
 row: Fila

 retorna: no parametros
********************************************************************************/
void lcd_i2c_setCursor( uint8_t col, uint8_t row )
{
    static const uint8_t lineStartPosition[] = {
        0x00, 0x40, 0x14, 0x54 };
    // Evita leer una dirección fuera del arreglo
    if ( row > 3 )
        return;

    lcd_i2c_write_instruction( LCD_SetCursor | ( col + lineStartPosition[row] ) );
}