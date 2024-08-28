/*
 * File:   main.c
 * Author: Microside Technology
 *
 * Created on 27 de noviembre de 2020, 01:38 AM
 */

// CONFIG1L
#pragma config PLLDIV = 5         // PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 2         // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes from the 96 MHz PLL divided by 2)

// CONFIG1H
#pragma config FOSC = HSPLL_HS // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FCMEN = ON      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF      // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON   // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = ON    // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 1    // Brown-out Reset Voltage bits (Setting 2 4.33V)
#pragma config VREGEN = ON // USB Voltage Regulator Enable bit (USB voltage regulator enabled)

// CONFIG2H
#pragma config WDT = OFF     // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768 // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF  // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON    // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF   // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define DHT11_DIR TRISBbits.RB2
#define DHT11_IN PORTBbits.RB2
#define DHT11_OUT LATBbits.LATB2

#define _XTAL_FREQ 48000000
#include "LCD.h"
#include "i2c1_master.h"
#include "dht_sensor.h"
#include <pic18f4550.h>
#include <xc.h>

float temperature = 0;
float humidity = 0;
int dht11_stat = 0;

void main( void )
{
    ADCON1 = 0x0F;

    I2C1_Initialize();
    __delay_ms( 1000 );
    
    lcd_i2c_init( 0x27 ); // Inicializa la pantalla LCD

    lcd_i2c_clear();
    lcd_i2c_setCursor( 3, 0 );
    lcd_i2c_write_str( "DHT11" );
    lcd_i2c_setCursor( 3, 1 );

    if ( dht11_init() != 0 ) { // Inicializa el DHT11
        lcd_i2c_write_str( "ERROR" );
        while ( 1 ) {
        }
    }

    lcd_i2c_write_str( "OK" );
    __delay_ms( 1000 );

    while ( 1 ) {
        __delay_ms( 2000 );
        // Lee el sensor DHT11
        dht11_stat = dht11_read( &temperature, &humidity );
        // Cualquier valor diferente a 0 significa error en la lectura
        if ( dht11_stat != 0 ) {
            lcd_i2c_clear();
            lcd_i2c_setCursor( 0, 0 );
            lcd_i2c_write_str( "DHT11 Error" );
            continue;
        }

        lcd_i2c_clear();
        lcd_i2c_setCursor( 0, 0 );
        lcd_i2c_write_str( "T: " );
        lcd_i2c_write_double( temperature, 2 );
        lcd_i2c_write_str( " C" );
        lcd_i2c_setCursor( 0, 1 );
        lcd_i2c_write_str( "H: " );
        lcd_i2c_write_double( humidity, 2 );
        lcd_i2c_write_str( " %" );
    }
}