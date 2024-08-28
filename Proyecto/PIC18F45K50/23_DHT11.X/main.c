/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F45K50
        Driver Version    :  2.00
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
 */

#define DHT11_DIR TRISBbits.RB2
#define DHT11_IN PORTBbits.RB2
#define DHT11_OUT LATBbits.LATB2

#include "dht_sensor.h"
#include "lcd.h"
#include "mcc_generated_files/mcc.h"

float temperature = 0;
float humidity = 0;
int dht11_stat = 0;

/*
                         Main application
 */
void main( void )
{
    // Initialize the device
    SYSTEM_Initialize();

    srand( 0x100 );

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    // INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    // INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    // INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    // INTERRUPT_PeripheralInterruptDisable();
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
/**
 End of File
 */