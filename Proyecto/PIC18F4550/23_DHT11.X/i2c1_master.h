/* Microchip Technology Inc. and its subsidiaries.  You may use this software
 * and any derivatives exclusively with Microchip products.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
 * TERMS.
 */

/*
 * File:
 * Author:
 * Comments:
 * Revision history:
 */

#ifndef I2C1_MASTER_H
#define I2C1_MASTER_H

#include <xc.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 48000000
#endif

#ifdef __cplusplus
extern "C" {
#endif

void I2C1_Initialize( void );
void I2C1_Wait( void );
void I2C1_Start();
void I2C1_Stop();
void I2C1_RepeatedStart();
void I2C1_SendAck();
void I2C1_SendNack();
inline uint8_t I2C1_IsNack();
uint8_t I2C1_Write( uint8_t byte );
uint8_t I2C1_WriteSlave( uint8_t address, uint8_t *pBuffer, size_t len );
uint8_t I2C1_Read();
uint8_t I2C1_ReadSlave( uint8_t address, uint8_t *pBuffer, size_t len );

/*******************************************************************************
 Funcion: I2C1_Initialize
 ----------------------
 Inicializa los registros del periférico MSSP en configuración I2C a 100kHz

 retorna: no parametros
********************************************************************************/
void I2C1_Initialize( void )
{
    SSPCON1 = 0x28;
    SSPCON2 = 0x00;
    SSPSTAT = 0x00;
    SSPADD = ( ( _XTAL_FREQ / 4 ) / 100000 - 1 ); // 100kHz

    // SDA & SCL
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
}

/*******************************************************************************
 Funcion: I2C1_Wait
 ----------------------
 Espera hasta que el periférico MSSP regrese a estado IDLE

 retorna: no parametros
********************************************************************************/
void I2C1_Wait( void )
{
    while ( SSPSTATbits.R_NOT_W | ( SSPCON2 & 0x1F ) )
        ;
}

/*******************************************************************************
 Funcion: I2C1_Start
 ----------------------
 Genera una condición de inicio I2C

 retorna: no parametros
********************************************************************************/
void I2C1_Start()
{
    I2C1_Wait();
    SSPCON2bits.SEN = 1;
}

/*******************************************************************************
 Funcion: I2C1_Stop
 ----------------------
 Genera una condición de parada

 retorna: no parametros
********************************************************************************/
void I2C1_Stop()
{
    I2C1_Wait();
    SSPCON2bits.PEN = 1;
}

/*******************************************************************************
 Funcion: I2C1_RepeatedStart
 ----------------------
 Genera una condición de reinicio (Repeated start)

 retorna: no parametros
********************************************************************************/
void I2C1_RepeatedStart()
{
    I2C1_Wait();
    SSPCON2bits.RSEN = 1;
}

/*******************************************************************************
 Funcion: I2C1_SendAck
 ----------------------
 Genera una señal de acknowledgment

 retorna: no parametros
********************************************************************************/
void I2C1_SendAck()
{
    I2C1_Wait();
    SSPCON2bits.ACKDT = 0;
    SSPCON2bits.ACKEN = 1;
}

/*******************************************************************************
 Funcion: I2C1_SendNack
 ----------------------
 Genera una señal de not acknowledgment

 retorna: no parametros
********************************************************************************/
void I2C1_SendNack()
{
    I2C1_Wait();
    SSPCON2bits.ACKDT = 1;
    SSPCON2bits.ACKEN = 1;
}

inline uint8_t I2C1_IsNack()
{
    return SSPCON2bits.ACKSTAT;
}

/*******************************************************************************
 Funcion: I2C1_Write
 ----------------------
 Escribe un byte en el BUS I2C

 byte: Datos a escribir en el BUS I2C

 retorna: 0 en exito, 1 en error
********************************************************************************/
uint8_t I2C1_Write( uint8_t byte )
{
    I2C1_Wait();
    SSPBUF = byte;
    I2C1_Wait();
    return SSPCON2bits.ACKSTAT;
}

uint8_t I2C1_Read()
{
    uint8_t pData = 0;
    I2C1_Wait();
    SSPCON2bits.RCEN = 1;
    I2C1_Wait();
    pData = SSPBUF;
    return pData;
}

/*******************************************************************************
 Funcion: I2C1_WriteSlave
 ----------------------
 Envía n bytes a la dirección I2C

 address: La dirección I2C a escribir
 pBuffer: Dirección de memoría de los bytes a escribir
 len: Tamaño del arreglo de bytes a escribir

 retorna: 0 en éxito, 1 si no se enconttró la dirección I2C en el BUS, 2 en error
 de transmisión
********************************************************************************/
uint8_t I2C1_WriteSlave( uint8_t address, uint8_t *pBuffer, size_t len )
{
    I2C1_Start();
    if ( I2C1_Write( (uint8_t)( address << 1 ) ) ) {
        I2C1_Stop();
        return 1;
    }

    for ( size_t i = 0; i < len; i++ ) {
        if ( I2C1_Write( pBuffer[i] ) ) {
            I2C1_Stop();
            return 2;
        }
    }
    I2C1_Stop();
    return 0;
}

/*******************************************************************************
 Funcion: I2C1_WriteSlave
 ----------------------
 Solicita n bytes de la dirección I2C

 address: La dirección I2C a leer
 pBuffer: Dirección de memoría de los bytes a leer
 len: Tamaño del arreglo de bytes a leer

 retorna: 0 en éxito, 1 si no se enconttró la dirección I2C en el BUS
********************************************************************************/
uint8_t I2C1_ReadSlave( uint8_t address, uint8_t *pBuffer, size_t len )
{
    I2C1_Start();
    if ( I2C1_Write( (uint8_t)( address << 1 ) + 0x01 ) ) {
        I2C1_Stop();
        return 1;
    }

    for ( size_t i = 0; i < len; ) {
        pBuffer[i] = I2C1_Read();
        if ( i++ < len ) {
            I2C1_SendAck();
        } else {
            I2C1_SendNack();
        }
    }
    I2C1_Stop();
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* I2C1_MASTER_H */
