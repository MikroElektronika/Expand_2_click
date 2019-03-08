/*
    __expand2_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__expand2_driver.h"
#include "__expand2_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __EXPAND2_DRV_I2C__
static uint8_t _slaveAddress;
#endif

const uint8_t _EXPAND2_I2C_ADDRESS                                       = 0x20;

const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_0                             = 0x00;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_1                             = 0x01;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_2                             = 0x02;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_3                             = 0x03;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_4                             = 0x04;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_5                             = 0x05;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_6                             = 0x06;
const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_7                             = 0x07;

// Port Direction
const uint8_t _EXPAND2_PORT_DIRECTION_OUTPUT                             = 0x00;
const uint8_t _EXPAND2_PORT_DIRECTION_INPUT                              = 0xFF;

// BANK 1 register configuration
const uint8_t _EXPAND2_IODIRA_BANK1                                      = 0x00;
const uint8_t _EXPAND2_IPOLA_BANK1                                       = 0x01;
const uint8_t _EXPAND2_GPINTENA_BANK1                                    = 0x02;
const uint8_t _EXPAND2_DEFVALA_BANK1                                     = 0x03;
const uint8_t _EXPAND2_INTCONA_BANK1                                     = 0x04;
const uint8_t _EXPAND2_IOCON_BANK1                                       = 0x05;
const uint8_t _EXPAND2_GPPUA_BANK1                                       = 0x06;
const uint8_t _EXPAND2_INTFA_BANK1                                       = 0x07;
const uint8_t _EXPAND2_INTCAPA_BANK1                                     = 0x08;
const uint8_t _EXPAND2_GPIOA_BANK1                                       = 0x09;
const uint8_t _EXPAND2_OLATA_BANK1                                       = 0x0A;
const uint8_t _EXPAND2_IODIRB_BANK1                                      = 0x10;
const uint8_t _EXPAND2_IPOLB_BANK1                                       = 0x11;
const uint8_t _EXPAND2_GPINTENB_BANK1                                    = 0x12;
const uint8_t _EXPAND2_DEFVALB_BANK1                                     = 0x13;
const uint8_t _EXPAND2_INTCONB_BANK1                                     = 0x14;
const uint8_t _EXPAND2_IOCONO_BANK1                                      = 0x15;
const uint8_t _EXPAND2_GPPUB_BANK1                                       = 0x16;
const uint8_t _EXPAND2_INTFB_BANK1                                       = 0x17;
const uint8_t _EXPAND2_INTCAPB_BANK1                                     = 0x18;
const uint8_t _EXPAND2_GPIOB_BANK1                                       = 0x19;
const uint8_t _EXPAND2_OLATB_BANK1                                       = 0x1A;

// BANK 0 register configuration
const uint8_t _EXPAND2_IODIRA_BANK0                                      = 0x00;
const uint8_t _EXPAND2_IODIRB_BANK0                                      = 0x01;
const uint8_t _EXPAND2_IPOLA_BANK0                                       = 0x02;
const uint8_t _EXPAND2_IPOLB_BANK0                                       = 0x03;
const uint8_t _EXPAND2_GPINTENA_BANK0                                    = 0x04;
const uint8_t _EXPAND2_GPINTENB_BANK0                                    = 0x05;
const uint8_t _EXPAND2_DEFVALA_BANK0                                     = 0x06;
const uint8_t _EXPAND2_DEFVALB_BANK0                                     = 0x07;
const uint8_t _EXPAND2_INTCONA_BANK0                                     = 0x08;
const uint8_t _EXPAND2_INTCONB_BANK0                                     = 0x09;
const uint8_t _EXPAND2_IOCON_BANK0                                       = 0x0A;
const uint8_t _EXPAND2_GPPUA_BANK0                                       = 0x0C;
const uint8_t _EXPAND2_GPPUB_BANK0                                       = 0x0D;
const uint8_t _EXPAND2_INTFA_BANK0                                       = 0x0E;
const uint8_t _EXPAND2_INTFB_BANK0                                       = 0x0F;
const uint8_t _EXPAND2_INTCAPA_BANK0                                     = 0x10;
const uint8_t _EXPAND2_INTCAPB_BANK0                                     = 0x11;
const uint8_t _EXPAND2_GPIOA_BANK0                                       = 0x12;
const uint8_t _EXPAND2_GPIOB_BANK0                                       = 0x13;
const uint8_t _EXPAND2_OLATA_BANK0                                       = 0x14;
const uint8_t _EXPAND2_OLATB_BANK0                                       = 0x15;

const uint8_t _EXPAND2_HD1_PA0                                           = 0x01;
const uint8_t _EXPAND2_HD1_PA1                                           = 0x02;
const uint8_t _EXPAND2_HD1_PA2                                           = 0x04;
const uint8_t _EXPAND2_HD1_PA3                                           = 0x08;
const uint8_t _EXPAND2_HD1_PA4                                           = 0x10;
const uint8_t _EXPAND2_HD1_PA5                                           = 0x20;
const uint8_t _EXPAND2_HD1_PA6                                           = 0x40;
const uint8_t _EXPAND2_HD1_PA7                                           = 0x80;

const uint8_t _EXPAND2_HD2_PB0                                           = 0x01;
const uint8_t _EXPAND2_HD2_PB1                                           = 0x02;
const uint8_t _EXPAND2_HD2_PB2                                           = 0x04;
const uint8_t _EXPAND2_HD2_PB3                                           = 0x08;
const uint8_t _EXPAND2_HD2_PB4                                           = 0x10;
const uint8_t _EXPAND2_HD2_PB5                                           = 0x20;
const uint8_t _EXPAND2_HD2_PB6                                           = 0x40;
const uint8_t _EXPAND2_HD2_PB7                                           = 0x80;

const uint8_t _EXPAND2_HD_START_POSITION                                 = 0x01;

const uint8_t _EXPAND2_INT_ERR                                           = 0xFF;


/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __EXPAND2_DRV_SPI__

void expand2_spiDriverInit(T_EXPAND2_P gpioObj, T_EXPAND2_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __EXPAND2_DRV_I2C__

void expand2_i2cDriverInit(T_EXPAND2_P gpioObj, T_EXPAND2_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __EXPAND2_DRV_UART__

void expand2_uartDriverInit(T_EXPAND2_P gpioObj, T_EXPAND2_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif



/* ----------------------------------------------------------- IMPLEMENTATION */


/* Generic read one bayt from register function */
uint8_t expand2_readByte( uint8_t moduleAddress, uint8_t regAddress )
{
    uint8_t bufferRead[ 1 ];
    uint8_t bufferWrite[ 1 ];

    bufferWrite[ 0 ] = regAddress;

    _slaveAddress |= moduleAddress;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, bufferWrite, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, bufferRead, 1, END_MODE_STOP );

    return bufferRead[ 0 ];
}

/* Generic write one bayt to register function */
void expand2_writeByte( uint8_t moduleAddress, uint8_t regAddress, uint8_t writeData )
{
    uint8_t bufferWrite[ 2 ];

    bufferWrite[ 0 ] = regAddress;
    bufferWrite[ 1 ] = writeData;
    
    _slaveAddress |= moduleAddress;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, bufferWrite, 2, END_MODE_STOP );
}

/* Set register bits function */
void expand2_setBits( uint8_t moduleAddress, uint8_t regAddress, uint8_t bitMask )
{
    uint8_t temp;

    temp = expand2_readByte( moduleAddress, regAddress );
    
    temp |= bitMask;
    
    expand2_writeByte( moduleAddress, regAddress, temp );
}

/*  Clear register beats function */
void expand2_clearBits( uint8_t moduleAddress, uint8_t regAddress, uint8_t bitMask )
{
    uint8_t temp;

    temp = expand2_readByte( moduleAddress, regAddress );
    
    temp &= ~bitMask;
    
    expand2_writeByte( moduleAddress, regAddress, temp );
}

/*  Toggle register beats function */
void expand2_toggleBits( uint8_t moduleAddress, uint8_t regAddress, uint8_t bitMask )
{
    uint8_t temp;

    temp = expand2_readByte( moduleAddress, regAddress );
    
    temp ^= bitMask;
    
    expand2_writeByte( moduleAddress, regAddress, temp );
}

/* Read one byte of data from PORTA function */
uint8_t expand2_readPortA( uint8_t moduleAddress )
{
    return expand2_readByte( moduleAddress, _EXPAND2_GPIOA_BANK0 );
}

/* Read one byte of data from PORTB function */
uint8_t expand2_readPortB( uint8_t moduleAddress )
{
    return expand2_readByte( moduleAddress, _EXPAND2_GPIOB_BANK0);
}

/* Read two byte of data from PORTA & PORTB function */
uint16_t expand2_readBothPorta( uint8_t moduleAddress )
{
    uint16_t result;
    uint8_t buffer[ 2 ];
    
    buffer[ 0 ] = expand2_readPortA( moduleAddress );
    buffer[ 1 ] = expand2_readPortB( moduleAddress );
    
    result = buffer[ 0 ];
    result <<= 8;
    result |= buffer[ 1 ];

    return result;
}

/* Write one byte of data to register for PORTA function */
void expand2_writePortA( uint8_t moduleAddress, uint8_t writeData )
{
    expand2_writeByte( moduleAddress, _EXPAND2_OLATA_BANK0, writeData );
}

/* Clear bit to register for PORTA function */
void expand2_clearBitPortA( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_clearBits( moduleAddress, _EXPAND2_OLATA_BANK0, bitMask );
}

/* Set bit to register for PORTA function */
void expand2_setBitPortA( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_setBits( moduleAddress, _EXPAND2_OLATA_BANK0, bitMask );
}

/* Toggle bit to register for PORTA function */
void expand2_toggleBitPortA( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_toggleBits( moduleAddress, _EXPAND2_OLATA_BANK0, bitMask );
}

/* Write one byte of data to register for PORTB function */
void expand2_writePortB( uint8_t moduleAddress, uint8_t writeData )
{
    expand2_writeByte( moduleAddress, _EXPAND2_OLATB_BANK0, writeData );
}

/* Clear bit to register for PORTB function */
void expand2_clearBitPortB( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_clearBits( moduleAddress, _EXPAND2_OLATB_BANK0, bitMask );
}

/* Set bit to register for PORTB function */
void expand2_setBitPortB( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_setBits( moduleAddress, _EXPAND2_OLATB_BANK0, bitMask );
}

/* Toggle bit to register for PORTB function */
void expand2_toggleBitPortB( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_toggleBits( moduleAddress, _EXPAND2_OLATB_BANK0, bitMask );
}

/* Set expander PORTA direction function */
void expand2_setDirectionPortA( uint8_t moduleAddress, uint8_t writeData )
{
    expand2_writeByte( moduleAddress, _EXPAND2_IODIRA_BANK0, writeData );
}

/* Set expander PORTA input direction function */
void expand2_setInputDirPortA( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_setBits( moduleAddress, _EXPAND2_IODIRA_BANK0, bitMask );
}

/* Set expander PORTA output direction function */
void expand2_setOutputDirPortA( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_clearBits( moduleAddress, _EXPAND2_IODIRA_BANK0, bitMask );
}

/* Set expander PORTB direction function */
void expand2_setDirectionPortB( uint8_t moduleAddress, uint8_t writeData )
{
    expand2_writeByte( moduleAddress, _EXPAND2_IODIRB_BANK0, writeData );
}

/* Set expander PORTB input direction function */
void expand2_setInputDirPortB( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_setBits( moduleAddress, _EXPAND2_IODIRB_BANK0, bitMask );
}

/* Set expander PORTB output direction function */
void expand2_setOutputDirPortB( uint8_t moduleAddress, uint8_t bitMask )
{
    expand2_clearBits( moduleAddress, _EXPAND2_IODIRB_BANK0, bitMask );
}

/*  Set pull-ups of the expander for PORTA pins function */
void expand2_setPullUpsPortA( uint8_t moduleAddress, uint8_t writeData )
{
    expand2_writeByte( moduleAddress, _EXPAND2_GPPUA_BANK0, writeData );
}

/*  Set pull-ups of the expander for PORTB pins function */
void expand2_setPullUpsPortB( uint8_t moduleAddress, uint8_t writeData )
{
    expand2_writeByte(ModuleAddress, _EXPAND2_GPPUB_BANK0, writeData );
}

/*  Active pin by position on PORTA function */
void expand2_setPotrA( uint8_t position )
{
    uint8_t writeData;
    
    position %= 8;
    
    writeData = 0x01 << position;
    
    expand2_writePortA( _EXPAND2_I2C__MODULE_ADDRESS_5, writeData );
}

/*  Active pin by position on PORTB function */
void expand2_setPotrB( uint8_t position )
{
    uint8_t writeData;

    position %= 8;

    writeData = 0x01 << position;

    expand2_writePortB( _EXPAND2_I2C__MODULE_ADDRESS_5, writeData );
}

/* Reset function */
void expand2_reset()
{
    hal_gpio_rstSet( 1 );
    Delay_5ms();
    hal_gpio_rstSet( 0 );
    Delay_5ms();
    hal_gpio_rstSet( 1 );
    Delay_1ms();
}

/* Get state of interrupt pin function */
uint8_t expand2_getInterrupt()
{
    return hal_gpio_intGet();
}



/* -------------------------------------------------------------------------- */
/*
  __expand2_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */