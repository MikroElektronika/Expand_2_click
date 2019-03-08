/*
    __expand2_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __expand2_driver.h
@brief    Expand_2 Driver
@mainpage Expand_2 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   EXPAND2
@brief      Expand_2 Click Driver
@{

| Global Library Prefix | **EXPAND2** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Nov 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _EXPAND2_H_
#define _EXPAND2_H_

/** 
 * @macro T_EXPAND2_P
 * @brief Driver Abstract type 
 */
#define T_EXPAND2_P    const uint8_t*

/** @defgroup EXPAND2_COMPILE Compilation Config */              /** @{ */

//  #define   __EXPAND2_DRV_SPI__                            /**<     @macro __EXPAND2_DRV_SPI__  @brief SPI driver selector */
   #define   __EXPAND2_DRV_I2C__                            /**<     @macro __EXPAND2_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __EXPAND2_DRV_UART__                           /**<     @macro __EXPAND2_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup EXPAND2_VAR Variables */                           /** @{ */

/** I2C address */
extern const uint8_t _EXPAND2_I2C_ADDRESS;

/**
 * I2C module 0 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_0;
/**
 * I2C module 1 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_1;
/**
 * I2C module 2 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_2;
/**
 * I2C module 3 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 0;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_3;
/**
 * I2C module 4 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_4;
/**
 * I2C module 5 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 0;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_5;
/**
 * I2C module 6 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 0;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_6;
/**
 * I2C module 7 address
 *
 * Jumper position:
 * - J3 ( A0 ) set 1;
 * - J4 ( A1 ) set 1;
 * - J5 ( A2 ) set 1;
 */
extern const uint8_t _EXPAND2_I2C__MODULE_ADDRESS_7;

extern const uint8_t _EXPAND2_PORT_DIRECTION_OUTPUT;
extern const uint8_t _EXPAND2_PORT_DIRECTION_INPUT;
extern const uint8_t _EXPAND2_IODIRA_BANK1;
extern const uint8_t _EXPAND2_IPOLA_BANK1;
extern const uint8_t _EXPAND2_GPINTENA_BANK1;
extern const uint8_t _EXPAND2_DEFVALA_BANK1;
extern const uint8_t _EXPAND2_INTCONA_BANK1;
extern const uint8_t _EXPAND2_IOCON_BANK1;
extern const uint8_t _EXPAND2_GPPUA_BANK1;
extern const uint8_t _EXPAND2_INTFA_BANK1;
extern const uint8_t _EXPAND2_INTCAPA_BANK1;
extern const uint8_t _EXPAND2_GPIOA_BANK1;
extern const uint8_t _EXPAND2_OLATA_BANK1;
extern const uint8_t _EXPAND2_IODIRB_BANK1;
extern const uint8_t _EXPAND2_IPOLB_BANK1;
extern const uint8_t _EXPAND2_GPINTENB_BANK1;
extern const uint8_t _EXPAND2_DEFVALB_BANK1;
extern const uint8_t _EXPAND2_INTCONB_BANK1;
extern const uint8_t _EXPAND2_IOCONO_BANK1;
extern const uint8_t _EXPAND2_GPPUB_BANK1;
extern const uint8_t _EXPAND2_INTFB_BANK1;
extern const uint8_t _EXPAND2_INTCAPB_BANK1;
extern const uint8_t _EXPAND2_GPIOB_BANK1;
extern const uint8_t _EXPAND2_OLATB_BANK1;
extern const uint8_t _EXPAND2_IODIRA_BANK0;
extern const uint8_t _EXPAND2_IODIRB_BANK0;
extern const uint8_t _EXPAND2_IPOLA_BANK0;
extern const uint8_t _EXPAND2_IPOLB_BANK0;
extern const uint8_t _EXPAND2_GPINTENA_BANK0;
extern const uint8_t _EXPAND2_GPINTENB_BANK0;
extern const uint8_t _EXPAND2_DEFVALA_BANK0;
extern const uint8_t _EXPAND2_DEFVALB_BANK0;
extern const uint8_t _EXPAND2_INTCONA_BANK0;
extern const uint8_t _EXPAND2_INTCONB_BANK0;
extern const uint8_t _EXPAND2_IOCON_BANK0;
extern const uint8_t _EXPAND2_GPPUA_BANK0;
extern const uint8_t _EXPAND2_GPPUB_BANK0;
extern const uint8_t _EXPAND2_INTFA_BANK0;
extern const uint8_t _EXPAND2_INTFB_BANK0;
extern const uint8_t _EXPAND2_INTCAPA_BANK0;
extern const uint8_t _EXPAND2_INTCAPB_BANK0;
extern const uint8_t _EXPAND2_GPIOA_BANK0;
extern const uint8_t _EXPAND2_GPIOB_BANK0;
extern const uint8_t _EXPAND2_OLATA_BANK0;
extern const uint8_t _EXPAND2_OLATB_BANK0;
extern const uint8_t _EXPAND2_HD1_PA0;
extern const uint8_t _EXPAND2_HD1_PA1;
extern const uint8_t _EXPAND2_HD1_PA2;
extern const uint8_t _EXPAND2_HD1_PA3;
extern const uint8_t _EXPAND2_HD1_PA4;
extern const uint8_t _EXPAND2_HD1_PA5;
extern const uint8_t _EXPAND2_HD1_PA6;
extern const uint8_t _EXPAND2_HD1_PA7;
extern const uint8_t _EXPAND2_HD2_PB0;
extern const uint8_t _EXPAND2_HD2_PB1;
extern const uint8_t _EXPAND2_HD2_PB2;
extern const uint8_t _EXPAND2_HD2_PB3;
extern const uint8_t _EXPAND2_HD2_PB4;
extern const uint8_t _EXPAND2_HD2_PB5;
extern const uint8_t _EXPAND2_HD2_PB6;
extern const uint8_t _EXPAND2_HD2_PB7;
extern const uint8_t _EXPAND2_HD_START_POSITION;
extern uint8_t _EXPAND2_INT_ERR;
                                                                       /** @} */
/** @defgroup EXPAND2_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup EXPAND2_INIT Driver Initialization */              /** @{ */

#ifdef   __EXPAND2_DRV_SPI__
void expand2_spiDriverInit(T_EXPAND2_P gpioObj, T_EXPAND2_P spiObj);
#endif
#ifdef   __EXPAND2_DRV_I2C__
void expand2_i2cDriverInit(T_EXPAND2_P gpioObj, T_EXPAND2_P i2cObj, uint8_t slave);
#endif
#ifdef   __EXPAND2_DRV_UART__
void expand2_uartDriverInit(T_EXPAND2_P gpioObj, T_EXPAND2_P uartObj);
#endif


/** @defgroup EXPAND2_FUNC Driver Functions */                   /** @{ */


/**
 * @brief Generic read one bayt from register function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] regAddress                        register address
 *
 * Function read 8-bit of data from 8-bit register address of MCP23017 chip.
 */
uint8_t expand2_readByte( uint8_t moduleAddress, uint8_t regAddress );

/**
 * @brief Generic write one bayt to register function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] regAddress                        register address
 *
 * @param[in] writeData                         data to write to register
 *
 * Function write 8-bit of data to 8-bit register address of MCP23017 chip.
 */
void expand2_writeByte( uint8_t moduleAddress, uint8_t regAddress, uint8_t writeData );

/**
 * @brief Set register bits function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] regAddress                        register address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function set bits to 8-bit register address of MCP23017 chip.
 */
void expand2_setBits( uint8_t moduleAddress, uint8_t regAddress, uint8_t bitMask );

/**
 * @brief Clear register bits function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] regAddress                        register address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function clear bits from 8-bit register address of MCP23017 chip.
 */
void expand2_clearBits( uint8_t moduleAddress, uint8_t regAddress, uint8_t bitMask );

/**
 * @brief Toggle register bits function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] regAddress                        register address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function toggle bits from 8-bit register address of MCP23017 chip.
 */
void expand2_toggleBits( uint8_t moduleAddress, uint8_t regAddress, uint8_t bitMask );

/**
 * @brief Read one byte of data from PORTA function
 *
 * @param[in] moduleAddress                     module address
 *
 * @return result                               read data ( PORTA )
 *
 * Function read 8-bit of data from PORTA from 8-bit register address of MCP23017 chip.
 */
uint8_t expand2_readPortA( uint8_t moduleAddress );

/**
 * @brief Read one byte of data from PORTB function
 *
 * @param[in] moduleAddress                     module address
 *
 * @return result                               read data ( PORTB )
 *
 * Function read 8-bit of data from PORTB from 8-bit register address of MCP23017 chip.
 */
uint8_t expand2_readPortB( uint8_t moduleAddress );

/**
 * @brief Read two byte of data from PORTA & PORTB function
 *
 * @param[in] moduleAddress                     module address
 *
 * @return result                               read data ( PORTA & PORTB )
 *
 * Function read 16-bit of data from PORTA & PORTB from 8-bit register address of MCP23017 chip.
 */
uint16_t expand2_readBothPorta( uint8_t moduleAddress );

/**
 * @brief Write one byte of data to register for PORTA function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         data to write
 *
 * Function write 8-bit of data to 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_writePortA( uint8_t moduleAddress, uint8_t writeData );

/**
 * @brief Clear bit from register for PORTA function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function clear bit from 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_clearBitPortA( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set bit to register for PORTA function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function set bit to 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_setBitPortA( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Toggle bit to register for PORTA function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function toggle bit from 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_toggleBitPortA( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Write one byte of data to register for PORTB function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         data to write
 *
 * Function write 8-bit of data 
 * to 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_writePortB( uint8_t moduleAddress, uint8_t writeData );

/**
 * @brief Clear bit from register for PORTB function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function clear bit from 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_clearBitPortB( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set bit to register for PORTB function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function set bit to 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_setBitPortB( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Toggle bit to register for PORTB function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bits mask
 *
 * Function toggle bit from 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_toggleBitPortB( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set expander PORTA direction function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         data to write
 *
 * Function set expander direction by write 8-bit data
 * to 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_setDirectionPortA( uint8_t moduleAddress, uint8_t writeData );

/**
 * @brief Set expander PORTA input direction function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTA set as input,
 * to 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_setInputDirPortA( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set expander PORTA output direction function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTA set as output,
 * to 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_setOutputDirPortA( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set expander PORTB direction function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         data to write
 *
 * Function set expander direction by write 8-bit data
 * to 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_setDirectionPortB( uint8_t moduleAddress, uint8_t writeData );

/**
 * @brief Set expander PORTB input direction function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTB set as input,
 * to 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_setInputDirPortB( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set expander PORTB output direction function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] bitMask                           bit mask
 *
 * Function write bit, when expander direction of PORTB set as output,
 * to 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_setOutputDirPortB( uint8_t moduleAddress, uint8_t bitMask );

/**
 * @brief Set pull-ups of the expander for PORTA pins function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         pull up value
 *
 * Function set pull-ups of the expander for PORTA pins
 * by write 8-bit pull up value data
 * to 8-bit register address from PORTA of MCP23017 chip.
 */
void expand2_setPullUpsPortA( uint8_t moduleAddress, uint8_t writeData );

/**
 * @brief Set pull-ups of the expander for PORTB pins function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         pull up value
 *
 * Function set pull-ups of the expander for PORTB pins
 * by write 8-bit pull up value data
 * to 8-bit register address from PORTB of MCP23017 chip.
 */
void expand2_setPullUpsPortB( uint8_t moduleAddress, uint8_t writeData );

/**
 * @brief Active pin by position on PORTA function
 *
 * @param[in] position                          pin position
 *
 * Function activate pin on PORTA by position, from PA0 to PA7.
 */
void expand2_setPotrA( uint8_t position );

/**
 * @brief Active pin by position on PORTB function
 *
 * @param[in] position                          pin position
 *
 * Function activate pin on PORTB by position, from PB0 to PB7.
 */
void expand2_setPotrB( uint8_t position );

/**
 * @brief Reset function
 *
 * @param[in] moduleAddress                     module address
 *
 * @param[in] writeData                         pull up value
 *
 * Function reset Expand 2 click by set RST pin from low to high.
 *
 * @note    
 * delay is 11ms
 */
void expand2_reset();

/**
 * @brief Get state of interrupt pin function
 *
 * @return state
 * 0 - No Active, 1 - Active
 *
 * Function get state of interrupt ( INT ) pin.
 */
uint8_t expand2_getInterrupt();



                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Expand_2_STM.c
    @example Click_Expand_2_TIVA.c
    @example Click_Expand_2_CEC.c
    @example Click_Expand_2_KINETIS.c
    @example Click_Expand_2_MSP.c
    @example Click_Expand_2_PIC.c
    @example Click_Expand_2_PIC32.c
    @example Click_Expand_2_DSPIC.c
    @example Click_Expand_2_AVR.c
    @example Click_Expand_2_FT90x.c
    @example Click_Expand_2_STM.mbas
    @example Click_Expand_2_TIVA.mbas
    @example Click_Expand_2_CEC.mbas
    @example Click_Expand_2_KINETIS.mbas
    @example Click_Expand_2_MSP.mbas
    @example Click_Expand_2_PIC.mbas
    @example Click_Expand_2_PIC32.mbas
    @example Click_Expand_2_DSPIC.mbas
    @example Click_Expand_2_AVR.mbas
    @example Click_Expand_2_FT90x.mbas
    @example Click_Expand_2_STM.mpas
    @example Click_Expand_2_TIVA.mpas
    @example Click_Expand_2_CEC.mpas
    @example Click_Expand_2_KINETIS.mpas
    @example Click_Expand_2_MSP.mpas
    @example Click_Expand_2_PIC.mpas
    @example Click_Expand_2_PIC32.mpas
    @example Click_Expand_2_DSPIC.mpas
    @example Click_Expand_2_AVR.mpas
    @example Click_Expand_2_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __expand2_driver.h

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