/*
Example for Expand_2 Click

    Date          : Nov 2018.
    Author        : Nenad Filipovic

Test configuration STM32 :
    
    MCU              : STM32F107VCT6
    Dev. Board       : EasyMx PRO v7 for STM32
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C, GPIO and LOG structures, set INT pin as input and RST pin as output.
- Application Initialization - Initialization driver enable's - I2C and GPIO,
     reset Expand 2 click, set PORTA to be output, set PORTB to be input.
- Application Task - (code snippet) This is a example which demonstrates the use of Expand 2 Click board.
     Expand 2 Click communicates with register via I2C by write to register and read from register,
     set configuration of ports and set or get ports status.
     This example shows pin activation on the PORTA by position.
     Results are being sent to the Usart Terminal where you can track their changes.
     All data logs on usb uart for aproximetly every 5 sec.

*/

#include "Click_Expand_2_types.h"
#include "Click_Expand_2_config.h"


uint8_t portStatus;
uint8_t pinPosition;
char logText[50];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_RST_PIN, _GPIO_OUTPUT );

    mikrobus_i2cInit( _MIKROBUS1, &_EXPAND2_I2C_CFG[0] );

    mikrobus_logInit( _LOG_USBUART_A , 9600 );

    Delay_ms( 100 );
}

void applicationInit()
{
    expand2_i2cDriverInit( (T_EXPAND2_P)&_MIKROBUS1_GPIO, (T_EXPAND2_P)&_MIKROBUS1_I2C, _EXPAND2_I2C_ADDRESS );
    
    // Reset
    expand2_reset();
    
    // Set Expand 2 PORTA to be output
    expand2_setDirectionPortA( _EXPAND2_I2C__MODULE_ADDRESS_5, _EXPAND2_PORT_DIRECTION_OUTPUT );
    
    // Set Expand 2 PORTB to be input
    expand2_setDirectionPortB( _EXPAND2_I2C__MODULE_ADDRESS_5, _EXPAND2_PORT_DIRECTION_INPUT );
    
    mikrobus_logWrite( "----------------", _LOG_LINE );
    mikrobus_logWrite( " Expand 2 Click ", _LOG_LINE );
    mikrobus_logWrite( "----------------", _LOG_LINE );
    
    Delay_ms( 100 );
}

void applicationTask()
{
    for ( pinPosition = 0; pinPosition < 8; pinPosition++ )
    {
        expand2_setPotrA( pinPosition );

        portStatus = expand2_readPortA( _EXPAND2_I2C__MODULE_ADDRESS_5 );
        
        IntToStr( pinPosition , logText );
        Ltrim( logText );
        mikrobus_logWrite( "      PA", _LOG_TEXT );
        mikrobus_logWrite( logText, _LOG_LINE );
        
        IntToStr( portStatus , logText );
        mikrobus_logWrite( " Status: ", _LOG_TEXT );
        mikrobus_logWrite( logText, _LOG_LINE );
        mikrobus_logWrite( "----------------", _LOG_LINE );
        
        Delay_ms( 5000 );
    }
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}