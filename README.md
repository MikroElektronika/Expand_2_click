![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Expand_2 Click

---

- **CIC Prefix**  : EXPAND2
- **Author**      : Nenad Filipovic
- **Verison**     : 1.0.0
- **Date**        : Nov 2018.

---

### Software Support

We provide a library for the Expand_2 Click on our [LibStock](https://libstock.mikroe.com/projects/view/1250/expand2-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library covers all the necessary functions to control Expand 2 Click board.
Expand 2 click communicates with the target board via I2C protocol. 
This library contains drivers for write and read data from MCP23017 chip,
set PORTA/B direction, get PORTA/B direction, 
set PORTA/B status, get PORTA/B status, etc.

Key functions :

- ``` void expand2_writePortA( uint8_t moduleAddress, uint8_t writeData ) ``` - Write one byte of data to register for PORTA function
- ``` uint8_t expand2_readPortA( uint8_t moduleAddress ) ``` - Read one byte of data from PORTA function
- ``` void expand2_setDirectionPortB( uint8_t moduleAddress, uint8_t writeData ) ``` - Set expander PORTB direction function

**Examples Description**

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


```.c

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

```



The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/1250/expand2-click) page.

Other mikroE Libraries used in the example:

- I2C


**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
