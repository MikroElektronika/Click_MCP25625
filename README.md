![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

![CLICKNAME_click]()

---
[Product Page](http://www.mikroe.com/)

[Manual Page](http://docs.mikroe.com/)

[Learn Page](http://learn.mikroe.com/)

---

### General Description

The MCP25625 is a complete, cost-effective and small-footprint CAN solution that can be easily added
to a microcontroller with an available SPI interface. The MCP25625 interfaces directly with
microcontrollers operating at 2.7V to 5.5V, there are no external level shifters required. In addition, 
the MCP25625 connects directly to the physical CAN bus, supporting all requirements for CAN high-speed
transceivers. The MCP25625 meets the automotive requirements for high-speed (up to 1 Mb/s), 
low quiescent current, electromagnetic compatibility (EMC) and electrostatic discharge (ESD).

---

### Features

- Vdd : 3.3 / 5.5V
- Implements CAN 2.0B (ISO11898-1)
- Three Transmit Buffers with Prioritization and Abort Feature
- Two Receive Buffers
- Six Filters and Two Masks, with Optional Filtering on the First Two Data Bytes
- Supports SPI Modes 0,0 and 1,1
- Specific SPI Commands to Reduce SPI Overhead
- Buffer Full, and Request-to-Send Pins Configurable as General Purpose I/O
- One Interrupt Output Pin

---

### Example

#### Configuration
* MCU:             P18F87K22
* Dev.Board:       EasyPIC Pro v7
* Oscillator:      16 Mhz
* Ext. Modules:    MCP25625 click
* SW:              MikroC PRO for PIC

```
#include "MCP25625.h"
#include "resources.h"

/*
 * TFT pins
 ******************************************************************************/
char TFT_DataPort at LATD;
sbit TFT_WR at LATB1_bit;
sbit TFT_RD at LATB0_bit;
sbit TFT_CS at LATB4_bit;
sbit TFT_RS at LATB2_bit;
sbit TFT_RST at LATB5_bit;
char TFT_DataPort_Direction at TRISD;
sbit TFT_WR_Direction at TRISB1_bit;
sbit TFT_RD_Direction at TRISB0_bit;
sbit TFT_CS_Direction at TRISB4_bit;
sbit TFT_RS_Direction at TRISB2_bit;
sbit TFT_RST_Direction at TRISB5_bit;

/*
 * MCP25625 pins
 ******************************************************************************/
sbit MCP25625_CS          at LATE0_bit;
sbit MCP25625_CS_DIR      at TRISE0_bit;
sbit MCP25625_RST         at LATC0_bit;
sbit MCP25625_RST_DIR     at TRISC0_bit;
sbit MCP25625_STB         at LATA0_bit;
sbit MCP25625_STB_DIR     at TRISA0_bit;
sbit MCP25625_TX0         at LATC7_bit;
sbit MCP25625_TX0_DIR     at TRISC7_bit;
sbit MCP25625_RX0         at LATC6_bit;
sbit MCP25625_RX0_DIR     at TRISC6_bit;
sbit MCP25625_TX1         at LATC3_bit;
sbit MCP25625_TX1_DIR     at TRISC3_bit;
sbit MCP25625_RX1         at LATC4_bit;
sbit MCP25625_RX1_DIR     at TRISC4_bit;

/*
 * Global vars
 ******************************************************************************/
uint32_t EID          = 0;
char EID_text[ 50 ]   = { 0 };
uint8_t tx_test[ 10 ] = { 'M', 'S', 'G', '\0' };

/*
 * Prototypes
 ******************************************************************************/
void system_init( void );
void display_init( void );

/*
 * Functions
 ******************************************************************************/
void system_init()
{
    TRISA            = 0xFF;
    TRISG            = 0xFF;
    MCP25625_CS_DIR  = 0;
    MCP25625_RST_DIR = 0;
    MCP25625_STB_DIR = 0;
    MCP25625_TX0_DIR = 0;
    MCP25625_RX0_DIR = 1;
    MCP25625_TX1_DIR = 0;
    MCP25625_RX1_DIR = 1;
    Delay_ms( 200 );
    
    SPI1_Init_Advanced( _SPI_MASTER_OSC_DIV16, _SPI_DATA_SAMPLE_MIDDLE,
                        _SPI_CLK_IDLE_HIGH, _SPI_LOW_2_HIGH );
    Delay_ms( 200 );
}

void display_init()
{
    TFT_Init_ILI9341_8bit( 320, 240 );
    //TFT_BLED = 1;
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Brush( 1, CL_WHITE, 0, 0, 0, 0 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );
    TFT_Fill_Screen( CL_WHITE );
    TFT_Set_Pen( CL_BLACK, 1 );
    TFT_Line(  20,  46, 300,  46 );
    TFT_Line(  20,  70, 300,  70 );
    TFT_Line(  20, 220, 300, 220 );
    TFT_Line(  20, 145, 300, 145 );
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "MCP25625", 110, 14 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLUE, FO_HORIZONTAL );
    TFT_Write_Text( "CAN Controller - TX node", 80, 50 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "EasyPIC PRO v7", 19, 223 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "www.mikroe.com", 200, 223 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "Message EID :", 40, 75 );
    Delay_ms( 200 );
}

void main() 
{
    system_init();
    display_init();
    mcp25625_init( OPMODE_NORMAL );
    
    LongIntToHex( EID, EID_text );
    Ltrim( EID_text );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( EID_text, 140, 75 );
    
    while( 1 )
    {
        if( Button( &PORTA, 4, 50, 1 ) )
        {
            EID++;
            LongIntToHex( EID, EID_text );
            Ltrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 138, 74, 338, 90 );
            TFT_Rectangle( 135, 110, 338, 135 );
            TFT_Write_Text( EID_text, 140, 75 );
        }

        if( Button( &PORTG, 0, 200, 1 ) )
        {
            if( !mcp25625_msg_load( TXB0, tx_test, 4, EID, true, false ) && \
                !mcp25625_msg_send( TXB0 ) )
            {
                TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_GREEN, FO_HORIZONTAL );
                TFT_Rectangle( 135, 110, 338, 135 );
                TFT_Write_Text( "SENT", 135, 110 );
            }
            else {
            
                TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_RED, FO_HORIZONTAL );
                TFT_Rectangle( 135, 110, 338, 135 );
                TFT_Write_Text( "ERROR", 135, 110 );
            }
        }
    }
}
```
