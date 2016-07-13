#include "MCP25625.h"
#include "resources.h"

unsigned int TFT_DataPort at GPIOE_ODR;
sbit TFT_RST              at GPIOE_ODR.B8;
sbit TFT_RS               at GPIOE_ODR.B12;
sbit TFT_CS               at GPIOE_ODR.B15;
sbit TFT_RD               at GPIOE_ODR.B10;
sbit TFT_WR               at GPIOE_ODR.B11;
sbit TFT_BLED             at GPIOE_ODR.B9;
sbit MCP25625_CS          at GPIOD_ODR.B13;
sbit MCP25625_RST         at GPIOC_ODR.B2;
sbit MCP25625_STB         at GPIOA_ODR.B4;
sbit MCP25625_TX0         at GPIOD_ODR.B9;
sbit MCP25625_RX0         at GPIOD_IDR.B8;
sbit MCP25625_TX1         at GPIOB_ODR.B6;
sbit MCP25625_RX1         at GPIOB_IDR.B7;

uint8_t count = 0;
bool RX_IDE, RX_RTR;
uint8_t rx_test[ 8 ] = { 0 };
uint8_t rx_data[ 10 ] = { 0 };


uint32_t EID         = 0;
char EID_text[ 50 ]  = { 0 };

void system_init()

{
    GPIO_Digital_Input( &GPIOD_IDR, _GPIO_PINMASK_0 );
    GPIO_Digital_Input( &GPIOD_IDR, _GPIO_PINMASK_3 );
    GPIO_Digital_Input( &GPIOD_IDR, _GPIO_PINMASK_4 );
    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    GPIO_Digital_Output( &GPIOC_BASE, _GPIO_PINMASK_2 );
    GPIO_Digital_Output( &GPIOA_BASE, _GPIO_PINMASK_4 );
    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_9 );
    GPIO_Digital_Input( &GPIOD_BASE, _GPIO_PINMASK_8 );
    GPIO_Digital_Output( &GPIOB_BASE, _GPIO_PINMASK_6 );
    GPIO_Digital_Input( &GPIOB_BASE, _GPIO_PINMASK_7 );
    Delay_ms( 200 );

    SPI3_Init_Advanced( _SPI_FPCLK_DIV64, _SPI_MASTER  | _SPI_8_BIT |
                        _SPI_CLK_IDLE_LOW | _SPI_FIRST_CLK_EDGE_TRANSITION |
                        _SPI_MSB_FIRST | _SPI_SS_DISABLE | _SPI_SSM_ENABLE |
                        _SPI_SSI_1, &_GPIO_MODULE_SPI3_PC10_11_12 );
    Delay_ms( 200 );
}

void display_init()
{
    TFT_Init_ILI9341_8bit( 320, 240 );
    TFT_BLED = 1;
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Brush( 1, CL_WHITE, 0, 0, 0, 0 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );
    TFT_Fill_Screen( CL_WHITE );
    TFT_Set_Pen( CL_BLACK, 1 );
    TFT_Line(  20,  46, 300,  46 );
    TFT_Line(  20,  70, 300,  70 );
    TFT_Line(  20, 220, 300, 220 );
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "MCP25625", 113, 14 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLUE, FO_HORIZONTAL );
    TFT_Write_Text( "CAN Controller - RX node ", 95, 50 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "EasyMx PRO v7 for STM32", 19, 223 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "www.mikroe.com", 200, 223 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "Message EID :", 40, 150 );
    TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_BLUE, FO_HORIZONTAL );
}

void main()
{
    system_init();
    display_init();
    mcp25625_init( OPMODE_NORMAL );

    while( 1 )
    {
        if( mcp25625_msg_ready( RXB0 ) )
        {
            mcp25625_msg_read( RXB0, rx_test, &count, &EID, &RX_IDE, &RX_RTR );
            TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_BLUE, FO_HORIZONTAL );
            TFT_Rectangle( 138, 98, 338, 128 );
            TFT_Write_Text( "RX 0", 140, 100 );
            LongLongUnsignedToHex( EID, EID_text );
            LTrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 138, 148, 338, 168 );
            TFT_Write_Text( EID_text, 140, 150 );
        }

        if( mcp25625_msg_ready( RXB1 ) )
        {
            mcp25625_msg_read( RXB1, rx_test, &count, &EID, &RX_IDE, &RX_RTR );
            TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_BLUE, FO_HORIZONTAL );
            TFT_Rectangle( 138, 98, 338, 128 );
            TFT_Write_Text( "RX 1", 140, 100 );
            LongLongUnsignedToHex( EID, EID_text );
            LTrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 138, 148, 338, 168 );
            TFT_Write_Text( EID_text, 140, 150 );
        }
        Delay_ms( 200 );
    }
}