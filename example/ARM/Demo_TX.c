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

uint8_t tx_test[ 10 ] = { 'M', 'S', 'G', '\0' };
uint8_t rx_test[ 10 ] = { 0 };
uint8_t count;
bool RX_IDE, RX_RTR;
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
    
    // DBG
    UART1_Init( 115200 );
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
    TFT_Line(  20, 145, 300, 145 );
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "MCP25625", 110, 14 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLUE, FO_HORIZONTAL );
    TFT_Write_Text( "CAN Controller LOOP mode", 80, 50 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "EasyMx PRO v7 for STM32", 19, 223 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "www.mikroe.com", 200, 223 );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "Message EID :", 40, 75 );
    TFT_Write_Text( "Received EID :", 40, 150 );
}

void main() 
{
    system_init();
    display_init();
    
    if( mcp25625_init( OPMODE_LOOP ) ) //|| \
        //mcp25625_btl_config( 0, 3, 7, 4, 4, true, true, true, false ) )
        UART1_Write_Text( " - Initialization Error\r\n" );
    else
        UART1_Write_Text( " - MCP25625 Initialization Done\r\n" );
    
    LongLongUnsignedToHex( EID, EID_text );
    LTrim( EID_text );
    TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( EID_text, 140, 75 );
    
    /*if( mcp25625_filter_config( RXF_0, 0x07FF, 0, false ) || \
        mcp25625_filter_config( RXF_1, 0x0001, 0, false ) || \
        mcp25625_filter_config( RXF_2, 0, 0x1FFFFFFF, true ) || \
        mcp25625_filter_config( RXF_3, 0, 0x00000000, true ) || \
        mcp25625_filter_config( RXF_4, 0, 0x00000001, true ) || \
        mcp25625_filter_config( RXF_5, 0, 0x0000000F, true ) )
        UART1_Write_Text( " - Filter Config Error\r\n" );
    else
        UART1_Write_Text( " - Filter Config Done\r\n" );


    if( mcp25625_mask_config( RXB0, 0x07FF, 0x1FFFFFFF ) || \
        mcp25625_mask_config( RXB1, 0x07FF, 0x1FFFFFFF ) )
        UART1_Write_Text( " - Mask Config Error\r\n" );
    else
        UART1_Write_Text( " - Mask Config Done\r\n" );


    if( mcp25625_rx_config( RXB0, RX_MODE_SID, false, false ) || \
        mcp25625_rx_config( RXB1, RX_MODE_EID, false, false ) )
        UART1_Write_Text( " - RX Configuration Error\r\n" );
    else
        UART1_Write_Text( " - RX Config Done\r\n" );*/
    
    while( 1 )
    {
        if( Button( &GPIOD_IDR, 0, 10, 1 ) )
        {
            EID--;
            LongLongUnsignedToHex( EID, EID_text );
            Ltrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 138, 74, 338, 90 );
            TFT_Rectangle( 135, 110, 338, 135 );
            TFT_Write_Text( EID_text, 140, 75 );
        }
            
        if( Button( &GPIOD_IDR, 3, 10, 1 ) )
        {
            EID++;
            LongLongUnsignedToHex( EID, EID_text );
            Ltrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 138, 74, 338, 90 );
            TFT_Rectangle( 135, 110, 338, 135 );
            TFT_Write_Text( EID_text, 140, 75 );
        }

        if( Button( &GPIOD_IDR, 4, 200, 1 ) )
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
        
        if( mcp25625_msg_ready( RXB0 ) )
        {
            if( mcp25625_msg_read( RXB0, rx_test, &count, &EID, &RX_IDE, &RX_RTR ) )
                UART1_Write_Text( " - - RX Error\r\n" );

            TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_BLUE, FO_HORIZONTAL );
            TFT_Rectangle( 135, 180, 338, 205 );
            TFT_Write_Text( "RX 0", 135, 180 );
            LongLongUnsignedToHex( EID, EID_text );
            LTrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 135, 150, 338, 170 );
            TFT_Write_Text( EID_text, 140, 150 );
            
            UART1_Write_Text( EID_text );
            UART1_Write_Text( "\r\nData : ");
            UART1_Write_Text( rx_test );
            UART1_Write_Text( "\r\n" );
        }

        if( mcp25625_msg_ready( RXB1 ) )
        {
            if( mcp25625_msg_read( RXB1, rx_test, &count, &EID, &RX_IDE, &RX_RTR ) )
                UART1_Write_Text( " - - RX Error\r\n" );
            
            TFT_Set_Font( &HandelGothic_BT21x22_Regular, CL_BLUE, FO_HORIZONTAL );
            TFT_Rectangle( 138, 178, 338, 198 );
            TFT_Write_Text( "RX 1", 135, 180 );
            LongLongUnsignedToHex( EID, EID_text );
            LTrim( EID_text );
            TFT_Set_Font( &Tahoma15x16_Bold, CL_RED, FO_HORIZONTAL );
            TFT_Rectangle( 138, 150, 338, 170 );
            TFT_Write_Text( EID_text, 140, 150 );
            
            UART1_Write_Text( EID_text );
            UART1_Write_Text( "\r\nData : ");
            UART1_Write_Text( rx_test );
            UART1_Write_Text( "\r\n" );
        }
    }
}