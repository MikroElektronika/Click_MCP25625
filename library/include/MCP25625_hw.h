/****************************************************************************
* Title                 :   MCP25625 CLICK
* Filename              :   MCP25625_hw.h
* Author                :   MSV
* Origin Date           :   28/06/2016
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  28/06/16    XXXXXXXXXXX         MSV      Interface Created.
*
*****************************************************************************/

/**
 * @file MCP25625_hw.h
 * @brief <h3> Hardware Layer </h3>
 *
 * @par
 * Low level functions for MCP25625 click board. All functions except
 * @link mcp25625_hw_status @endlink and @link mcp25625_hw_rx_status @endlink
 * returns 0 in case of successful execution.
 *
 * @par
 * mcp25625_hw_ctl_... are generic functions used to access all
 * control registers. ( Register lists @link mcp25625_ctl @endlink ). This type
 * of functions uses strutcs with ..._ctl sufix as a parameter,
 *
 * @par
 * Structs shoulud be implemented in higher layer. Filed reg must be
 * updated with proper value inside the higher layer. All ..._ctl structs have
 * to have fixed reg value except @link  mcp25625_int_ctl @endlink and
 * @link mcp25625_txb_ctl @endlink.
 * This allows user to have same structure for access multiple reisgers depend
 * on value assinged to reg and save a space on MCUs RAM.
 *
 * @par
 * + @link mcp25625_int_ctl @endlink is for access to the
 * @link mcp25625_int @endlink.
 * + @link mcp25625_txb_ctl @endlink is for access to the
 * @link CTL_TXB @endlink
 *
 * @par
 * Transfer @link mcp25625_data @endlink also and filter and mask buffes
 * @link mcp25625_id @endlink can also use same struct for operation.
 */

/**
 * @page LIB_INFO Library Info
 * @date 28 Jun 2016
 * @author Milos Vidojevic
 * @copyright GNU Public License
 * @version 1.0.0 - Initial testing and verification
 */

/**
 * @page TEST_CFG Test Configurations
 *
 * ### Test configuration STM : ###
 * @par
 * - <b> MCU           </b> :      STM32F107VC
 * - <b> Dev. Board    </b> :      EasyMx Pro v7
 * - <b> Oscillator    </b> :      72 Mhz internal
 * - <b> Ext. Modules  </b> :      MCP25625 Click
 * - <b> SW            </b> :      MikroC PRO for ARM 4.7.1
 *
 * ### Test configuration PIC32 : ###
 * @par
 * - <b> MCU           </b> :      PIC32MX795F512L
 * - <b> Dev. Board    </b> :      EasyPIC Fusion v7
 * - <b> Oscillator    </b> :      80 Mhz internal
 * - <b> Ext. Modules  </b> :      MCP25625 Click
 * - <b> SW            </b> :      MikroC PRO for PIC 3.6.0
 *
 * ### Test configuration FT90x : ###
 * @par
 * - <b> MCU           </b> :      FT900Q
 * - <b> Dev. Board    </b> :      EasyFT90x v7
 * - <b> Oscillator    </b> :      100 Mhz internal
 * - <b> Ext. Modules  </b> :      MCP25625 Click
 * - <b> SW            </b> :      MikroC PRO for FT90x 1.2.1
 *
 * ### Test configuration PIC : ###
 * @par
 * - <b> MCU           </b> :      PIC18F87K22
 * - <b> Dev. Board    </b> :      EasyPIC Pro v7
 * - <b> Oscillator    </b> :      16 Mhz external
 * - <b> Ext. Modules  </b> :      MCP25625 Click
 * - <b> SW            </b> :      MikroC PRO for PIC 6.6.3
 *
 * ### Test configuration dsPIC : ###
 * @par
 * - <b> MCU           </b> :      dsPIC33EJ256GP710A
 * - <b> Dev. Board    </b> :      EasyPIC Fusion v7
 * - <b> Oscillator    </b> :      8 Mhz internal
 * - <b> Ext. Modules  </b> :      MCP25625 Click
 * - <b> SW            </b> :      MikroC PRO for dsPIC 6.2.1
 *
 * ### Test configuration AVR : ###
 * @par
 * - <b> MCU           </b> :      ATMEGA32
 * - <b> Dev. Board    </b> :      EasyAVR v7
 * - <b> Oscillator    </b> :      8 Mhz external
 * - <b> Ext. Modules  </b> :      MCP25625 Click
 * - <b> SW            </b> :      MikroC PRO for FT90x 6.1.1
 */

/**
 * @mainpage
 * <h3> General Description </h3>
 * @par
 * The MCP25625 is a complete, cost-effective CAN solution that can be easily
 * added to a microcontroller with an available SPI interface.
 * The MCP25625 interfaces directly with microcontrollers operating at 2.7V to
 * 5.5V, there are no external level shifters required. In addition, the
 * MCP25625 connects directly to the physical CAN bus, supporting all
 * requirements for CAN high-speed transceivers.
 * The MCP25625 meets the automotive requirements for high-speed
 * (up to 1 Mb/s), low quiescent current, electromagnetic compatibility (EMC)
 * and electrostatic discharge (ESD).
 *
 * <h3> General Features </h3>
 * @par
 * - Stand-Alone CAN2.0B Controller with Integrated CAN Transceiver and SPI
 * - Up to 1 Mb/s Operation
 * - Very Low Standby Current ( 10 μA, typical )
 * - Up to 10 MHz SPI Clock Speed
 * - Interfaces Directly with Microcontrollers with 2.7V to 5.5V I/O
 * - Temperature Ranges: -40C ~ 125C
 *
 * <h3> CAN Controller Features </h3>
 * @par
 * - Vdd : 2.7 to 5.5V
 * - Implements CAN 2.0B ( ISO11898-1 )
 * - Three Transmit Buffers with Prioritization and Abort Feature
 * - Two Receive Buffers
 * - 6 Filters and 2 Masks, with Optional Filtering on the First Two Data Bytes
 * - Supports SPI Modes 0,0 and 1,1
 * - Specific SPI Commands to Reduce SPI Overhead
 * - Buffer Full, and Request-to-Send Pins Configurable as General Purpose I/O
 * - One Interrupt Output Pin
 *
 * <h3> CAN Transceiver Features </h3>
 * @par
 * - VDDA : 4.5V to 5.5V
 * - Implements ISO-11898-2 and ISO-11898-5 Standard Physical Layer Requirements
 * - CAN Bus Pins are Disconnected when Device is Unpowered
 *      + An Unpowered Node or Brown-Out Event will not load the CAN bus
 * - Detection of Ground Fault :
 *      + Permanent dominant detection on TXD
 *      + Permanent dominant detection on bus
 * - Power-on Reset and Voltage Brown-Out Protection on V DDA Pin
 * - Protection Against Damage Due to Short-Circuit Conditions (Positive or
 * Negative Battery Voltage)
 * - Protection Against High-Voltage Transients in Automotive Environments
 * - Automatic Thermal Shutdown Protection
 * - Suitable for 12V and 24V Systems
 * - Meets or Exceeds Stringent Automotive Design Requirements Including
 * “Hardware Requirements for LIN, CAN and FlexRay Interfaces in Automotive
 * Applications”, Version 1.3, May 2012
 * - High-Noise Immunity Due to Differential Bus Implementation
 * - High ESD Protection on CANH and CANL, meets IEC61000-4-2 up to ±8 kV
 */

#ifndef MCP25625_HW_H_
#define MCP25625_HW_H_

/******************************************************************************
* Includes
*******************************************************************************/

#include <string.h>
#include "MCP25625_hal.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/**
 * @macro MCP25625_RTS
 * @brief TRANSMIT BUFFER
 *
 * @par
 * Transmit registers - fast read.
 * @link mcp25625_cmd @endlink
 *
 * @param ( TXB0 / TXB1 / TXB2 )
 */
#define MCP25625_TX( x )            ( (x) * 2 )

/**
 * @macro MCP25625_RX
 * @brief RECEIVE BUFFER
 *
 * @par
 * Receive registers - fast read.
 * @link mcp25625_cmd @endlink
 *
 * @param ( RXB1 / RXB0 )
 */
#define MCP25625_RX( x )            ( (x) * 4 )

/**
 * @macro MCP25625_RTS
 * @brief PIN CONTROL COMMAND
 *
 * @par
 * RTS pin control
 *
 * @param ( TXB0 / TXB1 / TXB2 )
 */
#define MCP25625_RTS( x )           ( 1 << (x) )

/**
 * @macro CTL_TXB
 * @brief TRANSMIT BUFFER CONTROL
 *
 * @par
 * TX control register adresses
 *
 * @param ( TXB0 / TXB1 / TXB2 )
 */
#define CTL_TXB( x, y )             ( (x) * 10 + (y) )

/**
 * @macro CTL_RXB
 * @brief RECEIVE BUFFER CONTROL
 *
 * @par
 * RX control register addresses
 *
 * @param ( RXB1 / RXB0 )
 */
#define CTL_RXB( x )                ( (x) * 10 + 0x60 )

/**
 * @macro MCP25625_MASK
 * @brief RX MASKS
 *
 * @par
 * Mask register addresses
 *
 * @param ( RXM0 - Masks RXB0 /
 *          RXM1 - Masks RXB1 )
 */
#define MCP25625_MASK( x )          ( (x) * 4 + 0x20 )

/**
 * @macro RXF registers
 * @brief RX FILTERS
 *
 * @par
 * Filter register addresses
 *
 * @param ( RXF0 - Filters RXB0 /
 *          RXF1 - Filters RXB0 /
 *          RXF2 - Filters RXB1 /
 *          RXF3 - Filters RXB1 /
 *          RXF4 - Filters RXB1 /
 *          RXF5 - Filters RXB1 )
 */
#define MCP25625_FILTER( x )        ( (x) < 3 ? (x) * 4 : (x) * 4 + 0x10 )

/******************************************************************************
* Configuration Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/

/**
 * @enum RXB_t
 * @brief Receive Buffers
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_data_get @endlink
 * + @link mcp25625_hw_mask_set @endlink
 */
typedef enum {

    RXB0,
    RXB1

}RXB_t;

/**
 * @enum TXB_t
 * @brief Transmit Buffers
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_data_get @endlink
 * + @link mcp25625_hw_rts_ctl @endlink
 */
typedef enum {

    TXB0,
    TXB1,
    TXB2

}TXB_t;

/**
 * @enum RXF_t
 * @brief Receive Filters
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_filter_set @endlink
 */
typedef enum {

    RXF_0,
    RXF_1,
    RXF_2,
    RXF_3,
    RXF_4,
    RXF_5

}RXF_t;

/**
 * @enum mcp25625_cmd
 * @brief SPI INSTRUCTION SET
 *
 * @par
 * + CMD_RESET - Resets internal registers to default state,
 *               set Configuration mode.
 * + CMD_READ - Read data from register beginning at selected address.
 * + CMD_READ_RX - When reading a receive buffer, reduces the overhead of a
 *                 normal Read command
 * + CMD_WRITE - Write data to register beginning at selected address.
 * + CMD_LOAD_TX - When loading a transmit buffer, reduces the overhead of a
 *                 normal Write command
 * + CMD_RTS - Instructs controller to begin message transmission sequence for
 *             any of the transmit buffers.
 * + CMD_READ_STAT - Quick polling command that reads several Status bits for
 *                   transmit and receive functions.
 * + CMD_RX_STAT - Quick polling command that indicates filter match and message
 *                 type (standard, extended and/or remote) of received message.
 * + CMD_BIT_MODIFY - Allows the user to set or clear individual bits in a
 *                    particular register.
 */
typedef enum {

    CMD_RESET                   = 0xC0,
    CMD_READ                    = 0x03,
    CMD_READ_RX                 = 0x90,
    CMD_WRITE                   = 0x02,
    CMD_LOAD_TX                 = 0x40,
    CMD_RTS                     = 0x80,
    CMD_READ_STAT               = 0xA0,
    CMD_RX_STAT                 = 0xB0,
    CMD_BIT_MODIFY              = 0x05

}mcp25625_cmd;

/**
 * @enum mcp25625_ctl
 * @brief Control Registers
 *
 * @par
 * + CTL_RXP - RX PIN CONTROL AND STATUS
 * + CTL_RTS - TXRTS PIN CONTROL AND STATUS REGISTER
 * + CTL_CAN - CAN CONTROL REGISTER
 * + CTL_CNF3 - CONFIGURATION 3
 * + CTL_CNF2 - CONFIGURATION 2
 * + CTL_CNF1 - CONFIGURATION 1
 * + CTL_INTE - INTERRUPT ENABLE
 * + CTL_INTF - INTERRUPT FLAG
 * + CTL_ELG - ERROR FLAG
 */
typedef enum {

    CTL_RXP                     = 0x0C,
    CTL_RTS                     = 0x0D,
    CTL_CAN                     = 0x0F,
    CTL_CNF3                    = 0x28,
    CTL_CNF2                    = 0x29,
    CTL_CNF1                    = 0x2A,
    CTL_EFLG                    = 0x2D,
    CTL_TXB                     = 0x30,
    CTL_RXB0                    = 0x60,
    CTL_RXB1                    = 0x70

}mcp25625_ctl;

/**
 * @enum INT_t
 * @brief Interrupt Regiosters
 *
 * @par
 * + INT_CTL - INTERRUPT CONTROL
 * + INT_FLG - INTERRUPT FLAGS
 */
typedef enum {

    INT_CTL                     = 0x2B,
    INT_FLG                     = 0x2C

}mcp25625_int;

/**
 * @enum mcp25625_stat
 * @brief Status Registers
 *
 * @par
 * More info :
 * +
 *
 * @par
 * + STAT_CAN - CAN STATUS REGISTER
 * + STAT_TEC - TRANSMIT ERROR COUNTER
 * + STAT_REC - RECEIVER ERROR COUNTER
 */
typedef enum {

    STAT_CAN                    = 0x0E,
    STAT_TEC                    = 0x1C,
    STAT_REC                    = 0x1D

}mcp25625_stat;

/**
 * @enum can_opmode_t
 * @brief Operation mode
 *
 * @par
 * More info :
 * + @link mcp25625_CAN_stat @endlink
 * + @link mcp25625_CAN_ctl @endlink
 *
 * @par
 * + OPMODE_NORMAL - Normal Operation mode
 * + OPMODE_SLEEP - Sleep mode
 * + OPMODE_LOOP - Loopback mode
 * + OPMODE_LISTEN - Listen-Only mode
 * + OPMODE_CONFIG - Configuration mode
 */
typedef enum {

    OPMODE_NORMAL,
    OPMODE_SLEEP,
    OPMODE_LOOP,
    OPMODE_LISTEN,
    OPMODE_CONFIG

} can_opmode_t;

/**
 * @enum rx_mode_t
 * @brief Receive Mode
 *
 * @par
 * More information
 * + @link mcp25625_rxb0_ctl @endlink
 * + @link mcp25625_rxb1_ctl @endlink
 *
 * @par
 * + MODE_ALL - Receive all valid messages using either standard or extended
 *              identifiers that meet filter criteria. Extended ID filter
 *              registers RXF_EID are applied to first two bytes of data in
 *              the SID messages.
 * + MDOE_SID - Receive only valid messages with SID that meet filter criteria.
 * + MODE_EID - Receive only valid messages with EID that meet filter criteria.
 * + MODE_OFF - Turn mask / filters off - receive any message
 */
typedef enum {

    RX_MODE_ALL,
    RX_MODE_SID,
    RX_MODE_EID,
    RX_MODE_OFF

} rx_mode_t;

/**
 * @enum int_status_t
 * @brief Interrupt Status
 *
 * @par
 * More information
 * + @link mcp25625_can_stat @endlink
 *
 * @par
 * + INT_NO - No Interrupt
 * + INT_ERR - Error Interrupt
 * + INT_WAKE - Wake-up Interrupt
 * + INT_TXB0 - TXB0 Interrupt
 * + INT_TXB1 - TXB1 Interrupt
 * + INT_TXB2 - TXB2 Interrupt
 * + INT_RXB1 - RXB0 Interrupt
 * + INT_RXB2 - RXB1 Interrupt
 */
typedef enum {

    INT_NO,
    INT_ERR,
    INT_WAKE,
    INT_TXB0,
    INT_TXB1,
    INT_TXB2,
    INT_RXB0,
    INT_RXB1

} int_status_t;

/**
 * @enum fast_status_t
 * @brief Fast Status
 *
 * @par
 * Functions
 * + @link mcp25625_hw_status @endlink
 *
 * @par
 * More info :
 * @link mcp25625_RXB0_ctl @endlink
 * @link mcp25625_RXB1_ctl @endlink
 * @link mcp25625_TXB_ctl @endlink
 * @link mcp25625_INT_ctl @endlink
 *
 * @par
 * Real bits :
 * + RX0IF ( CANINTF register )
 * + RX1IF ( CANINTFL register )
 * + TX0REQ ( TXB0CNTRL register )
 * + TX0IF ( CANINTF register )
 * + TX1REQ ( TXB1CNTRL register )
 * + TX1IF ( CANINTF register )
 * + TX2REQ ( TXB2CNTRL register )
 * + TX2IF ( CANINTF register )
 */
typedef enum {

    FSTAT_RX0IF,
    FSTAT_RX1IF,
    FSTAT_TX0REQ,
    FSTAT_TX0IF,
    FSTAT_TX1REQ,
    FSTAT_TX1IF,
    FSTAT_TX2REQ,
    FSTAT_TX2IF

} can_fstatus;

/**
 * @enum rx_status_t
 * @brief RX Status
 *
 * @par
 * Functions:
 * - @link mcp25625_hw_rx_status @endlink
 *
 * @par
 * More info :
 * + filterhit
 *      - @link RXF_t @endlink
 * + exide
 *      - true - EID frame
 *      - false - SID frame
 * + rtr
 *      - true - remote frame
 *      - false - data frame
 *
 * + rxb_0
 *      - true - Message in RXB0
 *
 * + rxb_1
 *      - true - Message in RXB1
 */
typedef struct {

    RXF_t   filterhit       : 3;
    bool    exide           : 1;
    bool    rtr             : 1;
    uint8_t                 : 1;
    bool    rxb_0           : 1;
    bool    rxb_1           : 1;

} rx_fstatus;

/**
 * @enum mcp25625_RXP_ctl
 * @brief RX Pin Control and Status
 *
 * @par
 * Data type used alongside with @link CTL_RXP @endlink register.
 *
 * + CTL_RXP - RX PIN CONTROL AND STATUS
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + pX_mode - Pin Operation Mode
 *      - true - pin is used as interrupt when valid message loaded into RXB
 *      - false - pin is in digital output mode
 *
 * + pX_enab - Pin Function Enable
 *      - true - pin function is deteminted by pX_mode
 *      - false - pin function is disabled and it goes to logic 1
 *
 * + pX_stat - Pin State
 *      - sets the pin logic state if it is in output mode
 */
typedef enum {

    RXP_P0_MODE                 = 0x01,
    RXP_P1_MODE                 = 0x02,
    RXP_P0_ENAB                 = 0x04,
    RXP_P1_ENAB                 = 0x08,
    RXP_P0_STAT                 = 0x10,
    RXP_P1_STAT                 = 0x20

} rxp_ctl_mask;

typedef struct {

    bool        p0_mode         : 1;
    bool        p1_mode         : 1;
    bool        p0_enab         : 1;
    bool        p1_enab         : 1;
    bool        p0_stat         : 1;
    bool        p1_stat         : 1;
    uint8_t                     : 2;

    mcp25625_ctl        reg;
    rxp_ctl_mask        mask;

} mcp25625_rxp_ctl;

/**
 * @struct
 * @brief
 *
 * @par
 * Data type used alongside with @link CTL_RTS @endlink register.
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 *  + pX_mode <RW> - Pin State
 *      + true - Pin is used to request message transmission of TXBX buffer
 *      + false - Digital input
 *
 * + pX_stat <RO> - Pin mode
 *      + true - Reads state of TX RTS pin when in Digital Input mode
 *      + false - Reads as ‘0’ when pin is in ‘Request-to-Send’ mode
 */
typedef enum {

    RTS_P0_MODE                 = 0x01,
    RTS_P1_MODE                 = 0x02,
    RTS_P2_MODE                 = 0x04,
    RTS_P0_STAT                 = 0x08,
    RTS_P1_STAT                 = 0x10,
    RTS_P2_STAT                 = 0x20

} rts_ctl_mask;

typedef struct {

    bool        p0_mode         : 1;
    bool        p1_mode         : 1;
    bool        p2_mode         : 1;
    bool        p0_stat         : 1;
    bool        p1_stat         : 1;
    bool        p2_stat         : 1;
    uint8_t                     : 2;

    mcp25625_ctl        reg;
    rts_ctl_mask        mask;

} mcp25625_rts_ctl;

/**
 * @struct mcp25625_CAN_ctl
 * @brief CAN Control Settings
 *
 * @par
 * Data type used alongside with @link CAN_CTL @endlink registers.
 *
 * + CTL_CAN - CAN CONTROL REGISTER
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + clkpre <RW> - CLKOUT Pin Prescaler
 *      - Fclkout = System Clock / ( ( clkpre + 1 ) * 2 )
 *
 * + clken <RW> - CLKOUT Pin Control
 *      - true - CLKOUT pin enabled
 *      - false - CLKOUT pin disabled ( Pin is in high-impedance state )
 *
 * + osm <RW> - One-Shot Mode
 *      - true - Message will only attempt to transmit one time
 *      - false - Messages will reattempt transmission, if required
 *
 * + abat <RW> - Abort All Pending Transmissions
 *      - true - Request abort of all pending transmit buffers
 *      - false - Terminate request to abort all transmissions
 *
 * + reqop <RW> - Request Operation Mode
 *      - @link can_opmode_t @endlink
 */
typedef enum {

    CAN_CLKPRE                  = 0x03,
    CAN_CLKOUT                  = 0x04,
    CAN_OSM                     = 0x08,
    CAN_ABAT                    = 0x10,
    CAN_REQOP                   = 0xE0

} can_ctl_mask;

typedef struct {

    uint8_t     clkpre          : 2;
    bool        clken           : 1;
    bool        osm             : 1;
    bool        abat            : 1;
    can_opmode_t    reqop       : 3;

    mcp25625_ctl        reg;
    can_ctl_mask        mask;

} mcp25625_can_ctl;

/**
 * @struct mcp25625_CNF3_ctl
 * @brief Configuration 3 Settings
 *
 * @par
 * Data type used alongside with @link CTL_CNF3 @endlink register
 *
 * + CTL_CNF3 - CONFIGURATION 3
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + phseg 2 <RW> - PS2 Length bits ( Minimum valid setting for PS2 is 2 TQ )
 *      - ( phseg2 + 1 ) * TQ
 *
 * + wakfil <RW> - Wake-up Filter
 *      - true - Wake-up filter enabled
 *      - false - Wake-up filter disabled
 *
 * + sof <RW> - Start-of-Frame Signal
 *      - true - CLKOUT pin enabled for SOF signal
 *      - false - CLKOUT pin enabled for clockout function
 * ( If clken = false ( mcp25625_CAN_ctl ), bit is don’t care )
 */
typedef enum {

    CNF3_PHSEG2                 = 0x07,
    CNF3_WAKFIL                 = 0x40,
    CNF3_SOFR                   = 0x80

} cnf3_ctl_mask;

typedef struct {

    uint8_t     phseg2          : 3;
    uint8_t                     : 3;
    bool        wakfil          : 1;
    bool        sof             : 1;

    mcp25625_ctl        reg;
    cnf3_ctl_mask       mask;

} mcp25625_cnf3_ctl;

/**
 * @enum mcp25625_CNF2_ctl
 * @brief Configuration 2 Settings
 *
 * @par
 * Data type used alongside with @link CTL_CNF2 @endlink register
 *
 * + CTL_CNF2 - CONFIGURATION 2
 *
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + prseg <RW> - Propagation Segment Length
 *      - ( prseg + 1 ) * TQ
 *
 * + phseg1 <RW> - PS1 Length
 *      - ( phseg1 + 1 ) * TQ
 *
 * + sam <RW> - Sample Point Configuration
 *      - true - Bus line is sampled three times at the sample point
 *      - false - Bus line is sampled once at the sample point
 *
 * + btlmode <RW> - PS2 Bit Time Length
 *      - true - Length of PS2 determined by phseg2 ( mcp25625_CNF3_ctl )
 *      - false - Length of PS2 is the greater of PS1 and IPT ( 2 TQ )
 */
typedef enum {

    CNF2_PRSEG                   = 0x07,
    CNF2_PHSEG1                  = 0x38,
    CNF2_SAM                     = 0x40,
    CNF2_BTLMODE                 = 0x80

} cnf2_ctl_mask;

typedef struct {

    uint8_t     prseg           : 3;
    uint8_t     phseg1          : 3;
    bool        sam             : 1;
    bool        btlmode         : 1;

    mcp25625_ctl        reg;
    cnf2_ctl_mask       mask;

} mcp25625_cnf2_ctl;

/**
 * @enum mcp25625_CNF1_ctl
 * @brief Configuration 1 Settings
 *
 * @par
 * Data type used alongside with @link CTL_CNF1 @endlink register
 *
 * + CTL_CNF1 - CONFIGURATION 1
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + brp <RW> Baudrate Prescaler
 *      - TQ = 2 * ( brp + 1 ) / Fosc
 *
 * + sjw <RW> Synchronization Jump Width Length
 *      - Length = ( sjw + 1 ) * TQ
 */
typedef enum {

    CNF1_BRP                    = 0x3F,
    CNF1_SJW                    = 0xC0

} cnf1_ctl_mask;

typedef struct {

    uint8_t     brp             : 6;
    uint8_t     sjw             : 2;

    mcp25625_ctl        reg;
    cnf1_ctl_mask       mask;

} mcp25625_cnf1_ctl;

/**
 * @enum mcp25625_INT_ctl
 * @brief Interrupt Enable and Flags
 *
 * @par
 * Data type used for @link CTL_INTE @endlink or
 * @link CTL_INTE @endlink registers depend on reg field which whould be edited
 * depend on which one we want to access.
 *
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + rx0 <RW> - RXB0 Full Interrupt
 *      - Message received in RXB0
 *
 * + rx1 <RW> - RXB1 Full Interrupt
 *      - Message received in RXB1
 *
 * + tx0 <RW> - TXB0 Empty Interrupt
 *      - TXB0 becoming empty
 *
 * + tx1 <RW> - TXB1 Empty Interrupt
 *      - TXB1 becoming empty
 *
 * + tx2 <RW> - TXB2 Empty Interrupt
 *      - TXB2 becoming empty
 *
 * + err <RW> - Error Interrupt @link EFLG register @endlink
 *      - EFLG error condition change
 *
 * + wak <RW> - Wake-up Interrupt
 *      - CAN bus activity
 *
 * + merre <RW> - Message Error Interrupt
 *      - Error during message reception or transmission
 */
typedef enum {

    CAN_RX0IE                   = 0x01,
    CAN_RX1IE                   = 0x02,
    CAN_TX0IE                   = 0x04,
    CAN_TX1IE                   = 0x08,
    CAN_TX2IE                   = 0x10,
    CAN_ERRIE                   = 0x20,
    CAN_WAKIE                   = 0x40,
    CAN_MERRE                   = 0x80

} int_ctl_mask;

typedef struct {

    uint8_t     rx0             : 1;
    uint8_t     rx1             : 1;
    uint8_t     tx0             : 1;
    uint8_t     tx1             : 1;
    uint8_t     tx2             : 1;
    uint8_t     err             : 1;
    uint8_t     wak             : 1;
    uint8_t     merre           : 1;

    mcp25625_int            reg;
    int_ctl_mask            mask;

} mcp25625_int_ctl;

/**
 * @enum mcp25625_EFLG_ctl
 * @brief Error Flags
 *
 * @par
 * Data type used alongside with @link CTL_EFLG @endlink register.
 *
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + EWARN <RO> - Error Warning Flag bit.
 *      - true - TEC or REC is equal to or greater than 96.
 *
 * + RXWARN <RO> - Receive Error Warning Flag bit.
 *      - true - REC is equal to or greater than 96.
 *
 * + TXWARN <RO> - Transmit Error Warning Flag bit.
 *      - true - TEC is equal to or greater than 96.
 *
 * + RXEP <RO> - Receive Error-Passive Flag bit.
 *      - true - REC is equal to or greater than 128.
 *
 * + TXEP <RO> - Transmit Error-Passive Flag bit.
 *      - true - TEC is equal to or greater than 128.
 *
 * + TXBO <RO> - Bus-Off Error Flag bit.
 *      - true - TEC reaches 255.
 *
 * + RX0OVR <RW> - RXB0 Overflow Flag bit - ( Must be reset by MCU )
 *      - true - valid message is received for RXB0 and RX0IF
 *
 * + RX1OVR <RW> - RXB1 Overflow Flag bit - ( Must be reset by MCU )
 *      - true - valid message is received for RXB1 and RX1IF
 */
typedef enum {

    FLG_EWARN                   = 0x01,
    FLG_RXWAR                   = 0x02,
    FLG_TXWAR                   = 0x04,
    FLG_RXEP                    = 0x08,
    FLG_TXEP                    = 0x10,
    FLG_TXBO                    = 0x20,
    FLG_RX0OVR                  = 0x40,
    FLG_RX1OVR                  = 0x80

} eflg_ctl_mask;

typedef struct {

    bool        ewarn           : 1;
    bool        rxwar           : 1;
    bool        txwar           : 1;
    bool        rxerr           : 1;
    bool        txerr           : 1;
    bool        txbo            : 1;
    bool        rx0ovr          : 1;
    bool        rx1ovr          : 1;

    mcp25625_ctl        reg;
    eflg_ctl_mask       mask;

} mcp25625_eflg_ctl;

/**
 * @enum MCP25625_TXBCTRL_set
 * @brief TXB Settings
 *
 * @par
 * Data type used alongside with @link CTL_TXB @endlink registers.
 *
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + txp <RW> - Transmit Buffer Priority
 *      - ( 0 ~ 3 ) Bigger number means higher priority
 *
 * + txreq <RW> - Message Transmit Request
 *      - true - Buffer is currently pending transmission ( MCU sets this bit to
 * request message be transmitted – bit is automatically cleared when the
 * message is sent )
 *      - false - Buffer is not currently pending transmission ( MCU can clear
 * this bit to request a message abort)
 *
 * + txerr <RO> - Transmission Error Detected
 *      - true - Bus error occurred while the message was being transmitted
 *      - false - No bus error occurred while the message was being transmitted
 *
 * + mloa <RO> - Message Lost Arbitration
 *      - true - Message lost arbitration while being sent
 *      - false - Message did not lose arbitration while being sent
 *
 * + abft <RO> - Message Aborted Flag
 *      - true - Message lost arbitration while being sent
 *      - false - Message did not lose arbitration while being sent
 */

typedef enum {

    TXB_TXP                     = 0x03,
    TXB_TXREQ                   = 0x08,
    TXB_TXERR                   = 0x10,
    TXB_MLOA                    = 0x20,
    TXB_ABTF                    = 0x40

} txb_ctl_mask;

typedef struct {

    uint8_t     txp             : 2;
    uint8_t                     : 1;
    bool        txreq           : 1;
    bool        txerr           : 1;
    bool        mloa            : 1;
    uint8_t     abtf            : 1;
    uint8_t                     : 1;

    mcp25625_ctl        reg;
    txb_ctl_mask        mask;
    TXB_t               buffer;

} mcp25625_txb_ctl;

/**
 * @enum mcp25625_RXB0_ctl
 * @brief RXB0 Control Control
 *
 * @par
 * Data type used alongside with @link CTL_RXB @endlink register RXB0.
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * @par
 * + filhit0 <RO> - Which acceptance filter enabled reception of message
 *      - true - RXF1
 *      - false - RXF1
 *
 * + bukt_1 <RO> - Read-only Copy of BUKT
 *
 * + bukt <RW> - Rollover Enable setting
 *      - true - RXB0 message will rollover and written to RXB1 if RXB0 is full
 *      - false - Rollover disabled
 *
 * + rxrtr - <RO> - Received Remote Transfer Request
 *      - true - Remote Transfer Request Received
 *      - false - No Remote Transfer Request Received
 *
 * + rxm <RW> - Receive Buffer Operating mode
 *      - @link rx_mode_t @endlink
 */
typedef enum {

    RX0_FILHIT0                 = 0x01,
    RX0_BUKT1                   = 0x02,
    RX0_BUKT                    = 0x04,
    RX0_RTR                     = 0x08,
    RX0_RXM                     = 0X60

} rxb0_ctl_mask;

typedef struct {

    bool        filhit0         : 1;
    bool        bukt_1          : 1;
    bool        bukt            : 1;
    bool        rxrtr           : 1;
    uint8_t                     : 1;
    rx_mode_t   rxm             : 3;

    mcp25625_ctl        reg;
    rxb0_ctl_mask       mask;

} mcp25625_rxb0_ctl;

/**
 * @enum mcp25625_RXB1_ctl
 * @brief Receive Buffer 1 Control
 *
 * @par
 * Data type used alongside with @link CTL_RXB @endlink register RXB1.
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_ctl_set @endlink
 * + @link mcp25625_hw_ctl_get @endlink
 * + @link mcp25625_hw_ctl_on @endlink
 * + @link mcp25625_hw_ctl_off @endlink
 *
 * + filhit <RO> - Filter Hit bits
 *      - value is filter number of acceptance filter enabled reception
 *
 * + rxrtr <RO> - Received Remote Transfer Request
 *      - true - Remote Transfer Request Received
 *      - false - No Remote Transfer Request Received
 *
 * + rxm <RW> - Receive Buffer Operating mode
 *      - @link rx_mode_t @endlink
 */
typedef enum {

    RX1_FILHIT                  = 0x07,
    RX1_RTR                     = 0x08,
    RX1_RXM                     = 0x60

} rxb1_ctl_mask;

typedef struct {

    uint8_t     filhit         : 3;
    bool        rxrtr          : 1;
    uint8_t                    : 1;
    rx_mode_t   rxm            : 3;

    mcp25625_ctl        reg;
    rxb1_ctl_mask       mask;

} mcp25625_rxb1_ctl;

/**
 * @struct mcp25625_CAN_stat
 * @brief CAN Status
 *
 * @par
 * Data type used alongside with @link STAT_CAN @endlink register.
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_stat_get @endlink
 *
 * @par
 * + icod <RO> - Interrupt Flag Code
 *      - @link int_status_t @endlink
 *
 * + opmod <RO> - Operation mode
 *      - @link can_opmode_t @endlink
 */
typedef struct {

    uint8_t                     : 1;
    int_status_t    icod        : 3;
    uint8_t                     : 1;
    can_opmode_t    opmod       : 3;

} mcp25625_can_stat;

/**
 * @struct mcp25625_id
 * @brief
 *
 * @par
 * Data type used alonside with @link MCP25625_FILTER @endlink ans
 * @link MCP25625_MASK @endlink
 *
 * @note
 * When struct used for with Mask Set function exide parameter is not used
 *
 * @par
 * Functions :
 * + @link mcp25625_hw_filter_set @endlink
 * + @link mcp25625_hw_mask_set @endlink
 *
 * @par
 * + sidh - SID msb
 * + sidl - SID lsb
 * + exide - EXIDE
 * + eidt - EID msb
 * + eidh - EID mid
 * + eidl - EID lsb
 */

typedef struct {

    uint8_t     eidt            : 2;
    uint8_t                     : 1;
    uint8_t     ide             : 1;
    uint8_t     srr             : 1;
    uint8_t     sidl            : 3;

}mcp25625_sidl;

typedef struct {

    uint8_t             sidh;
    mcp25625_sidl       sidl;
    uint8_t             eidh;
    uint8_t             eidl;

} mcp25625_id;

typedef struct {

    uint8_t     len             : 4;
    uint8_t                     : 2;
    uint8_t     rtr             : 1;
    uint8_t                     : 1;

}mcp25625_dlc;

typedef struct {

    mcp25625_dlc        dlc;
    uint8_t             buff[ 8 ];

} mcp25625_data;

typedef struct {

    mcp25625_id         id;
    mcp25625_data       payload;

} mcp25625_transfer;

/******************************************************************************
* Variables
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief <h3> HW Layer Initialization </h3>
 *
 * @par
 * Prepares library for usage and sets needed values to default.
 *
 * @note
 * This function must be called first.
 */
int mcp25625_hw_init
(
        void
);

/**
 * @brief <h3> Hardware Reset </h3>
 *
 * @par
 * Resets the device using RST pin.
 */
int mcp25625_pin_reset
(
        void
);

/**
 * @brief <h3> Software Reset </h3>
 *
 * @par
 * Resets the device using @link CMD_RESET @endlink command.
 */
int mcp25625_hw_reset
(
        void
);

/**
 * @brief <h3> RTS Pin Contorol </h3>
 *
 * @par
 * Mode Info :
 * @link CMD_RTS @endlink
 *
 * @param ( TXB0 / TXB1 / TXB2 )
 */
int mcp25625_hw_rts_ctl
(
        TXB_t line
);

/**
 * @brief <h3> Read Status </h3>
 *
 * @par
 * Single instruction access to some of the often used Status bits for message
 * reception and transmission.
 *
 * More info :
 * + @link CMD_READ_STAT @endlink
 * + @link can_fstatus @endlink
 */
can_fstatus mcp25625_hw_status
(
        void
);

/**
 * @brief <h3> Read RX Status </h3>
 *
 * @par
 * Single instruction access to some of the often used Status bits for message
 * reception.
 *
 * @par
 * More Info:
 * + @link CMD_RX_STAT @endlink
 * + @link rx_fstatus @endlink
 */
void mcp25625_hw_rx_status
(
        rx_fstatus *status
);

/**
 * @brief <h3> Write Register </h3>
 *
 * @par
 * Writes data to control, configuration and interrupt registers.
 *
 * @param[in] value - any of ..._ctl registers ( except filter or mask ctl )
 */
int mcp25625_hw_ctl_set
(
        void *value
);

/**
 * @brief <h3> Update Register </h3>
 *
 * @par
 * Updates control, configuration and interrupt registers. Struct provided as
 * parameter must be converted to void pointer and proper value must be
 * assigned to the mask field.
 *
 * @param[in] value - any of ..._ctl registers ( except filter of mask ctl )
 */
int mcp25625_hw_ctl_update
(
        void *value
);

/**
 * @brief <h3> Read Register </h3>
 *
 * @par
 * Reads control, configuration, interrupt and status registers.
 *
 * @param result - - any of ..._ctl registers ( except filter of mask ctl )
 */
int mcp25625_hw_ctl_get
(
        void *result
);

/**
 * @brief <h3> Write Data Registers </h3>
 *
 * @par
 * Writes group of data registers for specified TX buffer.
 *
 * @param[in] reg ( TXB0 / TXB1 / TXB2 )
 * @param[in] tx_data
 */
int mcp25625_hw_data_set
(
        TXB_t reg,
        mcp25625_transfer *tx_data
);

/**
 * @brief <h3> Read Data Registers </h3>
 *
 * @par
 * Reads group of data registers for specified RX buffer.
 *
 * @param[in] reg ( RXB0 / RXB1 )
 * @param[out] rx_data
 */
int mcp25625_hw_data_get
(
        RXB_t reg,
        mcp25625_transfer *rx_data
);

/**
 * @brief <h3> Setup Filters </h3>
 *
 * @par
 * Writes data to filter registers.
 *
 * @param[in] reg ( RXF_0 / RXF_1 / RXF_2 / RXF_3 / RXF_4 / RXF_5 )
 * @param filter
 */
int mcp25625_hw_filter_set
(
        RXF_t reg,
        mcp25625_id *filter
);

/**
 * @brief <h3> Setup Mask </h3>
 *
 * @par
 * Writes data to mask registers.
 *
 * @param[in] reg ( RXB0 / RXB1 )
 * @param[in] mask
 */
int mcp25625_hw_mask_set
(
        RXB_t reg,
        mcp25625_id *mask
);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* MCP25625_HW_H_ */

/*** End of File **************************************************************/
