/******************************************************************************
* Title                 :   MCP25625 CAN CONTROLLER
* Filename              :   MCP25625.c
* Author                :   MSV
* Origin Date           :   30/06/2016
* Notes                 :   Hardware layer
*******************************************************************************/
/**************************CHANGE LIST ****************************************
*
*    Date    Software Version    Initials   Description
*  30/01/16     .1                  MSV     Interface Created.
*
*******************************************************************************/

/**
 * @file MCP25625.c
 * @brief <h3> MCP25625 Functions </h3>
 *
 * @par
 * Higher level functions for MCP25625 click board.
 */
/******************************************************************************
* Includes
*******************************************************************************/

#include "MCP25625.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

#define MCP25625_RTS_ERR    8
#define MCP25625_RXP_ERR    9
#define MCP25625_CTL_ERR    7
#define MCP25625_COM_ERR    6

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

//---------------------------------------
//      CONTROL TYPES
//---------------------------------------
static mcp25625_can_ctl     can_ctl;
static mcp25625_rts_ctl     rts_pins;
static mcp25625_rxp_ctl     rx_pins;
static mcp25625_int_ctl     ie_ctl;
static mcp25625_txb_ctl     tx_ctl;
static mcp25625_rxb0_ctl    rx0_ctl;
static mcp25625_rxb1_ctl    rx1_ctl;
static mcp25625_cnf1_ctl    cnf_1;
static mcp25625_cnf2_ctl    cnf_2;
static mcp25625_cnf3_ctl    cnf_3;
static can_opmode_t         default_mode;

//---------------------------------------
//      TRANSFER TYPE
//---------------------------------------
static mcp25625_transfer    transfer;

//---------------------------------------
//      STATUS TYPES
//---------------------------------------
static mcp25625_can_stat    can_stat;
static rx_fstatus           rx_status;
static can_fstatus          status;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

static void unpack_sid
(       mcp25625_id *id,
        uint32_t *SID
);

static void unpack_eid
(
        mcp25625_id *id,
        uint32_t *EID
);

static void pack_sid
(
        uint32_t SID,
        mcp25625_id *id
);

static void pack_eid
(
        uint32_t EID,
        mcp25625_id *id
);

/******************************************************************************
* Private Function Definitions
*******************************************************************************/

static void unpack_sid
(
        mcp25625_id *id,
        uint32_t *SID
)
{
    *SID = 0;
    *SID |= id->sidh;
    *SID <<= 3;
    *SID |= id->sidl.sidl;
}

static void unpack_eid
(
        mcp25625_id *id,
        uint32_t *EID
)
{
    *EID = 0;
    *EID |= id->sidh;
    *EID <<= 3;
    *EID |= id->sidl.sidl;
    *EID <<= 2;
    *EID |= id->sidl.eidt;
    *EID <<= 8;
    *EID |= id->eidh;
    *EID <<= 8;
    *EID |= id->eidl;
}

static void pack_sid
(
        uint32_t SID,
        mcp25625_id *id
)
{
    id->sidh = ( SID & 0x07F8 ) >> 3;
    id->sidl.sidl = ( SID & 0x0007 );
}

static void pack_eid
(
        uint32_t EID,
        mcp25625_id *id
)
{
    id->sidh  = ( EID & 0x1FE00000 ) >> 21;
    id->sidl.sidl  = ( EID & 0x001C0000 ) >> 18;
    id->sidl.eidt  = ( EID & 0x00030000 ) >> 16;
    id->eidh  = ( EID & 0x0000FF00 ) >> 8;
    id->eidl  = ( EID & 0x000000FF );
}

/******************************************************************************
* Public Function Definitions
*******************************************************************************/

int mcp25625_init
(
        can_opmode_t mode
)
{
    MCP25625_hal_tx0( 1 );
    MCP25625_hal_tx1( 1 );

    can_ctl.reg         = CTL_CAN;
    rts_pins.reg        = CTL_RTS;
    rx_pins.reg         = CTL_RXP;
    tx_ctl.reg          = CTL_TXB;
    rx0_ctl.reg         = CTL_RXB0;
    rx1_ctl.reg         = CTL_RXB1;
    cnf_1.reg           = CTL_CNF1;
    cnf_2.reg           = CTL_CNF2;
    cnf_3.reg           = CTL_CNF3;
    default_mode        = mode;

    if( mcp25625_hw_init() )
        return MCP25625_HW_ERR;

    // Setup Control
    can_ctl.clkpre  = 0;
    can_ctl.clken   = false;
    can_ctl.abat    = true;
    can_ctl.osm     = false;
    can_ctl.reqop   = OPMODE_CONFIG;

    if( mcp25625_hw_ctl_set( ( void* )&can_ctl ) )
        return MCP25625_CTL_ERR;

    // Activate RTS pins ( TX0, TX1 )
    rts_pins.p0_mode    = true;
    rts_pins.p1_mode    = true;
    rts_pins.p2_mode    = false;

    if( mcp25625_hw_ctl_set( ( void* )&rts_pins ) )
        return MCP25625_RTS_ERR;

    // Activate RX pins ( RX0, RX1 )
    rx_pins.p0_mode = true;
    rx_pins.p0_enab = true;
    rx_pins.p1_mode = true;
    rx_pins.p1_enab = true;

    if( mcp25625_hw_ctl_set( ( void* )&rx_pins ) )
        return MCP25625_RXP_ERR;

    // Configure TXB registers( same cofig for all txb regs )
    tx_ctl.txp      = 2;
    tx_ctl.txreq    = false;
    tx_ctl.buffer   = TXB0;
    if( mcp25625_hw_ctl_set( &tx_ctl ) )
        return MCP25625_CTL_ERR;

    tx_ctl.buffer   = TXB1;
    if( mcp25625_hw_ctl_set( &tx_ctl ) )
        return MCP25625_CTL_ERR;

    tx_ctl.buffer   = TXB2;
    if( mcp25625_hw_ctl_set( &tx_ctl ) )
        return MCP25625_CTL_ERR;

    // Configure RXB registers
    rx0_ctl.bukt    = false;
    rx0_ctl.rxm     = RX_MODE_OFF;
    rx1_ctl.rxm     = RX_MODE_OFF;
    if( mcp25625_hw_ctl_set( ( void* )&rx0_ctl ) ||
        mcp25625_hw_ctl_set( ( void* )&rx1_ctl ) )
        return MCP25625_CTL_ERR;

    // Clear all interrupt flags
    ie_ctl.err     = false;
    ie_ctl.merre   = false;
    ie_ctl.rx0     = false;
    ie_ctl.rx1     = false;
    ie_ctl.tx0     = false;
    ie_ctl.tx1     = false;
    ie_ctl.tx2     = false;
    ie_ctl.wak     = false;
    ie_ctl.reg     = INT_FLG;
    if( mcp25625_hw_ctl_set( ( void* )&ie_ctl ) )
        return MCP25625_CTL_ERR;

    // Enable interrupts
    ie_ctl.err     = true;
    ie_ctl.mask    = true;
    ie_ctl.merre   = true;
    ie_ctl.rx0     = true;
    ie_ctl.rx1     = true;
    ie_ctl.tx0     = true;
    ie_ctl.tx1     = true;
    ie_ctl.tx2     = true;
    ie_ctl.wak     = false;
    ie_ctl.reg     = INT_CTL;
    if( mcp25625_hw_ctl_set( ( void* )&ie_ctl ) )
        return MCP25625_CTL_ERR;

    if( mcp25625_mode( default_mode ) )
        return MCP25625_MODE_FAULT;

    return MCP25625_OK;
}

int mcp25625_mode
(
        can_opmode_t mode
)
{   
    can_ctl.reqop = mode;
    can_ctl.mask = CAN_REQOP;

    mcp25625_hw_ctl_update( ( void* )&can_ctl );

    if( mcp25625_hw_ctl_get( ( void* )&can_stat ) )
        if( can_stat.opmod != mode )
            return MCP25625_CTL_ERR;

    return MCP25625_OK;
}

int mcp25625_com_error
(
        bool clear
)
{
    ie_ctl.reg = INT_FLG;

    mcp25625_hw_ctl_get( &ie_ctl );

    if( ie_ctl.merre )
    {
        ie_ctl.merre = 0;
        ie_ctl.mask = CAN_MERRE;

        if( clear )
            mcp25625_hw_ctl_update( &ie_ctl );

        return MCP25625_COM_ERR;
    }

    return MCP25625_OK;
}

int mcp25625_btl_config
(
        uint8_t BRP,
        uint8_t SJW,
        uint8_t PRSEG,
        uint8_t PHSEG1,
        uint8_t PHSEG2,
        bool SAM,
        bool BTLMODE,
        bool WAKFIL,
        bool SOFR
)
{
    if( mcp25625_mode( OPMODE_CONFIG ) )
        return MCP25625_MODE_FAULT;

    cnf_1.brp = BRP;
    cnf_1.sjw = SJW;
    cnf_2.btlmode = BTLMODE;
    cnf_2.sam = SAM;
    cnf_2.prseg = PRSEG;
    cnf_2.phseg1 = PHSEG1;
    cnf_3.phseg2 = PHSEG2;
    cnf_3.sof = SOFR;
    cnf_3.wakfil = WAKFIL;

    if( mcp25625_hw_ctl_set( ( void* )&cnf_1 ) || \
        mcp25625_hw_ctl_set( ( void* )&cnf_2 ) || \
        mcp25625_hw_ctl_set( ( void* )&cnf_3 ) )
        MCP25625_CTL_ERR;

    if( mcp25625_mode( default_mode ) )
        return MCP25625_MODE_FAULT;

    return MCP25625_OK;
}

int mcp25625_tx_config
(
        TXB_t tx_buff,
        uint8_t TXP,
        bool use_RTS_pin
)
{
    tx_ctl.buffer = tx_buff;
    tx_ctl.txp = TXP;

    if( mcp25625_mode( OPMODE_CONFIG ) )
        return MCP25625_MODE_FAULT;

    if( mcp25625_hw_ctl_set( ( void* )&tx_ctl ) )
        return MCP25625_CTL_ERR;

    switch ( tx_buff )
    {
        case TXB0:

            rts_pins.p0_mode = use_RTS_pin;
            rts_pins.mask = RTS_P0_MODE;

            if( mcp25625_hw_ctl_update( ( void* )&rts_pins ) )
                return MCP25625_CTL_ERR;
        break;
        case TXB1:

            rts_pins.p1_mode = use_RTS_pin;
            rts_pins.mask = RTS_P1_MODE;

            if( mcp25625_hw_ctl_update( ( void* )&rts_pins ) )
                return MCP25625_CTL_ERR;
        break;
        case TXB2:

            rts_pins.p2_mode = use_RTS_pin;
            rts_pins.mask = RTS_P2_MODE;

            if( mcp25625_hw_ctl_update( ( void* )&rts_pins ) )
                return MCP25625_CTL_ERR;
        break;
    }

    if( mcp25625_mode( default_mode ) )
        return MCP25625_MODE_FAULT;

    return MCP25625_OK;
}

int mcp25625_rx_config
(
        RXB_t rx_buff,
        rx_mode_t mode,
        bool BUKT,
        bool RX_PIN
)
{
    switch( rx_buff )
    {
        case RXB0 :

            rx0_ctl.bukt = BUKT;
            rx0_ctl.rxm = mode;

            if( mcp25625_hw_ctl_set( ( void* )&rx0_ctl ) )
                return MCP25625_CTL_ERR;

            rx_pins.p0_enab = RX_PIN;
            rx_pins.p0_mode = true;

            if( mcp25625_hw_ctl_set( ( void* )&rx_pins ) )
                return MCP25625_CTL_ERR;


        break;
        case RXB1 :

            rx1_ctl.rxm = mode;

            if( mcp25625_hw_ctl_set( ( void* )&rx1_ctl ) )
                return MCP25625_CTL_ERR;

            rx_pins.p1_enab = RX_PIN;
            rx_pins.p1_mode = true;

            if( mcp25625_hw_ctl_set( ( void* )&rx_pins ) )
                return MCP25625_CTL_ERR;

        break;
    }

    return MCP25625_OK;
}

int mcp25625_filter_config
(
        RXF_t rx_filter,
        uint16_t SID,
        uint32_t EID,
        bool EXIDE
)
{
    if( mcp25625_mode( OPMODE_CONFIG ) )
        return MCP25625_MODE_FAULT;

    if( ( transfer.id.sidl.ide = EXIDE ) )
        pack_eid( EID, &transfer.id );
    else
        pack_sid( SID, &transfer.id );

    if( mcp25625_hw_filter_set( rx_filter, &transfer.id ) )
        return MCP25625_CTL_ERR;

    if( mcp25625_mode( default_mode ) )
        return MCP25625_MODE_FAULT;

    return MCP25625_OK;
}

int mcp25625_mask_config
(
        RXB_t rx_mask,
        uint16_t SID,
        uint32_t EID
)
{
   pack_eid( EID, &transfer.id );
   pack_sid( SID, &transfer.id );

    if( mcp25625_hw_mask_set( rx_mask, &transfer.id ) )
        return MCP25625_CTL_ERR;

    return MCP25625_OK;
}

int mcp25625_msg_load
(
        TXB_t tx_buff,
        uint8_t *msg,
        uint8_t count,
        uint32_t ID,
        bool IDE,
        bool RTR
)
{
    tx_ctl.buffer = tx_buff;
    tx_ctl.txreq = false;
    tx_ctl.mask = TXB_TXREQ;

    if( mcp25625_mode( OPMODE_CONFIG ) )
        return MCP25625_MODE_FAULT;

    if( mcp25625_hw_ctl_update( &tx_ctl ) )
        return MCP25625_CTL_ERR;

    if( mcp25625_mode( default_mode ) )
        return MCP25625_MODE_FAULT;

    if( ( transfer.id.sidl.ide = IDE ) )
        pack_eid( ID, &transfer.id );
    else
        pack_sid( ID, &transfer.id );

    transfer.payload.dlc.rtr = RTR;
    transfer.payload.dlc.len = count;
    memcpy( ( void* )&transfer.payload.buff, ( void* )msg, count );

    if( mcp25625_hw_data_set( tx_buff, &transfer ) )
        return MCP25625_TX_ERR;

    return MCP25625_OK;
}

int mcp25625_msg_send
(
        TXB_t tx_buff
)
{
    tx_ctl.buffer = tx_buff;
    tx_ctl.txreq = true;
    tx_ctl.mask = TXB_TXREQ;

    if( mcp25625_hw_ctl_update( ( void* )&tx_ctl ) )
        return MCP25625_CTL_ERR;

    while( tx_ctl.txreq )
        if( mcp25625_hw_ctl_get( ( void* )&tx_ctl ) )
            return MCP25625_HW_ERR;

    return MCP25625_OK;
}

bool mcp25625_msg_ready
(
        RXB_t rx_buffer
)
{
   rx_fstatus rx_status;

   mcp25625_hw_rx_status( &rx_status );

    switch ( rx_buffer )
    {
        case RXB0:

            return rx_status.rxb_0;

        break;
        case RXB1 :

            return rx_status.rxb_1;

        break;
        default:

            return false;

        break;
    }
}
int mcp25625_msg_read
(
        RXB_t rx_buffer,
        uint8_t *msg,
        uint8_t *count,
        uint32_t *ID,
        bool *IDE,
        bool *RTR
)
{
    if( mcp25625_hw_data_get( rx_buffer, &transfer ) )
        return MCP25625_RX_ERR;

    if( ( *IDE = transfer.id.sidl.ide ) )
    {
        *RTR = transfer.payload.dlc.rtr;
        unpack_eid( &transfer.id, ID );

    } else {

        *RTR = transfer.id.sidl.srr;
        unpack_sid( &transfer.id, ID );
    }

    *count = transfer.payload.dlc.len;
    memcpy( ( void* )msg, ( void* )transfer.payload.buff, 8 );

    ie_ctl.rx0 = false;
    ie_ctl.rx1 = false;
    ie_ctl.mask = CAN_RX0IE | CAN_RX1IE;
    ie_ctl.reg = INT_FLG;
    if( mcp25625_hw_ctl_update( ( void* )&ie_ctl ) )
        return MCP25625_CTL_ERR;

    return MCP25625_OK;
}

/*************** END OF FUNCTIONS *********************************************/