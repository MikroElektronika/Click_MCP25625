/****************************************************************************
* Title                 :   MCP25625 HAL
* Filename              :   MCP25625_hal.h
* Author                :   MSV
* Origin Date           :   01/02/2016
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials       Description
*  01/02/16       1.0.0             MSV        Module Created.
*
*****************************************************************************/
/**
 * @file MCP25625_hal.h
 * @brief <h2> HAL layer </h2>
 *
 * @par
 * HAL layer for MCP25625 click board.
 */
#ifndef MCP25625_HAL_H_
#define MCP25625_HAL_H_
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define DUMMY_BYTE                                                  0x00
/******************************************************************************
* Configuration Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/
	
/******************************************************************************
* Typedefs
*******************************************************************************/

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
 * @brief <h3> CS Pin </h3>
 *
 * @par
 * Used to set CS PIN state
 *
 * @param[in] state
 */
void MCP25625_hal_cs( int state );

/**
 * @brief <h3> RST Pin </h3>
 *
 * @par
 * Used to set RST PIN state
 *
 * @param[in] state
 */
void MCP25625_hal_rst( int state );

/**
 * @brief <h3> STB Pin </h3>
 *
 * @par
 * Used to set STB PIN state
 *
 * @param[in] state
 */
void MCP25625_hal_stb( int state );

/**
 * @brief <h3> TX0 Pin </h3>
 *
 * @par
 * Used to set TX0 PIN state
 *
 * @param[in] state
 */
void MCP25625_hal_tx0( int state );

/**
 * @brief <h3> TX1 Pin </h3>
 *
 * @par
 * Used to set TX1 PIN state
 *
 * @param[in] state
 */
void MCP25625_hal_tx1( int state );

/**
 * @brief <h3> RX0 Pin </h3>
 *
 * @par
 * Reads RX0 PIN state.
 *
 * @return Pin logic state
 */
int MCP25625_hal_rx0( void );

/**
 * @brief <h3> RX0 Pin </h3>
 *
 * @par
 * Reads RX1 PIN state.
 *
 * @return Pin logic state.
 */
int MCP25625_hal_rx1( void );

/**
 * @brief <h3> HAL Initialization </h3>
 *
 * Hal layer initialization. Must be called before any other function.
 */
void MCP25625_hal_init( void );


void MCP25625_hal_cmd( uint8_t cmd );

/**
 * @brief <h3> HAL Write </h3>
 *
 * @par
 * Writes data through SPI bus
 *
 * @note Function have no affect to the CS PIN state - chip select is
 * controled directly from HW layer.
 *
 * @param[in] buffer
 * @param[in] count
 */
void MCP25625_hal_write( uint8_t *buffer,
                         uint16_t count );

/**
 * @brief <h3> HAL Read </h3>
 *
 * @par
 * Reads data from SPI bus
 *
 * @note Function have no affect to the CS PIN state - chip select is
 * controled directly from HW layer
 *
 * @param[out] buffer
 * @param[in] count
 */
void MCP25625_hal_read( uint8_t *buffer,
                        uint16_t count );

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* MCP25625_HAL_H_ */

/*** End of File **************************************************************/
