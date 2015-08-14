/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef HAL_TWI_H__
#define HAL_TWI_H__

#include <stdbool.h>
#include <stdint.h>


/**@brief The bit IDs.
 */
typedef enum
{
    HAL_TWI_ID_TWI0,
    HAL_TWI_ID_TWI1,
    HAL_TWI_ID_NONE,
} hal_twi_id_t;


/**@brief The twi configuration.
 */
typedef struct
{
    uint32_t frequency;
    uint8_t address;
} hal_twi_cfg_t;


/**@brief The callback signals of the driver.
 */
typedef enum
{
    HAL_TWI_SIGNAL_TYPE_TX_COMPLETE,    ///< Sent when the specified TX buffer has been sent.
    HAL_TWI_SIGNAL_TYPE_RX_COMPLETE,    ///< Sent when the specified number of bytes have been received.
    HAL_TWI_SIGNAL_TYPE_TX_ERROR,       ///< Sent when failed to send the specified buffer.
    HAL_TWI_SIGNAL_TYPE_RX_ERROR,       ///< Sent when failed to receive the specified number of bytes.
} hal_twi_signal_type_t;


/**@brief The modes for generating stop conditions on the bus.
 */
typedef enum
{
    HAL_TWI_STOP_MODE_NONE,                 ///< Do not genarate stop conditions.
    HAL_TWI_STOP_MODE_STOP_ON_TX_BUF_END,   ///< Genarate stop conditions whenever a buffer has been transmitted.
    HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END,   ///< Genarate stop conditions whenever a buffer has been received.
    HAL_TWI_STOP_MODE_STOP_ON_ANY,          ///< Genarate stop conditions whenever a buffer has been transmitted or received.
} hal_twi_stop_mode_t;


/**@brief The TWI status codes.
 */
enum
{
    HAL_TWI_STATUS_CODE_SUCCESS,     ///< Successfull.
    HAL_TWI_STATUS_CODE_WRITE_ERROR, ///< Write failed.
    HAL_TWI_STATUS_CODE_READ_ERROR,  ///< Read failed.
    HAL_TWI_STATUS_CODE_DISALLOWED,  ///< Disallowed.
};


/**@brief The type of the signal callback conveying signals from the driver.
 */
typedef void (*hal_twi_sig_callback_t) (hal_twi_signal_type_t hal_twi_signal_type);


/**@brief Initializes the twi interface.
 */
void hal_twi_init(void);


/**@brief Opens the driver to access the specified HW peripheral.
 *
 * @param{in] id    The id of the HW peripheral to open the driver for.
 * @param{in] cfg   The driver configuration.
 *
 * @retval ::HAL_TWI_STATUS_CODE_SUCCESS    if successful.
 * @retval ::HAL_TWI_STATUS_CODE_DISALLOWED if the driver could not be opened.
 */
uint32_t hal_twi_open(hal_twi_id_t id, hal_twi_cfg_t const * const cfg);


/**@brief Sets the callback function.
 *
 * @nore Reading and writing data will be blocking calls if no callback is set.
 *
 * @param{in] id                    The id of the HW peripheral to open the driver for.
 * @param{in] hal_twi_sig_callback  The signal callback function, or NULL if not used.
 */
void hal_twi_callback_set(hal_twi_id_t id, hal_twi_sig_callback_t hal_twi_sig_callback);


/**@brief Sets device address.
 *
 * @param{in] id        The id of the HW peripheral to set the device address for.
 * @param{in] dev_addr  The device address.
 */
void hal_twi_address_set(hal_twi_id_t id, uint8_t dev_addr);


/**@brief Sets stop mode (i.e. when to generate the stop condition on the bus).
 *
 * @param{in] id        The id of the HW peripheral to set the stop mode for.
 * @param{in] stop_mode The stop mode.
 */
void hal_twi_stop_mode_set(hal_twi_id_t id, hal_twi_stop_mode_t stop_mode);


/**@brief Writes bytes to a device.
 *
 * @note The transmit buffer shall be available to the driver until all bytes
 *       have been sent.
 *
 * @param{in] id        The id of the HW peripheral to write through.
 * @param[in] length    The number of bytes to transmit from the buffer.
 * @param{ut] tx_buffer The transmit buffer to use.
 *
 * @retval ::HAL_TWI_STATUS_CODE_SUCCESS        if successful.
 * @retval ::HAL_TWI_STATUS_CODE_WRITE_ERROR    if the bytes could not be written.
 */
uint32_t hal_twi_write(hal_twi_id_t id, uint32_t length, uint8_t * tx_buffer);


/**@brief Reads bytes from a device.
 *
 * @note The receive buffer shall be available to the driver until all bytes
 *       have been received.
 *
 * @param{in]   id        The id of the HW peripheral to read through.
 * @param{in]   length    The number of bytes to read from the buffer.
 * @param{[out] rx_buffer The receive buffer to use.
 *
 * @retval ::HAL_TWI_STATUS_CODE_SUCCESS    if successful.
 * @retval ::HAL_TWI_STATUS_CODE_READ_ERROR if the bytes could not be read.
 */
uint32_t hal_twi_read(hal_twi_id_t id, uint32_t length, uint8_t * rx_buffer);


/**@brief Closes the specified driver.
 *
 * @param{in] id    The id of the HW peripheral to close the driver for.
 *
 * @retval ::HAL_TWI_STATUS_CODE_SUCCESS    if successful.
 * @retval ::HAL_TWI_STATUS_CODE_DISALLOWED if the driver could not be closed.
 */
uint32_t hal_twi_close(hal_twi_id_t id);

#endif // HAL_TWI_H__
