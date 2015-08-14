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
#ifndef DRV_LPS25H_H__
#define DRV_LPS25H_H__

#include "hal_twi.h"


/**@brief The sx1509b status codes.
 */
enum
{
    DRV_LPS25H_STATUS_CODE_SUCCESS,         ///< Successfull.
    DRV_LPS25H_STATUS_CODE_DISALLOWED,      ///< Disallowed.
    DRV_LPS25H_STATUS_CODE_INVALID_PARAM,   ///< Invalid parameter.
};


/**@brief The access modes.
 */
typedef enum
{
    DRV_LPS25H_ACCESS_MODE_CPU_INACTIVE,    ///< The CPU is inactive while waiting for the access to complete.
    DRV_LPS25H_ACCESS_MODE_CPU_ACTIVE,      ///< The CPU is active while waiting for the access to complete.
} drv_lps25h_access_mode_t;


/**@brief The type of the signal callback conveying signals from the driver.
 */
typedef void (*drv_lps25h_sleep_hook_t)(void);


/**@brief The sx1509b configuration.
 */
typedef struct
{
    hal_twi_id_t            twi_id;         ///< The ID of TWI master to be used for transactions.
    hal_twi_cfg_t           twi_cfg;        ///< The TWI configuration to use while the driver is opened.
    drv_lps25h_sleep_hook_t p_sleep_hook;   ///< Pointer to a function for CPU power down to be used in the CPU inactive mode.
} drv_lps25h_cfg_t;


/**@brief Initializes the lps25h interface.
 */
void drv_lps25h_init(void);


/**@brief Opens access to the lps25h driver.
 *
 * @param{in] id    The id of the HW peripheral to open the driver for.
 * @param{in] cfg   The driver configuration.
 *
 * @retval ::DRV_LPS25H_STATUS_CODE_SUCCESS     if successful.
 * @retval ::DRV_LPS25H_STATUS_CODE_DISALLOWED  if the driver could not be opened.
 */
uint32_t drv_lps25h_open(drv_lps25h_cfg_t const * const p_drv_lps25h_cfg);


/**@brief Sets the access mode.
 *
 * @nore Reading and writing data will be blocking calls if no callback is set.
 *
 * @param{in] access_mode   The mode to be used while accessing the lps25h chip.
 *
 * @return DRV_LPS25H_STATUS_CODE_SUCCESS          If the call was successful.
 * @return DRV_LPS25H_STATUS_CODE_DISALLOWED       If the call was not allowed at this time.
 * @return DRV_LPS25H_STATUS_CODE_INVALID_PARAM    If specified mode is not compatible with the configuration.
 */
uint32_t drv_lps25h_access_mode_set(drv_lps25h_access_mode_t access_mode);


/**@brief Gets the status register of the lps25h device.
 *
 * @param[in]   p_stat_value    A pointer to where the status is to be stored.
 *
 * @return DRV_LPS25H_STATUS_CODE_SUCCESS      If the call was successful.
 * @return DRV_LPS25H_STATUS_CODE_DISALLOWED   If the call was not allowed at this time. 
 */
uint32_t drv_lps25h_status_reg_get(uint8_t *p_stat_value);


/**@brief Modifies the control register of the lps25h device.
 *
 * @param[in]   set_mask    A mask specifying what bits to set.
 * @param[in]   clr_mask    A mask specifying what bits to clear.
 *
 * @return DRV_LPS25H_STATUS_CODE_SUCCESS          If the call was successful.
 * @return DRV_LPS25H_STATUS_CODE_DISALLOWED       If the call was not allowed at this time.
 * @return DRV_LPS25H_STATUS_CODE_INVALID_PARAM    If specified masks overlaps.
 */
uint32_t drv_lps25h_ctrl_reg_modify(uint32_t set_mask, uint32_t clr_mask);


/**@brief Gets the temperature in milli degrees Celcius.
 *
 * @param[in]   p_temperature_milli_deg A pointer to where value of the temperature is to be stored.
 *
 * @return DRV_LPS25H_STATUS_CODE_SUCCESS      If the call was successful.
 * @return DRV_LPS25H_STATUS_CODE_DISALLOWED   If the call was not allowed at this time.
 */
uint32_t drv_lps25h_temperature_get(int32_t * p_temperature_milli_deg);


/**@brief Gets the pressure in kilo Pascal.
 *
 * @param[in]   p_pressure_pa  A pointer to where value of the pressure is to be stored.
 *
 * @return DRV_LPS25H_STATUS_CODE_SUCCESS      If the call was successful.
 * @return DRV_LPS25H_STATUS_CODE_DISALLOWED   If the call was not allowed at this time.
 */
uint32_t drv_lps25h_pressure_get(uint32_t * p_pressure_pa);


/**@brief Opens access to the lps25h driver.
 *
 * @retval ::DRV_LPS25H_STATUS_CODE_SUCCESS     if successful.
 * @retval ::DRV_LPS25H_STATUS_CODE_DISALLOWED  if the driver could not be opened.
 */
uint32_t drv_lps25h_close(void);


#endif // DRV_LPS25H_H__
