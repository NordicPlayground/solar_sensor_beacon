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
#ifndef DRV_LSP25H_BITFIELDS_H__
#define DRV_LSP25H_BITFIELDS_H__

/* Register: CTRL_REG. */
/* Description: Control register. */

/* Field PD: Power down control. */
#define DRV_LSP25H_CTRL_REG_PD_Pos       (7)                                 
#define DRV_LSP25H_CTRL_REG_PD_Msk       (0x1 << DRV_LSP25H_CTRL_REG_PD_Pos) 
#define DRV_LSP25H_CTRL_REG_PD_PowerDown (0)                                  /*!< Power-down mode */
#define DRV_LSP25H_CTRL_REG_PD_Active    (1)                                  /*!< Active mode */

/* Field ODR: Output data rate selection. */
#define DRV_LSP25H_CTRL_REG_ODR_Pos     (4)                                  
#define DRV_LSP25H_CTRL_REG_ODR_Msk     (0x7 << DRV_LSP25H_CTRL_REG_ODR_Pos) 
#define DRV_LSP25H_CTRL_REG_ODR_OneShot (0)                                   /*!< One shot */
#define DRV_LSP25H_CTRL_REG_ODR_1HZ0    (1)                                   /*!< 'Pressure 1.0 Hz, Temperatur 1.0 Hz */
#define DRV_LSP25H_CTRL_REG_ODR_7HZ0    (2)                                   /*!< 'Pressure 7.0 Hz, Temperatur 7.0 Hz */
#define DRV_LSP25H_CTRL_REG_ODR_12HZ5   (3)                                   /*!< 'Pressure 12.5 Hz, Temperatur 12.5 Hz */
#define DRV_LSP25H_CTRL_REG_ODR_25HZ0   (4)                                   /*!< 'Pressure 25.0 Hz, Temperatur 25.0 Hz */

/* Register: STATUS_REG. */
/* Description: Status register. */

/* Field T_DA: Indicates whether temperature data is available (automatically cleared when temperature is read).. */
#define DRV_LSP25H_STATUS_REG_T_DA_Pos         (0)                                     
#define DRV_LSP25H_STATUS_REG_T_DA_Msk         (0x1 << DRV_LSP25H_STATUS_REG_T_DA_Pos) 
#define DRV_LSP25H_STATUS_REG_T_DA_Unavailable (0)                                      /*!< Temperature data is not available */
#define DRV_LSP25H_STATUS_REG_T_DA_Available   (1)                                      /*!< Temperature data is available */

/* Field P_DA: Indicates whether pressure data is available (automatically cleared when pressure is read).. */
#define DRV_LSP25H_STATUS_REG_P_DA_Pos         (1)                                     
#define DRV_LSP25H_STATUS_REG_P_DA_Msk         (0x1 << DRV_LSP25H_STATUS_REG_P_DA_Pos) 
#define DRV_LSP25H_STATUS_REG_P_DA_Unavailable (0)                                      /*!< Pressure data is not available */
#define DRV_LSP25H_STATUS_REG_P_DA_Available   (1)                                      /*!< Pressure data is available */

/* Register: PRESS_OUT. */
/* Description: Contains the pressure output value. */

/* Field POUT: Contains the pressure output value. */
#define DRV_LSP25H_PRESS_OUT_POUT_Pos (0)                                         
#define DRV_LSP25H_PRESS_OUT_POUT_Msk (0xffffff << DRV_LSP25H_PRESS_OUT_POUT_Pos) 

/* Register: TEMP_OUT. */
/* Description: Contains the temperature output value. */

/* Field TOUT: Contains the temperature output value. */
#define DRV_LSP25H_TEMP_OUT_TOUT_Pos (0)                                      
#define DRV_LSP25H_TEMP_OUT_TOUT_Msk (0xffff << DRV_LSP25H_TEMP_OUT_TOUT_Pos) 

#endif // DRV_LSP25H_BITFIELDS_H__
