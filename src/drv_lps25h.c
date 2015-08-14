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
#include "drv_lps25h.h"
#include "drv_lps25h_bitfields.h"
#include <stdlib.h>

/* Register addresses of the lps25h device. */
#define M_REGSTATUS     (0x27)
#define M_REGCTRL1      (0x20)
#define M_REGCTRL2      (0x21)
#define M_REGCTRL3      (0x22)
#define M_REGCTRL4      (0x23)
#define M_REGPRESSOUTXL (0x28)
#define M_REGPRESSOUTL  (0x29)
#define M_REGPRESSOUTH  (0x2A)
#define M_REGTEMPOUTL   (0x2B)
#define M_REGTEMPOUTH   (0x2C)


/* Driver properties. */
static struct
{
    drv_lps25h_cfg_t const  *   p_drv_lps25h_cfg;       ///< Pointer to the device configuration.
    drv_lps25h_access_mode_t    current_access_mode;    ///< The currently used access mode.
    volatile bool               twi_sig_callback_called;///< Indicates whether the signal callback was called.
} m_drv_lps25h;


/* Gets the value of the specified register. */
static bool reg_get(uint8_t reg_addr, uint8_t *p_value)
{
    if ( m_drv_lps25h.p_drv_lps25h_cfg != NULL )
    {
        hal_twi_id_t twi_id = m_drv_lps25h.p_drv_lps25h_cfg->twi_id;

        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END);

        m_drv_lps25h.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 1, &reg_addr)== HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_lps25h.current_access_mode == DRV_LPS25H_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_lps25h.twi_sig_callback_called) )
            {
                m_drv_lps25h.p_drv_lps25h_cfg->p_sleep_hook();
            }

            m_drv_lps25h.twi_sig_callback_called = false;
            if ( hal_twi_read(twi_id, 1, p_value) == HAL_TWI_STATUS_CODE_SUCCESS )
            {
                while ( (m_drv_lps25h.current_access_mode == DRV_LPS25H_ACCESS_MODE_CPU_INACTIVE)
                &&      (!m_drv_lps25h.twi_sig_callback_called) )
                {
                    m_drv_lps25h.p_drv_lps25h_cfg->p_sleep_hook();
                }

                return ( true );
            }
        }
    }
    
    return ( false );
}


/* Gets the value of the registers from left to right and stores them at the specified location. */
static bool two_registers_get(uint8_t reg_a, uint8_t reg_b, uint16_t *value)
{
    uint8_t  tmp_u8;
    uint16_t tmp_u16;
    
    if ( (m_drv_lps25h.p_drv_lps25h_cfg != NULL)
    &&   (reg_get(reg_b, &tmp_u8)) )
    {
        tmp_u16 = tmp_u8;
        if ( reg_get(reg_a, &tmp_u8) )
        {
            *value = (tmp_u16 << 8) | tmp_u8;
             return ( true );
        }
    }
    
    return ( false );
}


/* Gets the value of the registers from left to right and stores them at the specified location. */
static bool three_registers_get(uint8_t reg_a, uint8_t reg_b, uint8_t reg_c, uint32_t *value)
{
    uint8_t  tmp_u8;
    uint32_t tmp_u32;
    
    if ( (m_drv_lps25h.p_drv_lps25h_cfg != NULL)
    &&   (reg_get(reg_c, &tmp_u8)) )
    {
        tmp_u32 = tmp_u8;
        if ( reg_get(reg_b, &tmp_u8) )
        {
            tmp_u32 <<= 8;
            if ( reg_get(reg_a, &tmp_u8) )
            {
                *value = (tmp_u32 << 8) | tmp_u8;
                return ( true );
            }
        }
    }
    
    return ( false );
}


/* Sets the specified register to the specified value. */
static bool reg_set(uint8_t reg_addr, uint8_t value)
{
    if ( m_drv_lps25h.p_drv_lps25h_cfg != NULL )
    {
        hal_twi_id_t twi_id  = m_drv_lps25h.p_drv_lps25h_cfg->twi_id;
        uint8_t tx_buffer[2] = {reg_addr, value};
    
        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_TX_BUF_END);
    
        m_drv_lps25h.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 2, &(tx_buffer[0])) == HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_lps25h.current_access_mode == DRV_LPS25H_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_lps25h.twi_sig_callback_called) )
            {
                m_drv_lps25h.p_drv_lps25h_cfg->p_sleep_hook();
            }

            return ( true );
        }
    }
    
    return ( false );
}


/* Modifies the specified register according to the specified set and clear masks. */
static bool register_bits_modify(uint8_t reg, uint8_t set_mask, uint8_t clear_mask)
{
    uint8_t  tmp_u8;
    
    if ( ((set_mask & clear_mask)       == 0)
    &&   (m_drv_lps25h.p_drv_lps25h_cfg != NULL)
    &&   (reg_get(reg, &tmp_u8)) )
    {
        tmp_u8 |= set_mask;
        tmp_u8 &= ~(clear_mask);
        return ( reg_set(reg, tmp_u8) );
    }
    
    return ( false );
}


/* Handles the signals from the TWI driver. */
static void hal_twi_sig_callback(hal_twi_signal_type_t hal_twi_signal_type)
{
    (void)hal_twi_signal_type;
    
    m_drv_lps25h.twi_sig_callback_called = true;
}


void drv_lps25h_init(void)
{
    m_drv_lps25h.p_drv_lps25h_cfg = NULL;
}


uint32_t drv_lps25h_open(drv_lps25h_cfg_t const * const p_drv_lps25h_cfg)
{
    if ( hal_twi_open(p_drv_lps25h_cfg->twi_id, &(p_drv_lps25h_cfg->twi_cfg)) == HAL_TWI_STATUS_CODE_SUCCESS )
    {
        m_drv_lps25h.p_drv_lps25h_cfg    = p_drv_lps25h_cfg;
        m_drv_lps25h.current_access_mode = DRV_LPS25H_ACCESS_MODE_CPU_ACTIVE;
        
        return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
}


uint32_t drv_lps25h_access_mode_set(drv_lps25h_access_mode_t access_mode)
{
    if ( m_drv_lps25h.p_drv_lps25h_cfg == NULL )
    {
        return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
    }
    
    if ( access_mode == DRV_LPS25H_ACCESS_MODE_CPU_ACTIVE )
    {
        m_drv_lps25h.current_access_mode = DRV_LPS25H_ACCESS_MODE_CPU_ACTIVE;
        hal_twi_callback_set(m_drv_lps25h.p_drv_lps25h_cfg->twi_id, NULL);
    }
    else if ( (access_mode == DRV_LPS25H_ACCESS_MODE_CPU_INACTIVE)
    &&        (m_drv_lps25h.p_drv_lps25h_cfg->p_sleep_hook != NULL) )
    {
        m_drv_lps25h.current_access_mode = DRV_LPS25H_ACCESS_MODE_CPU_INACTIVE;
        hal_twi_callback_set(m_drv_lps25h.p_drv_lps25h_cfg->twi_id, hal_twi_sig_callback);
    }
    else
    {
        return ( DRV_LPS25H_STATUS_CODE_INVALID_PARAM );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
}


uint32_t drv_lps25h_status_reg_get(uint8_t *p_stat_value)
{
    uint8_t tmp_u8;
    
    if ( reg_get(M_REGSTATUS, &tmp_u8) )
    {
        *p_stat_value = tmp_u8;

        return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
}


uint32_t drv_lps25h_ctrl_reg_modify(uint32_t set_mask, uint32_t clr_mask)
{
    uint8_t set_mask_u8;
    uint8_t clr_mask_u8;
    
    if ( (set_mask & clr_mask) != 0 )
    {
        return ( DRV_LPS25H_STATUS_CODE_INVALID_PARAM );
    }
    
    set_mask_u8 = set_mask & 0xFF;
    clr_mask_u8 = clr_mask & 0xFF;
    if ( ((set_mask_u8 | clr_mask_u8) != 0)
    &&   (!register_bits_modify(M_REGCTRL1, set_mask_u8, clr_mask_u8)) )
    {
        return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
    }
        
    set_mask_u8 = (set_mask >> 8) & 0xFF;
    clr_mask_u8 = (clr_mask >> 8) & 0xFF;
    if ( ((set_mask_u8 | clr_mask_u8) != 0)
    &&   (!register_bits_modify(M_REGCTRL2, set_mask_u8, clr_mask_u8)) )
    {
        return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
    }

    set_mask_u8 = (set_mask >> 16) & 0xFF;
    clr_mask_u8 = (clr_mask >> 16) & 0xFF;
    if ( ((set_mask_u8 | clr_mask_u8) != 0)
    &&   (!register_bits_modify(M_REGCTRL3, set_mask_u8, clr_mask_u8)) )
    {
        return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
    }

    set_mask_u8 = (set_mask >> 24) & 0xFF;
    clr_mask_u8 = (clr_mask >> 24) & 0xFF;
    if ( ((set_mask_u8 | clr_mask_u8) != 0)
    &&   (!register_bits_modify(M_REGCTRL4, set_mask_u8, clr_mask_u8)) )
    {
        return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
}


uint32_t drv_lps25h_temperature_get(int32_t * p_temperature_milli_deg)
{
    int16_t tmp_16;
    
    if ( two_registers_get(M_REGTEMPOUTL, M_REGTEMPOUTH, (uint16_t *)(&tmp_16)) )
    {
        //T(milli °C) = 42.5 + (TEMP_OUT / 480) * 1000 
        *p_temperature_milli_deg = (425 + ((int32_t)tmp_16 / 48)) * 100;
        
        return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
}


uint32_t drv_lps25h_pressure_get(uint32_t * p_pressure_pa)
{
    uint32_t tmp_u32;
    
    if ( three_registers_get(M_REGPRESSOUTXL, M_REGPRESSOUTL, M_REGPRESSOUTH, &tmp_u32) )
    {
        // Pout(Pa) = (PRESS_OUT * 100) / 4096
        *p_pressure_pa = (tmp_u32 * 100) / 4096;
        
        return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
    
}


uint32_t drv_lps25h_close(void)
{
    if ( hal_twi_close(m_drv_lps25h.p_drv_lps25h_cfg->twi_id) == HAL_TWI_STATUS_CODE_SUCCESS )
    {
        m_drv_lps25h.p_drv_lps25h_cfg = NULL;
        
        return ( DRV_LPS25H_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_LPS25H_STATUS_CODE_DISALLOWED );
}
