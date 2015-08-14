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
#include "hal_twi.h"
#include "hal_serial.h"

#include "nrf.h"
#include <stdlib.h>
#include <stdbool.h>


typedef struct
{
    hal_twi_sig_callback_t  current_sig_callback;
    struct
    {
        uint8_t * tx;
        uint8_t * rx;

        uint8_t   tx_index;
        uint8_t   tx_length;
        uint8_t   rx_index;
        uint8_t   rx_length;
    } current_buffer;
    enum
    {
        STATE_IDLE,
        STATE_TX,
        STATE_RX,
        STATE_TX_ERROR,
        STATE_RX_ERROR,
    } state;
    uint8_t                 current_address;
    hal_twi_stop_mode_t     current_stop_mode;
} hal_twi_t;


#ifdef SYS_CFG_USE_TWI0
static hal_twi_t hal_twi0;
#endif
#ifdef SYS_CFG_USE_TWI1
static hal_twi_t hal_twi1;
#endif


static void hal_twi_isr_handler(NRF_TWI_Type * twi, hal_twi_t * context);


static void hal_twi_stop_check_and_handle(NRF_TWI_Type * twi)
{
    if ( (twi->SHORTS & (TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos)) != 0 )
    {
        while ( twi->EVENTS_STOPPED == 0 );
    }
}


static void hal_twi_tx_shorts_setup(NRF_TWI_Type * twi, hal_twi_t * context)
{
    twi->SHORTS = 0;
    if ( context->current_buffer.tx_index + 1 < context->current_buffer.tx_length )
    {
        twi->SHORTS = (TWI_SHORTS_BB_SUSPEND_Enabled << TWI_SHORTS_BB_SUSPEND_Pos);
    }
    else
    {
        if ( (context->current_stop_mode == HAL_TWI_STOP_MODE_STOP_ON_ANY)
        ||   (context->current_stop_mode == HAL_TWI_STOP_MODE_STOP_ON_TX_BUF_END) )
        {
            twi->EVENTS_STOPPED = 0;
            twi->SHORTS = (TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos);
        }
    }
}


static void hal_twi_rx_shorts_setup(NRF_TWI_Type * twi, hal_twi_t * context)
{
    twi->SHORTS = 0;
    if ( context->current_buffer.rx_index + 1 < context->current_buffer.rx_length )
    {
        twi->SHORTS = (TWI_SHORTS_BB_SUSPEND_Enabled << TWI_SHORTS_BB_SUSPEND_Pos);
    }
    else
    {
        if ( (context->current_stop_mode == HAL_TWI_STOP_MODE_STOP_ON_ANY)
        ||   (context->current_stop_mode == HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END) )
        {
            twi->EVENTS_STOPPED = 0;
            twi->SHORTS = (TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos);
        }
    }
}


void hal_twi_init(void)
{
#ifdef SYS_CFG_USE_TWI0
    hal_twi0.current_sig_callback = NULL;
    hal_twi0.current_stop_mode    = HAL_TWI_STOP_MODE_NONE;
    hal_twi0.current_address      = 0;

    hal_twi0.state = STATE_IDLE;
    hal_twi0.current_sig_callback = NULL;
#endif
#ifdef SYS_CFG_USE_TWI1
    hal_twi1.current_sig_callback = NULL;
    hal_twi1.current_stop_mode    = HAL_TWI_STOP_MODE_NONE;
    hal_twi1.current_address      = 0;

    hal_twi1.state = STATE_IDLE;
    hal_twi1.current_sig_callback = NULL;
#endif
}


uint32_t hal_twi_open(hal_twi_id_t id, hal_twi_cfg_t const * const cfg)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            if ( hal_serial_id_acquire(HAL_SERIAL_ID_TWI0) )
            {
                NRF_TWI0->ADDRESS = cfg->address;
                NRF_TWI0->FREQUENCY = cfg->frequency;
                return ( HAL_TWI_STATUS_CODE_SUCCESS );
            }
            break;
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            if ( hal_serial_id_acquire(HAL_SERIAL_ID_TWI1) )
            {
                NRF_TWI1->ADDRESS = cfg->address;
                NRF_TWI1->FREQUENCY = cfg->frequency;
                return ( HAL_TWI_STATUS_CODE_SUCCESS );
            }
            break;
#endif
        default:
            break;
    }
    
    return ( HAL_TWI_STATUS_CODE_DISALLOWED );
}


uint32_t hal_twi_close(hal_twi_id_t id)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            return ( (hal_serial_id_release(HAL_SERIAL_ID_TWI0)) ? HAL_TWI_STATUS_CODE_SUCCESS : HAL_TWI_STATUS_CODE_DISALLOWED );
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            return ( (hal_serial_id_release(HAL_SERIAL_ID_TWI1)) ? HAL_TWI_STATUS_CODE_SUCCESS : HAL_TWI_STATUS_CODE_DISALLOWED );
#endif
        default:
            break;
    }
    
    return ( HAL_TWI_STATUS_CODE_DISALLOWED );
}


void hal_twi_callback_set(hal_twi_id_t id, hal_twi_sig_callback_t hal_twi_sig_callback)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            hal_twi0.current_sig_callback = hal_twi_sig_callback;
            NRF_TWI0->INTENCLR = 0xFFFFFFFF;
            NRF_TWI0->EVENTS_ERROR = 0;
            break;
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            hal_twi1.current_sig_callback = hal_twi_sig_callback;
            NRF_TWI1->INTENCLR = 0xFFFFFFFF;
            NRF_TWI1->EVENTS_ERROR = 0;
            break;
#endif
        default:
            break;
    }
}


void hal_twi_address_set(hal_twi_id_t id, uint8_t dev_addr)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            hal_twi0.current_address = dev_addr & 0x7F;
            NRF_TWI0->ADDRESS = (hal_twi0.current_address << TWI_ADDRESS_ADDRESS_Pos);
            break;
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            hal_twi1.current_address = dev_addr & 0x7F;
            NRF_TWI1->ADDRESS = (hal_twi1.current_address << TWI_ADDRESS_ADDRESS_Pos);
            break;
#endif
        default:
            break;
    }
}


void hal_twi_stop_mode_set(hal_twi_id_t id, hal_twi_stop_mode_t stop_mode)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            hal_twi0.current_stop_mode = stop_mode;
            break;
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            hal_twi1.current_stop_mode = stop_mode;
            break;
#endif
        default:
            break;
    }
}


static uint32_t write_start(NRF_TWI_Type * twi, hal_twi_t * context, uint32_t length, uint8_t * tx_buffer)
{
    if ( (context->state == STATE_TX)
    ||   (context->state == STATE_RX) )
    {
        return ( HAL_TWI_STATUS_CODE_DISALLOWED );
    }
    else 
    {
        if ( context->state != STATE_TX_ERROR )
        {
            context->current_buffer.tx_index = 0;
        }
        context->state = STATE_TX;
        context->current_buffer.tx = tx_buffer;
        context->current_buffer.tx_length = length;
    }

    twi->INTENCLR =
    (
        (TWI_INTENCLR_ERROR_Clear    << TWI_INTENSET_ERROR_Pos)   |
        (TWI_INTENCLR_TXDSENT_Clear  << TWI_INTENCLR_TXDSENT_Pos) |
        (TWI_INTENCLR_RXDREADY_Clear << TWI_INTENCLR_RXDREADY_Pos)
    );
    twi->EVENTS_ERROR = 0;
    twi->EVENTS_TXDSENT = 0;
    twi->EVENTS_RXDREADY = 0;

    hal_twi_tx_shorts_setup(twi, context);
    twi->TXD = context->current_buffer.tx[context->current_buffer.tx_index];

    if ( context->current_sig_callback == NULL )
    {
        twi->TASKS_STARTTX = 1;
        hal_twi_isr_handler(twi, context);
    }
    else
    {
        twi->INTENSET = (TWI_INTENSET_TXDSENT_Enabled << TWI_INTENSET_TXDSENT_Pos) |
                        (TWI_INTENSET_ERROR_Enabled   << TWI_INTENSET_ERROR_Pos);
        twi->TASKS_STARTTX = 1;
    }
    return ( (context->state == STATE_TX_ERROR) ? HAL_TWI_STATUS_CODE_WRITE_ERROR : HAL_TWI_STATUS_CODE_SUCCESS );
}


uint32_t hal_twi_write(hal_twi_id_t id, uint32_t length, uint8_t * tx_buffer)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            return ( write_start(NRF_TWI0, &hal_twi0, length, tx_buffer) );
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            return ( write_start(NRF_TWI1, &hal_twi1, length, tx_buffer) );
#endif
        default:
            return ( HAL_TWI_STATUS_CODE_WRITE_ERROR );
    }
}


static uint32_t read_start(NRF_TWI_Type * twi, hal_twi_t * context, uint32_t length, uint8_t * rx_buffer)
{
    if ( (context->state == STATE_TX)
    ||   (context->state == STATE_RX) )
    {
        return ( HAL_TWI_STATUS_CODE_DISALLOWED );
    }
    else 
    {
        if ( context->state != STATE_RX_ERROR )
        {
            context->current_buffer.rx_index = 0;
        }
        context->state = STATE_RX;
        context->current_buffer.rx = rx_buffer;
        context->current_buffer.rx_length = length;
    }
                
    twi->INTENCLR =
    (
        (TWI_INTENCLR_ERROR_Clear    << TWI_INTENSET_ERROR_Pos)   |
        (TWI_INTENCLR_TXDSENT_Clear  << TWI_INTENCLR_TXDSENT_Pos) |
        (TWI_INTENCLR_RXDREADY_Clear << TWI_INTENCLR_RXDREADY_Pos)
    );
    twi->EVENTS_ERROR = 0;
    twi->EVENTS_TXDSENT = 0;
    twi->EVENTS_RXDREADY = 0;

    hal_twi_rx_shorts_setup(twi, context);

    if ( context->current_sig_callback == NULL )
    {
        twi->TASKS_STARTRX = 1;
        hal_twi_isr_handler(twi, context);
    }
    else
    {
        twi->INTENSET = (TWI_INTENSET_RXDREADY_Enabled << TWI_INTENSET_RXDREADY_Pos) |
                        (TWI_INTENSET_ERROR_Enabled    << TWI_INTENSET_ERROR_Pos);
        twi->TASKS_STARTRX = 1;
    }

    return ( (context->state == STATE_RX_ERROR) ? HAL_TWI_STATUS_CODE_READ_ERROR : HAL_TWI_STATUS_CODE_SUCCESS );
}
    

uint32_t hal_twi_read(hal_twi_id_t id, uint32_t length, uint8_t * rx_buffer)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_TWI_ID_TWI0:
            return ( read_start(NRF_TWI0, &hal_twi0, length, rx_buffer) );
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_TWI_ID_TWI1:
            return ( read_start(NRF_TWI1, &hal_twi1, length, rx_buffer) );
#endif
        default:
            return ( HAL_TWI_STATUS_CODE_READ_ERROR );
    }
}


static void hal_twi_isr_handler(NRF_TWI_Type * twi, hal_twi_t * context)
{
    bool done = (context->current_sig_callback != NULL);

    do
    {
        if ( context->state == STATE_TX )
        {
            if ( twi->EVENTS_TXDSENT != 0 )
            {
                twi->EVENTS_TXDSENT = 0;

                ++context->current_buffer.tx_index;
                if ( context->current_buffer.tx_index < context->current_buffer.tx_length )
                {
                    hal_twi_tx_shorts_setup(twi, context);

                    twi->TXD = context->current_buffer.tx[context->current_buffer.tx_index];
                    twi->TASKS_RESUME = 1;
                }
                else
                {
                    twi->INTENCLR = (TWI_INTENCLR_TXDSENT_Clear << TWI_INTENCLR_TXDSENT_Pos) |
                                    (TWI_INTENCLR_ERROR_Clear   << TWI_INTENSET_ERROR_Pos);
                    
                    hal_twi_stop_check_and_handle(twi);
                    context->state = STATE_IDLE;
                    if ( context->current_sig_callback != NULL )
                    {
                        context->current_sig_callback(HAL_TWI_SIGNAL_TYPE_TX_COMPLETE);
                    }
                    done = true;
                }
            }
            else if ( twi->EVENTS_ERROR != 0 )
            {
                twi->EVENTS_ERROR = 0;

                twi->INTENCLR = (TWI_INTENCLR_TXDSENT_Clear << TWI_INTENCLR_TXDSENT_Pos) |
                                (TWI_INTENCLR_ERROR_Clear   << TWI_INTENSET_ERROR_Pos);

                //hal_twi_stop_check_and_handle(twi);
                context->state = STATE_TX_ERROR;
                if ( context->current_sig_callback != NULL )
                {
                    context->current_sig_callback(HAL_TWI_SIGNAL_TYPE_TX_ERROR);
                }
                done = true;
            }
        }
        else if ( context->state == STATE_RX )
        {
            if ( twi->EVENTS_RXDREADY != 0 )
            {
                twi->EVENTS_RXDREADY = 0;

                context->current_buffer.rx[context->current_buffer.rx_index] = twi->RXD;
                ++context->current_buffer.rx_index;
                if ( context->current_buffer.rx_index < context->current_buffer.rx_length )
                {
                    hal_twi_rx_shorts_setup(twi, context);

                    twi->TASKS_RESUME = 1;
                }
                else
                {
                    twi->INTENCLR = (TWI_INTENCLR_RXDREADY_Clear << TWI_INTENCLR_RXDREADY_Pos) |
                                    (TWI_INTENCLR_ERROR_Clear    << TWI_INTENSET_ERROR_Pos);

                    hal_twi_stop_check_and_handle(twi);
                    context->state = STATE_IDLE;
                    if ( context->current_sig_callback != NULL )
                    {
                        context->current_sig_callback(HAL_TWI_SIGNAL_TYPE_RX_COMPLETE);
                    }
                    done = true;
                }
            }
            else if ( twi->EVENTS_ERROR != 0 )
            {
                twi->EVENTS_ERROR = 0;

                twi->INTENCLR = (TWI_INTENCLR_RXDREADY_Clear << TWI_INTENCLR_RXDREADY_Pos) |
                                (TWI_INTENCLR_ERROR_Clear    << TWI_INTENSET_ERROR_Pos);

                //hal_twi_stop_check_and_handle(twi);
                context->state = STATE_RX_ERROR;
                if ( context->current_sig_callback != NULL )
                {
                    context->current_sig_callback(HAL_TWI_SIGNAL_TYPE_RX_ERROR);
                }
                done = true;
            }
        }
    } while ( !done );
}


#ifdef SYS_CFG_USE_TWI0
void hal_serial_twi0_isr_handler(void)
{
    hal_twi_isr_handler(NRF_TWI0, &hal_twi0);
}
#endif


#ifdef SYS_CFG_USE_TWI1
void hal_serial_twi1_isr_handler(void)
{
    hal_twi_isr_handler(NRF_TWI1, &hal_twi1);
}
#endif
