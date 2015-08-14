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
#include "nrf.h"


uint8_t access_address[4] = {0xD6, 0xBE, 0x89, 0x8E};
uint8_t seed[3] = {0x55, 0x55, 0x55};


/**@brief The maximum possible length in device discovery mode. */
#define DD_MAX_PAYLOAD_LENGTH         (31 + 6)


/**@brief The default SHORTS configuration. */
#define DEFAULT_RADIO_SHORTS                                             \
(                                                                        \
    (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | \
    (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos)   \
)


/**@brief The default CRC init polynominal. 

   @note Written in little endian but stored in big endian, because the BLE spec. prints
         is in little endian but the HW stores it in big endian. */
#define CRC_POLYNOMIAL_INIT_SETTINGS  ((0x5B << 0) | (0x06 << 8) | (0x00 << 16))


/**@brief This macro converts the given channel index to a freuency
          offset (i.e. offset from 2400 MHz).

  @param index - the channel index to be converted into frequency offset.
  
  @return The frequency offset for the given index. */
#define CHANNEL_IDX_TO_FREQ_OFFS(index) \
    (((index) == 37)?\
        (2)\
        :\
            (((index) == 38)?\
                (26)\
            :\
                (((index) == 39)?\
                    (80)\
                :\
                    ((/*((index) >= 0) &&*/ ((index) <= 10))?\
                        ((index)*2 + 4)\
                    :\
                        ((index)*2 + 6)))))
                        

void hal_radio_channel_index_set(uint8_t channel_index)
{
    NRF_RADIO->FREQUENCY = CHANNEL_IDX_TO_FREQ_OFFS(channel_index);
    NRF_RADIO->DATAWHITEIV = channel_index;
}


void hal_radio_reset(void)
{
    NVIC_DisableIRQ(RADIO_IRQn);
    
    NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;

    NRF_RADIO->SHORTS = DEFAULT_RADIO_SHORTS;
    
    NRF_RADIO->PCNF0 =   (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)
                       | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)
                       | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk);

    NRF_RADIO->PCNF1 =   (((RADIO_PCNF1_ENDIAN_Little)        << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk)
                       | (((3UL)                              << RADIO_PCNF1_BALEN_Pos)  & RADIO_PCNF1_BALEN_Msk)
                       | (((0UL)                              << RADIO_PCNF1_STATLEN_Pos)& RADIO_PCNF1_STATLEN_Msk)
                       | ((((uint32_t)DD_MAX_PAYLOAD_LENGTH)  << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk)
                       | ((RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk);
    
    /* The CRC polynomial is fixed, and is set here. */
    /* The CRC initial value may change, and is set by */
    /* higher level modules as needed. */
    NRF_RADIO->CRCPOLY = (uint32_t)CRC_POLYNOMIAL_INIT_SETTINGS;
    NRF_RADIO->CRCCNF = (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk)
                      | (((RADIO_CRCCNF_LEN_Three)      << RADIO_CRCCNF_LEN_Pos)       & RADIO_CRCCNF_LEN_Msk);

    NRF_RADIO->RXADDRESSES  = ( (RADIO_RXADDRESSES_ADDR0_Enabled) << RADIO_RXADDRESSES_ADDR0_Pos);

    NRF_RADIO->MODE    = ((RADIO_MODE_MODE_Ble_1Mbit) << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;

    NRF_RADIO->TIFS = 150;

    NRF_RADIO->PREFIX0 = access_address[3];
    NRF_RADIO->BASE0   = ( (((uint32_t)access_address[2]) << 24) 
                         | (((uint32_t)access_address[1]) << 16)
                         | (((uint32_t)access_address[0]) << 8) );

    NRF_RADIO->CRCINIT = ((uint32_t)seed[0]) | ((uint32_t)seed[1])<<8 | ((uint32_t)seed[2])<<16;

    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
}


void hal_radio_send(uint8_t *p_data)
{
    NRF_RADIO->PACKETPTR = (uint32_t)&(p_data[0]);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_TXEN = 1;
}
