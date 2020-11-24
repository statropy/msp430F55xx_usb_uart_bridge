/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, Erik Larson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * hal.h
 *
 *  Created on: May 7, 2020
 *      Author: Erik Larson
 */

#ifndef HAL_H_
#define HAL_H_

#if defined (BRIDGE_UART0)
#define UART_BRIDGE USCI_A0_BASE
#define UCA_BRIDGE_IV UCA0IV
#else
#define UART_BRIDGE USCI_A1_BASE
#define UCA_BRIDGE_IV UCA1IV
#endif

//*****************************************************************************
//
//! Initializes pins and clocks.
//!
//! \param mclkFreq is the desired main clock frequency.
//
//*****************************************************************************
void hal_init(uint32_t mclkFreq);

//*****************************************************************************
//
//! Control the external BOOT line.
//!
//! \param active is bool, TRUE makes BOOT active.
//!
//!  Mimic open-drain operation, active low
//!  Drive low for boot mode, Input for normal
//
//*****************************************************************************
void hal_ext_boot(uint8_t active);

//*****************************************************************************
//
//! Read the external BOOT line.
//!
//! \return TRUE when high, FALSE when low
//
//*****************************************************************************
uint8_t hal_ext_boot_read(void);

//*****************************************************************************
//
//! Control the external RESET line.
//!
//! \param active is bool, TRUE in reset.
//!
//!  Mimic open-drain operation, active low
//!  Drive low for reset, Input for normal
//
//*****************************************************************************
void hal_ext_reset(uint8_t active);

//*****************************************************************************
//
//! Control the UART connection.
//!
//! \param active is bool, TRUE to connect UART to external chip.
//!
//!  Active high
//!  Drive high for UART, low to disconnect
//
//*****************************************************************************
void hal_ext_uart(uint8_t active);

#endif /* HAL_H_ */
