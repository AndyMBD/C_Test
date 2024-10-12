/*
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************************************/

/*******************************************************************************
**                                                                            **
**                _  _               ___             _                        **
**               (_)| |__   _ __    / _ \ _ __ ___  | |__    /\  /\           **
**               | || '_ \ | '__|  / /_\/| '_ ` _ \ | '_ \  / /_/ /           **
**               | || | | || |    / /_\\ | | | | | || |_) |/ __  /            **
**               |_||_| |_||_|    \____/ |_| |_| |_||_.__/ \/ /_/             **
**                                                                            **
**  ihr GmbH                                                                  **
**  Airport Boulevard B210                                                    **
**  77836 Rheinmünster - Germany                                              **
**  http://www.ihr.de                                                         **
**  Phone +49(0) 7229-18475-0                                                 **
**  Fax   +49(0) 7229-18475-11                                                **
**                                                                            **
********************************************************************************
**                                                                            **
** (c) Alle Rechte bei IHR GmbH, auch fuer den Fall von Schutzrechts-         **
** anmeldungen. Jede Verfuegungsbefugnis, wie Kopier- und Weitergaberecht     **
** bei uns.                                                                   **
**                                                                            **
** (c) All rights reserved by IHR GmbH including the right to file            **
** industrial property rights. IHR GmbH retains the sole power of             **
** disposition such as reproduction or distribution.                          **
**                                                                            **
********************   Workfile:      lin_main.c      **************************
**                                                                            **
** PROJECT-DESCRIPTION:  LIN Driver Protocol Layer                            **
**                                                                            **
** FILE-DESCRIPTION:  Main routines for LIN driver                            **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                             Author(s) Identity                             **
********************************************************************************
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** BR           B.Roegl                                                       **
** HS           H. Spinner                                                    **
** BG           Blandine Guillot                                              **
*******************************************************************************/

/*******************************************************************************
**                          Revision Control History                          **
********************************************************************************
** V1.0.0: 2014-02-18, HS:   New Demo driver for Infineon TLE987x             **
** V1.0.1: 2020-04-15, BG:   Updated abstract                                 **
*******************************************************************************/

/* ===========================================================================
 *  Header files
 * ==========================================================================*/

#define LIN_MAIN_C
#include "genLinConfig.h"
#include "lin_hal.h"
#include "lin_slave_task.h"
#include "lin_driver_api.h"

#include "lin_main.h"

/* ===========================================================================
 *  Global Variables
 * ==========================================================================*/

l_u8              g_lin_frame_index                      = 0xFFu;
l_u8              g_lin_active_pid                       = 0x0u;
t_Lin_Frame_Ctrl  g_lin_frame_ctrl[LIN_NUMBER_OF_FRAMES] = LIN_FRAME_CTRL_INIT;  /* init with const from lin_config_file */
t_lin_prod_id     g_lin_prod_id;                                                 /* init with const from lin_config_file */
t_lin_status_word g_lin_status_word;
l_irqmask         g_lin_irqState                         = 0u;
l_u8              g_lin_frame_data_size                  = 0u;


/* ===========================================================================
 *  Functions
 * ==========================================================================*/

/* ---------------------------------------------------------------------------
 *  void lin_main_init(void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Initializes driver, calls HAL-init
   @param  void
   @retval void
*/
void lin_main_init(void)
{
  /* init the Hardware */
  lin_hal_init();
  /* initialise general data settings */
#if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1
  g_lin_prod_id.Supplier_lo = (l_u8)(LIN_SUPPLIER_ID & 0xFFu);
  g_lin_prod_id.Supplier_hi = (l_u8)((LIN_SUPPLIER_ID >> 8u) & 0xFFu);
  g_lin_prod_id.Function_lo = (l_u8)(LIN_FUNCTION_ID & 0xFFu);
  g_lin_prod_id.Function_hi = (l_u8)((LIN_FUNCTION_ID >> 8u) & 0xFFu);
  g_lin_prod_id.Variant = LIN_VARIANT_ID;
#endif /* end #if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1 */
  g_lin_status_word.reg = 0u;
}

/* ---------------------------------------------------------------------------
 *  void l_get_byte_array(l_u8 start[], l_u8 count, l_u8 destination[])
 * --------------------------------------------------------------------------*/
/**
   @brief  Fetch a byte array
   @pre    LIN driver initialized
   @param  l_u8 start        source buffer
   @param  l_u8 count        number of bytes to be copied
   @param  l_u8 destination  target buffer
   @retval void
*/
void l_get_byte_array(l_u8 start[], l_u8 count, l_u8 destination[])
{
  l_u8 loc_i;

  for (loc_i = 0u; loc_i < count; loc_i++)
  {
    destination[loc_i] = start[loc_i];
  }
}

/* ---------------------------------------------------------------------------
 *  void l_set_byte_array(l_u8 start[], l_u8 count, const l_u8 source[])
 * --------------------------------------------------------------------------*/
/**
   @brief  Set a byte array
   @pre    LIN driver initialized
   @param  l_u8 start        target buffer
   @param  l_u8 count        number of bytes to be copied
   @param  l_u8 source       source buffer
   @retval void
*/
void l_set_byte_array(l_u8 start[], l_u8 count, const l_u8 source[])
{
  l_u8 loc_i;

  for (loc_i = 0u; loc_i < count; loc_i++)
  {
    start[loc_i] = source[loc_i];
  }
}
