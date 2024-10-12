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
********************   Workfile:      lin_driver_api.c      ********************
**                                                                            **
** PROJECT-DESCRIPTION:  LIN Driver Protocol Layer                            **
**                                                                            **
** FILE-DESCRIPTION:     All routines for access from application to lin      **
**                       driver and callback function from lin driver         **
**                       to application                                       **
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

#include "genLinConfig.h"
#include "lin_slave_task.h"

#include "lin_driver_api.h"

/* ===========================================================================
 *  Global Variables
 * ==========================================================================*/

/* ===========================================================================
 *  Functions
 * ==========================================================================*/

/* ---------------------------------------------------------------------------
 *  l_bool l_sys_init (void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Initialise LIN driver system.
   @pre    MCU started
   @param  void
   @retval void
*/
l_bool l_sys_init (void)
{
  lin_main_init();
  return 0u;
}


/* ---------------------------------------------------------------------------
 *  l_bool l_ifc_init(void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Initialize LIN interface.
   @pre    LIN driver initialized
   @param  void
   @retval State of initialization
*/
l_bool l_ifc_init(void)
{
  return 0u;
}

/* ---------------------------------------------------------------------------
 *  l_u16 l_ifc_read_status(void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Get status information from LIN driver.
   @pre    LIN driver initialized
   @param  void
   @retval LIN_status word        MSB contains last PID information
                                  LSB contains status flags
   @see    LIN 2.0 and LIN 2.1 specification
*/
l_u16 l_ifc_read_status(void)
{
  l_u16 loc_temp;
  loc_temp = g_lin_status_word.reg;
  g_lin_status_word.reg = 0u;
  return loc_temp;
}

#ifdef LIN_MASTER
/* ---------------------------------------------------------------------------
 *  l_ifc_goto_sleep(void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Create a goto sleep master request.
   @pre    LIN driver initialized
   @param  void
   @retval void
*/
void l_ifc_goto_sleep(void)
{
  /* create goto sleep message and transmit at next tick */
}
#endif
