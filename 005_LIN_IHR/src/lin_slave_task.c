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
********************   Workfile:      lin_slave_task.c        ******************
**                                                                            **
** PROJECT-DESCRIPTION:  LIN Driver Protocol Layer                            **
**                                                                            **
** FILE-DESCRIPTION:  All Routines for LIN data receive/transmit              **
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
#include "lin_main.h"
#include "lin_driver_api.h"
#include "lin_hal.h"
#include "lin_slave_task.h"

/* ===========================================================================
 *  Global Variables
 * ==========================================================================*/

/* ===========================================================================
 *  Functions
 * ==========================================================================*/

/* ---------------------------------------------------------------------------
 *  void lin_slave_task_tx_data(void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Cleans up all states after a successful frame transmit
   @pre    LIN driver initialized
   @param  void
   @retval void
*/
void lin_slave_task_tx_data(void)
{
  /* tx successful finished */
  if (g_lin_status_word.flag.last_pid != 0x0u)
  {
    g_lin_status_word.flag.overrun = L_SET;
  }

  g_lin_status_word.flag.last_pid = g_lin_active_pid;
  g_lin_status_word.flag.successful_transfer = L_SET;
#if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1

  if (g_lin_active_pid == LIN_Response_Error_Frame_PID) /* pid of status frame */
  {
    /* Reset Response Error Flag */
    l_Reset_Response_Error_Flag();
  }

#endif /* end #if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1 */

  /* Update Frame Flag */
  if (g_lin_frame_index < LIN_NUMBER_OF_FRAMES)
  {
    g_lin_frame_ctrl[g_lin_frame_index].frame.frame_type.update_flag = 0u;
  }
}

/* ---------------------------------------------------------------------------
 *  void lin_slave_task_rx_pid(l_u8 rx_pid_uc)
 * --------------------------------------------------------------------------*/
/**
   @brief  PID recognition
   @pre    LIN driver initialized
   @param  void
   @retval void
*/
void lin_slave_task_rx_pid(l_u8 rx_pid_uc)
{
  l_u8 lin_frame_found = 0u;
  l_u8 loc_i;
  g_lin_active_pid = rx_pid_uc;

  switch (rx_pid_uc)
  {
    /* Master Request Frame */
    case (0x3C):
      break;

    /* Slave Response Frame */
    case (0x7D):
      break;

    /* User Defined Frame */
    case (0xFE):
      break;

    /* Reserved PID */
    case (0xBF):
      break;

    /* Application Frame Handling */
    default:

      /* find frame according to pid */
      for (loc_i = 0u; loc_i < LIN_NUMBER_OF_FRAMES; loc_i++)
      {
        if (g_lin_frame_ctrl[loc_i].frame.pid == rx_pid_uc)
        {
          g_lin_frame_index = loc_i;

          /* unconditional frames */
          /* check if publish or subscribe frame */
          if (g_lin_frame_ctrl[loc_i].reg[L_CTRL_TYPE_OFFSET] & PUBLISH)
          {
            /* config lin for unconditional tx data */
            lin_hal_set_tx_pid(g_lin_active_pid);
            lin_hal_tx_response(LIN_PROTOCOL_TYPE, l_LinData.Frame[loc_i].frame_data, g_lin_frame_ctrl[loc_i].reg[L_CTRL_LENGTH_OFFSET]);
          }
          else
          {
            /* config lin for rx data */
            lin_hal_set_rx_pid(g_lin_active_pid);
            lin_hal_rx_response(LIN_PROTOCOL_TYPE, g_lin_frame_ctrl[loc_i].reg[L_CTRL_LENGTH_OFFSET]);
          }

          lin_frame_found = 1u;
          break;
        }
        else
        {
          g_lin_frame_index = 0xFFu;
        }
      }

      if (lin_frame_found == 0u)
      {
        lin_slave_state_g = idle;
      }

      break;
  } /* switch */

  return;
}

/* ---------------------------------------------------------------------------
 *  void lin_slave_task_rx_data(void)
 * --------------------------------------------------------------------------*/
/**
   @brief  Cleans up all states after a successful frame receive
   @pre    LIN driver initialized
   @param  void
   @retval void
*/
void lin_slave_task_rx_data(void)
{
  l_u8 loc_n;

  switch (g_lin_frame_index)
  {
    /* invalid frame index */
    case (0xFF):
      break;

    /* diagnostic master request */
    case (0xFB):
      break;

    /* application frames */
    default:
    {
      if (g_lin_status_word.flag.last_pid != 0x0u)
      {
        g_lin_status_word.flag.overrun = L_SET;
      }

      g_lin_status_word.flag.last_pid = g_lin_active_pid;
      g_lin_status_word.flag.successful_transfer = L_SET;
      /* copy rx data to appl. frame buffer */
      g_lin_irqState = l_sys_irq_disable();

      for (loc_n = 0u; loc_n < g_lin_frame_data_size; loc_n++)
      {
        l_LinData.Frame[g_lin_frame_index].frame_data[loc_n] = lin_temp_buffer.frame.DataBuffer[loc_n];
      }

      l_sys_irq_restore(g_lin_irqState);
      /* Update Frame Flag */
      g_lin_frame_ctrl[g_lin_frame_index].frame.frame_type.update_flag = 1u;
      /* Update Signal flags */
      l_Update_flags_frame(g_lin_frame_index);
    }
    break;
  }
}


