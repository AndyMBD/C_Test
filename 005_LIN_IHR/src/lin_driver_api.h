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
********************   Workfile:      lin_driver_api.h     *********************
**                                                                            **
** PROJECT-DESCRIPTION:  LIN Driver Protocol Layer                            **
**                                                                            **
** FILE-DESCRIPTION:  All defines for lin_driver_api.c                        **
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

#ifndef LIN_DRIVER_API_H /* to interprete header file only once */
#define LIN_DRIVER_API_H

#include "genLinConfig.h"
#include "lin_type.h"
#include "lin_main.h"
#include "lin_hal.h"


/* ===========================================================================
 *  Constants
 * ==========================================================================*/



/* ===========================================================================
 *  Structures
 * ==========================================================================*/

/* ===========================================================================
 *  Prototypes
 * ==========================================================================*/

l_bool l_sys_init (void);
l_bool l_ifc_init (void);
l_u16 l_ifc_ioctl (l_ioctl_op op, void *pv);
l_u16 l_ifc_read_status(void);
l_irqmask l_sys_irq_disable (void);
void l_sys_irq_restore (l_irqmask mask);
void LIN_Enable_Transceiver(void);
void LIN_Disable_Transceiver(void);

#ifdef LIN_MASTER
  l_u8 l_sch_tick ();
  void l_sch_set (l_u16, l_u8);
  void l_ifc_goto_sleep (void);
  l_u8 ld_is_ready (l_ifc_handle iii);
  void ld_check_response (l_ifc_handle iii, l_u8 *RSID, l_u8 *error_code);
  void ld_assign_frame_id_range (l_ifc_handle iii, l_u8 NAD, l_u8 start_index, l_u8 *PIDs);
  void ld_assign_NAD (l_ifc_handle iii, l_u8 initial_NAD, l_u16 supplier_id, l_u16 function_id, l_u8 new_NAD);
  void ld_conditional_change_NAD (l_ifc_handle iii, l_u8 NAD, l_u8 id, l_u8 byte, l_u8 mask, l_u8 invert, l_u8 new_NAD);
  void ld_read_by_id (l_ifc_handle iii, l_u8 NAD, l_u16 supplier_id, l_u16 function_id, l_u8 id, l_u8 *const data);
  #ifdef LIN_PROTOCOL_VERSION_2_1
    void ld_save_configuration (l_ifc_handle iii, l_u8 NAD);
  #endif /* end #ifdef LIN_PROTOCOL_VERSION_2_1 */
#endif /* ifdef LIN_MASTER */

#ifdef LIN_SLAVE
  #ifdef LIN_PROTOCOL_VERSION_2_1
    l_u8 ld_read_configuration (l_u8 *const data, l_u8 *const length);
    l_u8 ld_set_configuration (const l_u8 *const data, l_u8 *const length);
  #endif /* end #ifdef LIN_PROTOCOL_VERSION_2_1 */
#endif /* end #ifdef LIN_SLAVE */


/* TASK for LIN Communication outside the ISR. */
#define l_cyclic_com_task()            l_hal_cyclic_call()

#define l_ifc_wake_up()    lin_hal_tx_wake_up();
#define l_ifc_rx()         lin_hal_ISR()
#define l_ifc_tx()
#define l_ifc_aux()
#ifdef AUTOBAUD_ENABLED
  #define l_ifc_t2()         lin_hal_T2_ISR()
#endif /* #ifdef AUTOBAUD_ENABLED */

#ifndef LIN_PROTOCOL_VERSION_1_3
#define ld_task()                do { l_frame_timeout_control();              \
                                         l_autobaud_control();                   \
                                         l_hal_baudrate_control(); } while (0)
#else
#define ld_task()                   do { l_autobaud_control();                \
                                         l_hal_baudrate_control(); } while (0)
#endif /* end #ifndef LIN_PROTOCOL_VERSION_1_3 */


#endif /* end #ifndef LIN_DRIVER_API_H */
