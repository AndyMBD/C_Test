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
********************   Workfile:      lin_main.h        ************************
**                                                                            **
** PROJECT-DESCRIPTION:  LIN Driver Protocol Layer                            **
**                                                                            **
** FILE-DESCRIPTION:  All defines for lin_main.c                              **
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

#ifndef LIN_MAIN_H /* to interprete header file only once */
#define LIN_MAIN_H

#include "genLinConfig.h"

/* ===========================================================================
 *  Constants
 * ==========================================================================*/

#ifndef L_SET
  #define L_SET                       1u
#endif /* end #ifdef L_SET */

#ifndef L_RESET
  #define L_RESET                     0u
#endif /* end #ifdef L_RESET */

#define PUBLISH                        1u
#define SUBSCRIBE                      0u


#define UNCONDITIONAL                  0u
#define LIN_2_X                        20u
#define LIN_1_3                        13u

#if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1
  #define LIN_PROTOCOL_TYPE           20u
#else
  #define LIN_PROTOCOL_TYPE           13u
#endif


/* ===========================================================================
 *  Structures
 * ==========================================================================*/

/* ===========================================================================
 *  Function Prototypes
 * ==========================================================================*/

void lin_main_init(void);

/* ===========================================================================
 *  Variables
 * ==========================================================================*/

extern l_u8               g_lin_frame_index;
extern t_lin_prod_id      g_lin_prod_id;
extern t_Lin_Frame_Ctrl   g_lin_frame_ctrl[LIN_NUMBER_OF_FRAMES];
extern t_lin_status_word  g_lin_status_word;
extern l_u8               g_lin_active_pid;
extern l_irqmask          g_lin_irqState;
extern l_u8               g_lin_frame_data_size;

#endif /* end #ifndef LIN_MAIN_H */
