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
*********************   Workfile:      lin_type.h         **********************
**                                                                            **
** PROJECT-DESCRIPTION:  LIN Driver Protocol Layer                            **
**                                                                            **
** FILE-DESCRIPTION:  All defines special data types for the lin driver       **
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
** DM           Daniel Mysliwitz                                              **
*******************************************************************************/

/*******************************************************************************
**                          Revision Control History                          **
********************************************************************************
** V1.0.0: 2014-02-18, HS:   New Demo driver for Infineon TLE987x             **
** V1.0.1: 2020-04-15, BG:   Updated abstract                                 **
** V1.0.2: 2020-07-31, DM:   l_s16 data type added                            **
*******************************************************************************/

#ifndef LIN_TYPE_H /* to interprete header file only once */
#define LIN_TYPE_H

/* ===========================================================================
 *  Type Definitions
 * ==========================================================================*/

typedef unsigned char  l_bool;                                                           /* Boolean Datatype */
typedef unsigned char  l_u8;                                                             /* 8Bit Datatype */
typedef unsigned short l_u16;                                                            /* 16Bit Datatype */
typedef signed short   l_s16;                                                            /* 16Bit Datatype */
typedef unsigned short l_irqmask, l_ioctl_op;                                            /* Datatypes for special LIN functions */
typedef unsigned short l_signal_handle, l_flag_handle, l_ifc_handle, l_schedule_handle;  /* Datatypes for special LIN functions */

/* ===========================================================================
 *  Structures
 * ==========================================================================*/

#if defined LIN_ENABLE_ASSIGN_FRAME_ID || defined LIN_ENABLE_RBI_RD_MSG_ID_PID
typedef union
{
  l_u8 reg[5];
  struct
  {
    struct
    {
      l_u8  lo;
      l_u8  hi;
    } msg_id;
    l_u8 pid;
    struct
    {
      l_u8  publisher               : 1;
      l_u8  update_flag             : 1;
      l_u8  res                     : 2;
      l_u8  transfer_type           : 4;
    } frame_type;
    l_u8     length;
  } frame;
} t_Lin_Frame_Ctrl;
#else
typedef union
{
  l_u8 reg[3];
  struct
  {
    l_u8   pid;
    struct
    {
      l_u8  publisher               : 1;
      l_u8  update_flag             : 1;
      l_u8  res                     : 2;
      l_u8  transfer_type           : 4;
    } frame_type;
    l_u8     length;
  } frame;
} t_Lin_Frame_Ctrl;
#endif /* end #ifdef LIN_ENABLE_ASSIGN_FRAME_ID */


typedef struct LIN_PROD_ID_s
{
  l_u8  NAD;
  l_u8  Supplier_lo;
  l_u8  Supplier_hi;
  l_u8  Function_lo;
  l_u8  Function_hi;
  l_u8  Variant;
  l_u8  Initial_NAD;
} t_lin_prod_id;

typedef union u_lin_status_word
{
  struct
  {
    l_u16   error_in_resp                 : 1;
    l_u16   successful_transfer           : 1;
    l_u16   overrun                       : 1;
    l_u16   goto_sleep                    : 1;
    l_u16   bus_activity                  : 1;
    l_u16   event_triggerd_frame_coll     : 1;
    l_u16   save_config                   : 1;
    l_u16   reserved                      : 1;
    l_u16   last_pid                      : 8;
  } flag;
  l_u16                 reg;
} t_lin_status_word;


typedef union
{
  struct LIN_TIMEOUT_CTRL
  {
    l_u8   autobaud        : 1;
    l_u8   frame           : 1;
    l_u8   transm_ongoing  : 1;
    l_u8   recept_started  : 1;
    l_u8   startup_config  : 1;
    l_u8   lock_init_uart  : 1;
    l_u8   res             : 2;
  } flag;
  l_u8 byte;
} t_lin_timeout_ctrl;

#endif /* end ifndef LIN_TYPE_H */
