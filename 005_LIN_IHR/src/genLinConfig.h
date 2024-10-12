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
*********************    Workfile:    genLinConfig.h      **********************
**                                                                            **
**  PROJECT-DESCRIPTION:  Infineon LIN Driver                                 **
**                                                                            **
**  FILE-DESCRIPTION:  Generated LIN Configuration File                       **
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
** JO           Julia Ott                                                     **
*******************************************************************************/

/*******************************************************************************
**                          Revision Control History                          **
********************************************************************************
** V1.0.0: 2014-02-18, HS:   New Demo driver for Infineon TLE987x             **
** V1.0.1: 2020-04-15, BG:   Updated abstract                                 **
** V1.1.0: 2020-07-31, DM:   l_u16_rd_RefSpeed changed to l_s16_rd_RefSpeed   **
** V1.2.0: 2022-07-07, JO:   EP-1206: Changed l_s16_rd_RefSpeed to            **
**                           l_u16_rd_RefSpeed and l_s16_wr_ActSpeed to       **
**                           l_u16_wr_ActSpeed for signal in range [0...5000] **
*******************************************************************************/

/* Generated by LDCTool 2.0.4.1      Date: 17.09.2015 */
/* Generated from LDF: c:\userdata\mysliwid\02_produkte\03_proteus\01_software\02_b-step\tle9879\blabc\boards\infineon\tle9879_evalkit\bldc_bc_hall\lin_driver\lin_ldf.ldf */
#ifndef        GEN_LIN_CONFIG_H
#define        GEN_LIN_CONFIG_H


/* LIN Attributes */
#define        LIN_BAUDRATE 19200    /* for CPU speed of 24 MHz */

/* Node Attributes */

/* Generated Node: Motor */

#define        LIN_SLAVE
#define        LIN_PROTOCOL_VERSION_2_1
#define        LIN_INITIAL_NAD    (l_u8)1
#define        LIN_SUPPLIER_ID    (l_u16)4660
#define        LIN_FUNCTION_ID    (l_u16)22136
#define        LIN_VARIANT_ID     (l_u8)0

/* Access Macros for Node Information */
#define        l_get_current_NAD()        (g_lin_prod_id.NAD)
#define        l_set_current_NAD(x)       (g_lin_prod_id.NAD = (x))
#define        l_get_initial_NAD()        (g_lin_prod_id.Initial_NAD)
#define        l_get_SupplierID_Low()     (g_lin_prod_id.Supplier_lo)
#define        l_get_SupplierID_High()    (g_lin_prod_id.Supplier_hi)
#define        l_get_FunctionID_Low()     (g_lin_prod_id.Function_lo)
#define        l_get_FunctionID_High()    (g_lin_prod_id.Function_hi)
#define        l_get_Variant()            (g_lin_prod_id.Variant)


/* User Defined Attributes */

/* User Defined Diagnostic Attributes */

#define        LIN_DIAG_ENABLE
#define        LIN_COOKED_API
#define        LIN_TASK_CYCLE_MS    1u
#define        LIN_FRAME_TIMEOUT    10u
#define        LIN_AUTOBAUD_TIMEOUT    4u

/* User Defined Node Attributes */

#define        LIN_ENABLE_ASSIGN_FRAME_ID_RANGE


/* User Defined MCU Attributes */

/* TLE9879 */

#define        BRG_PRE       (l_u8)0
#define        BRGH_VAL      (l_u8)9
#define        BRGL_VAL      (l_u8)6
#define        BRG_FD_SEL    (l_u8)4
#define        BG_SEL        (l_u8)3
#define        LIN_UART1
#define        AUTOBAUD_ENABLED
#define        MAX_BDREG        ((l_u16)(86u & 0x3FFu))
#define        MIN_BDREG        ((l_u16)(70u & 0x3FFu))
#define        LIN_INTERRUPT_PRIORITY  (l_u8)5
#define        STRUCT_ATTRIBUTE
#define        STRUCT16BIT

/* Generate the Signal Attributes */

#include "lin_type.h"


/* Frame Attributes */

#define        LIN_NUMBER_OF_FRAMES    (l_u8)2

#define        LIN_FRAME_CTRL_INIT {{{(l_u8)17,(l_u8)128,(l_u8)8}},{{(l_u8)146,(l_u8)129,(l_u8)8}}}

/* Notification Flag API, PID and Message ID Access for whole frames */

/* Use only with LIN 2.0 */
#define        l_get_msg_id_low_frame(x)     (g_lin_frame_ctrl[(x)].frame.msg_id.lo)
#define        l_get_msg_id_high_frame(x)    (g_lin_frame_ctrl[(x)].frame.msg_id.hi)
/* Use only with LIN 2.X */
#define        l_get_pid_frame(x)            (g_lin_frame_ctrl[(x)].frame.pid)
#define        l_set_pid_frame(x,y)          (g_lin_frame_ctrl[(x)].frame.pid = (y))
#define        l_flg_tst_frm_MotorCtrl()    (g_lin_frame_ctrl[0].frame.frame_type.update_flag == 1u)
#define        l_flg_clr_frm_MotorCtrl()    (g_lin_frame_ctrl[0].frame.frame_type.update_flag = 0u)
#define        LIN_FRAME_MotorCtrl()    (g_lin_frame_ctrl[0].frame.pid)
#define        LIN_CHANGE_PID_FRAME_MotorCtrl(x)    (g_lin_frame_ctrl[0].frame.pid = (x))
#define        LIN_FRAME_LOW_MotorCtrl_MSG_ID()    (g_lin_frame_ctrl[0].frame.msg_id.lo)
#define        LIN_FRAME_HIGH_MotorCtrl_MSG_ID()    (g_lin_frame_ctrl[0].frame.msg_id.hi)
#define        l_flg_tst_frm_MotorStatus()    (g_lin_frame_ctrl[1].frame.frame_type.update_flag == 1u)
#define        l_flg_clr_frm_MotorStatus()    (g_lin_frame_ctrl[1].frame.frame_type.update_flag = 0u)
#define        LIN_FRAME_MotorStatus()    (g_lin_frame_ctrl[1].frame.pid)
#define        LIN_CHANGE_PID_FRAME_MotorStatus(x)    (g_lin_frame_ctrl[1].frame.pid = (x))
#define        LIN_FRAME_LOW_MotorStatus_MSG_ID()    (g_lin_frame_ctrl[1].frame.msg_id.lo)
#define        LIN_FRAME_HIGH_MotorStatus_MSG_ID()    (g_lin_frame_ctrl[1].frame.msg_id.hi)

/* Signal Attributes */

/* Signal to Frame Mapping and Signal Update Flags */

typedef union
{
  struct
  {
    l_u16 RefSpeed_f  : 1;
    l_u16 res00  : 7;
    l_u16 res01  : 8;
  } flags;
  l_u16 reg[1];
} t_l_flags_MotorCtrl;

extern volatile t_l_flags_MotorCtrl l_flags_MotorCtrl;

typedef union
{
  struct
  {
    l_u16 ActSpeed_f  : 1;
    l_u16 Error_f  : 1;
    l_u16 res00  : 6;
    l_u16 res01  : 8;
  } flags;
  l_u16 reg[1];
} t_l_flags_MotorStatus;

extern volatile t_l_flags_MotorStatus l_flags_MotorStatus;

typedef union
{
  l_u8    frame_data[8];
  l_u8    *dataptr;
} t_l_FrmData;

typedef union
{
  l_u8    dataBytes[16];
  struct
  {
    t_l_FrmData    l_frm_MotorCtrl;
    t_l_FrmData    l_frm_MotorStatus;
  } frames;
  t_l_FrmData    Frame[LIN_NUMBER_OF_FRAMES];
} t_l_Lin_Data;
extern t_l_Lin_Data    l_LinData;

/* Signal Access Macros */

/* Signal Read Access for Signal RefSpeed */
#define        l_u16_rd_RefSpeed()    (l_u16)((l_LinData.frames.l_frm_MotorCtrl.frame_data[0])|\
                                    (((l_LinData.frames.l_frm_MotorCtrl.frame_data[1])&(255u))<<8u))
#define        l_flg_tst_RefSpeed()    (l_flags_MotorCtrl.flags.RefSpeed_f == 1u)
#define        l_flg_clr_RefSpeed()    (l_flags_MotorCtrl.flags.RefSpeed_f = 0u)

/* Signal Write Access for Signal ActSpeed */
#define        l_u16_wr_ActSpeed(x)    do{ l_LinData.frames.l_frm_MotorStatus.frame_data[0]&=(0u); \
                                    l_LinData.frames.l_frm_MotorStatus.frame_data[1]&=(0u); \
                                    l_LinData.frames.l_frm_MotorStatus.frame_data[0]|=(l_u8)((l_u16)((x)<<0u)); \
                                    l_LinData.frames.l_frm_MotorStatus.frame_data[1]|=(l_u8)((l_u16)((x)>>8u)); \
                                    g_lin_frame_ctrl[1].frame.frame_type.update_flag = 1u; \
                                    }while(0)

/* Signal Write Access for Signal Error */
#define        l_bool_wr_Error(x)    do{ l_LinData.frames.l_frm_MotorStatus.frame_data[2]&=(254u); \
                                    l_LinData.frames.l_frm_MotorStatus.frame_data[2]|=((l_u8)((x)<<0u)); \
                                    g_lin_frame_ctrl[1].frame.frame_type.update_flag = 1u; \
                                    }while(0)

/* Lin Driver Access to Update Signal Flags for Frame MotorCtrl */
#define        l_Update_flags_frame0() l_flags_MotorCtrl.reg[0]=(l_u16)0xFFFF;

/* Lin Driver Access to Update Signal Flags for Frame MotorStatus */
#define        l_Update_flags_frame1() l_flags_MotorStatus.reg[0]=(l_u16)0xFFFF;

#define        l_Update_flags_frame(x) l_Update_Frame_Flags((x))
void l_Update_Frame_Flags(l_u8 number);

/* Prototype for Byte Array Signal Access Function  */
void l_get_byte_array(l_u8 start[], l_u8  count, l_u8 destination[]);
void l_set_byte_array(l_u8 start[], l_u8  count, const l_u8 source[]);

/* Macros for Response Error Flag / Frame handling */

#define        l_Set_Response_Error_Flag() l_bool_wr_Error(1u)

#define        l_Reset_Response_Error_Flag() l_bool_wr_Error(0u)

#define        LIN_Response_Error_Frame_PID g_lin_frame_ctrl[1].frame.pid


/* File Footer */
#endif /* end #ifndef GEN_LIN_CONFIG_H */

/* genLinConfig.h file for TLE9879 */
