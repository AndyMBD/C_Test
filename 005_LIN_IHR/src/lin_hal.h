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
********************   Workfile:    lin_hal.h              *********************
**                                                                            **
** PROJECT-DESCRIPTION:  Infineon LIN Slave Driver                            **
**                                                                            **
** FILE-DESCRIPTION:  All defines for lin_hal.c                               **
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


#ifndef LIN_HW_DRIVER_H /* to interprete header file only once */
#define LIN_HW_DRIVER_H

/* ===========================================================================
 *  Header Files
 * ==========================================================================*/

#include <tle_device.h>
#include "genLinConfig.h"

/* ===========================================================================
 *  Constants
 * ==========================================================================*/

#ifndef LIN_FRAME_TIMEOUT
  #define LIN_FRAME_TIMEOUT       LIN_AUTOBAUD_TIMEOUT
#endif /* ifndef LIN_FRAME_TIMEOUT */

#ifdef LIN_UART2
  /* All UART control and access registers*/
  #define LIN_UART_BUFFER          UART2->SBUF.bit.VAL
  #define LIN_UART_CONTROL         UART2->SCON.reg
  #define LIN_UART_MODE0           UART2->SCON.bit.SM0
  #define LIN_UART_MODE1           UART2->SCON.bit.SM1
  #define LIN_UART_RUN             UART2->SCON.bit.REN
  #define LIN_UART_9BITREC         UART2->SCON.bit.RB8

  #define LIN_UART_BCON_PRE        SCU->BCON2.bit.BRPRE
  #define LIN_UART_BCON_FDSEL      SCU->BGL2.bit.FD_SEL
  #define LIN_UART_BCON_BGL        SCU->BGL2.bit.BR_VALUE
  #define LIN_UART_BCON_BGH        SCU->BGH2.bit.BR_VALUE
  #define LIN_UART_BCON_RUN        SCU->BCON2.bit.R

  #define LIN_UART_IR_SET_EN       CPU->NVIC_ISER0.bit.Int_UART2
  #define LIN_UART_IR_PENDING      CPU->NVIC_ISPR0.bit.Int_UART2
  #define LIN_UART_IR_PRIO         CPU->NVIC_IPR2.bit.PRI_UART2
  #define LIN_UART_IR_EN           SCU->MODIEN2.bit.RIEN2

  #define LIN_UART_STATUS          SCU->LINST.reg
  #define LIN_UART_BR_SYN_DET      SCU->LINST.bit.BGSEL
  #define LIN_UART_BR_SYN_DET_EN   SCU->LINST.bit.BRDIS
  #ifdef AUTOBAUD_ENABLED
    /* Timer 21 is needed for baudrate synchronization when using UART 2*/
    #define LIN_TIMER_PRE         TIMER21->T2MOD.bit.T2PRE
    #define LIN_TIMER_PRE_EN      TIMER21->T2MOD.bit.PREN
    #define LIN_TIMER_COUNTER_EN  TIMER21->T2MOD.bit.DCEN
    #define LIN_TIMER_EDGE_SELECT TIMER21->T2MOD.bit.T2REGS
    #define LIN_TIMER_EXT_START   TIMER21->T2MOD.bit.T2RHEN
    #define LIN_TIMER_CAP_REL     TIMER21->T2CON.bit.CP_RL2
    #define LIN_TIMER_T_C_SEL     TIMER21->T2CON.bit.C_T2
    #define LIN_TIMER_EXT_EN      TIMER21->T2CON.bit.EXEN2
    #define LIN_TIMER_EXT_IR_EN   TIMER21->T2CON1.bit.EXF2EN
    #define LIN_TIMER_INT_IR_EN   TIMER21->T2CON1.bit.TF2EN
    #define LIN_TIMER_RUN         TIMER21->T2CON.bit.TR2
    #define LIN_TIMER_HIGH        TIMER21->T2H.bit.T2H
    #define LIN_TIMER_LOW         TIMER21->T2L.bit.T2L
    #define LIN_TIMER_CAP_H       TIMER21->RC2H.bit.RC2
    #define LIN_TIMER_CAP_L       TIMER21->RC2L.bit.RC2
  #endif /* end #ifdef AUTOBAUD_ENABLED */
#endif /* end #ifdef LIN_UART2 */

#ifdef LIN_UART1
  /* All UART control and access registers*/
  #define LIN_UART_BUFFER           UART1->SBUF.bit.VAL
  #define LIN_UART_CONTROL          UART1->SCON.reg
  #define LIN_UART_MODE0            UART1->SCON.bit.SM0
  #define LIN_UART_MODE1            UART1->SCON.bit.SM1
  #define LIN_UART_RUN              UART1->SCON.bit.REN
  #define LIN_UART_9BITREC          UART1->SCON.bit.RB8

  #define LIN_UART_BCON_PRE         SCU->BCON1.bit.BRPRE
  #define LIN_UART_BCON_FDSEL       SCU->BGL1.bit.FD_SEL
  #define LIN_UART_BCON_BGL         SCU->BGL1.bit.BR_VALUE
  #define LIN_UART_BCON_BGH         SCU->BGH1.bit.BR_VALUE
  #define LIN_UART_BCON_RUN         SCU->BCON1.bit.R

  #define LIN_UART_IR_SET_EN        CPU->NVIC_ISER0.bit.Int_UART1
  #define LIN_UART_IR_PENDING       CPU->NVIC_ISPR0.bit.Int_UART1
  #define LIN_UART_IR_PRIO          CPU->NVIC_IPR2.bit.PRI_UART1
  #define LIN_UART_IR_EN            SCU->MODIEN1.bit.RIEN1
  #define LIN_UART_IRTX_EN          SCU->MODIEN1.bit.TIEN1

  #define LIN_UART_STATUS           SCU->LINST.reg
  #define LIN_UART_CLR_STATUS       SCU->LINSCLR.reg
  #define LIN_UART_BR_SYN_DET       SCU->LINST.bit.BGSEL
  #define LIN_UART_BR_SYN_DET_EN    SCU->LINST.bit.BRDIS

  #ifdef AUTOBAUD_ENABLED
    /* Timer 2 is needed for baudrate synchronization when using UART 1 */
    #define LIN_TIMER_PRE          TIMER2->T2MOD.bit.T2PRE
    #define LIN_TIMER_PRE_EN       TIMER2->T2MOD.bit.PREN
    #define LIN_TIMER_COUNTER_EN   TIMER2->T2MOD.bit.DCEN
    #define LIN_TIMER_EDGE_SELECT  TIMER2->T2MOD.bit.T2REGS
    #define LIN_TIMER_EXT_START    TIMER2->T2MOD.bit.T2RHEN
    #define LIN_TIMER_CAP_REL      TIMER2->T2CON.bit.CP_RL2
    #define LIN_TIMER_T_C_SEL      TIMER2->T2CON.bit.C_T2
    #define LIN_TIMER_EXT_EN       TIMER2->T2CON.bit.EXEN2
    #define LIN_TIMER_EXT_IR_EN    TIMER2->T2CON1.bit.EXF2EN
    #define LIN_TIMER_INT_IR_EN    TIMER2->T2CON1.bit.TF2EN
    #define LIN_TIMER_RUN          TIMER2->T2CON.bit.TR2
    #define LIN_TIMER_HIGH         TIMER2->T2H.bit.T2H
    #define LIN_TIMER_LOW          TIMER2->T2L.bit.T2L
    #define LIN_TIMER_CAP_H        TIMER2->RC2H.bit.RC2
    #define LIN_TIMER_CAP_L        TIMER2->RC2L.bit.RC2
  #endif /* end #ifdef AUTOBAUD_ENABLED */
#endif /* end #ifdef LIN_UART1 */

#define BREAK_FIELD_k  0x0u
#define SYNC_FIELD_k    0x55u

#ifndef CHECKSUM_NEXT
  #define CHECKSUM_NEXT 2u
#endif

#define lin_hal_tx_char(x)        (LIN_UART_BUFFER = (x))
#define lin_hal_is_byte_received() (lin_hal_status_g.byte_received == L_SET)
#define lin_hal_reset_byte_received() (lin_hal_status_g.byte_received = L_RESET)

#define lin_hal_set_rx_pid(pid)   (lin_temp_buffer.frame.PID = (pid))
#define lin_hal_set_tx_pid(pid)   (lin_temp_buffer.frame.PID = (pid))

/* ===========================================================================
 *  Structures
 * ==========================================================================*/

typedef struct _lin_hal_stat_reg
{
  l_u8 byte_received : 1;
  l_u8 ferror : 1;
  l_u8 res0: 6;
} LIN_hal_stat_reg_t;

typedef enum _lin_slave_state
{
  idle = 0,
  break_received,
  sync_received,
  rx_data,
  rx_checksum,
  tx_data,
  tx_checksum,
} LIN_SLAVE_STATE_t;

typedef union s_lin_temp_frame_buffer
{
  l_u8 byte[11];
  struct
  {
    l_u8 PID;
    l_u8 DataBuffer[8];
    l_u8 CRC;       /* used for subscribe frames CRC */
    l_u8 BufferIndex;
  } frame;
} t_lin_temp_frame_buffer;

/**
@struct LIN_TX_DATA_s
@brief Stucture to build a Transmit Frame
*/
typedef struct LIN_TX_DATA_s
{
  l_u8 length_c;     /* length of frame data */
  l_u8 *data_puc;    /* pointer to frame data */
  l_u8 old_data_uc;  /* tx data for check */
  l_u8 index_uc;     /* index to fill receive buffer with transmited data */
  l_u8 checksum;     /* checksum for data transmit */
} LIN_TX_DATA_t;

/**
@struct LIN_FLAGS_s
@brief Stucture for transmit each byte of a Frame
*/
typedef struct LIN_FLAGS_s
{
  l_u8 tx_next   : 2; /* Indicates if next byte to transmit or not */
  l_u8 tx_finished  : 1; /* Indicated if transmition is finished */
  l_u8 res0 : 5;
} LIN_FLAGS_t;

/**
@struct LIN_ERROR_STATE_s
@brief Stucture for Error State of the Lin Driver
*/
typedef union
{
  struct LIN_ERROR_STATE_s
  {
    l_u8 framing   : 1;
    l_u8 overflow   : 1;
    l_u8 invalid_break : 1;
    l_u8 invalid_field : 1;
    l_u8 bus_error  : 1;
    l_u8 checksum  : 1;
    l_u8 timeout  : 1;
    l_u8 autobaud_error   : 1;
  } e_bit;
  l_u8 byte;
} LIN_ERROR_STATE_t;

/* ===========================================================================
 *  Function Prototypes
 * ==========================================================================*/

extern void lin_hal_init (void);
#ifdef AUTOBAUD_ENABLED
  extern void lin_hal_init_T2 (void);
#endif /* #ifdef AUTOBAUD_ENABLED */
extern void lin_hal_rx_response (l_u8 l_type, l_u8 l_len);
extern void lin_hal_tx_response (l_u8 l_type, l_u8 l_data[], l_u8 l_len);

void l_hal_txchecksum_calculation(void);
void l_hal_rx_state_machine(void);
void l_hal_tx_state_machine(void);
void lin_hal_ISR(void);
void lin_hal_tx_wake_up(void);
void lin_hal_tx_checksum(l_u8 checksum);
void lin_hal_tx_byte(l_u8 lin_hal_bytetoTX);
void lin_hal_init_uart(void);

/* ===========================================================================
 *  Function Macros
 * ==========================================================================*/

#ifdef AUTOBAUD_ENABLED
/* Autobaud synchronization, timer runs with no prescaler, formulae allows to calculate necessary baudrate
 * settings only with shifting.
 * If not within valid limits, increase timeout timer to cause next call of ld_task to reset UART.
 */
#define lin_hal_brreg_calc()  do { lin_synch_time = (lin_synch_time >> 2u) >> BRG_PRE;                             \
                                if (BRG_PRE == 0u) { LIN_UART_BCON_FDSEL = ((l_u8)(lin_synch_time & 0x1Fu)) >> 1u; \
                                } else { LIN_UART_BCON_FDSEL   = (l_u8)(lin_synch_time & 0x1Fu); }                 \
                                lin_synch_time = (lin_synch_time >> 5u);                                           \
                                if (lin_synch_time >= MIN_BDREG)                                                   \
                                { if (lin_synch_time <= MAX_BDREG)                                                 \
                                  { LIN_UART_BCON_BGL = (l_u8)(lin_synch_time & 0x07u);                            \
                                    LIN_UART_BCON_BGH = (l_u8)(lin_synch_time >> 3u);                              \
                                    lin_slave_state_g = sync_received;                                             \
                                  } else {                                                                         \
                                    lin_ab_timeout_timer = LIN_AUTOBAUD_TIMEOUT + 1u;                              \
                                    lin_ab_t2_state = 7u; }                                                        \
                                } else {                                                                           \
                                  lin_ab_timeout_timer = LIN_AUTOBAUD_TIMEOUT + 1u;                                \
                                  lin_ab_t2_state = 7u; }                                                          \
                                LIN_UART_BCON_RUN = 1u; } while(0)

#define lin_hal_T2_ISR()      do { if (lin_ab_t2_state == 5u)                                                      \
                              { LIN_TIMER_RUN = 0u;LIN_TIMER_EXT_IR_EN = 0u;                                       \
                                LIN_TIMER_EXT_EN = 0u;lin_ab_t2_state = 0u;                                        \
                                lin_synch_time = ((l_u16)LIN_TIMER_CAP_H << 8) | (l_u16)LIN_TIMER_CAP_L;           \
                                lin_hal_brreg_calc();                                                              \
                              } else { lin_ab_t2_state++; } } while(0)
#endif /* ifdef AUTOBAUD_ENABLED */

#define l_hal_cyclic_call()   do { if(lin_hal_is_byte_received())                                                  \
                              { l_hal_rx_state_machine();                                                          \
                                lin_hal_reset_byte_received(); }                                                   \
                                if (l_txchk_byte_nr > 0u)                                                          \
                                { l_hal_txchecksum_calculation(); } }while(0)

#define l_autobaud_control()  do { if(lin_ab_t2_state != 0u)                                                       \
                              { lin_ab_timeout_timer++;                                                            \
                                if(lin_ab_timeout_timer >= LIN_AUTOBAUD_TIMEOUT)                                   \
                                { lin_slave_state_g = idle;                                                        \
                                  lin_hal_init_uart();                                                             \
                                  lin_ab_t2_state = 0u;                                                            \
                                  lin_ab_timeout_timer = 0u; } } } while(0)

#if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1
#define l_frame_timeout_control()  do { if(lin_timeout_ctrl.flag.frame != 0u)                                      \
                                   { lin_frm_timeout_timer++;                                                      \
                                     if(lin_frm_timeout_timer >= LIN_FRAME_TIMEOUT)                                \
                                     { lin_slave_state_g = idle;                                                   \
                                       lin_hal_init_uart();                                                        \
                                       if (lin_timeout_ctrl.flag.recept_started != 0u)                             \
                                       { l_Set_Response_Error_Flag();                                              \
                                         lin_timeout_ctrl.flag.recept_started = 0u; }                              \
                                       if (lin_timeout_ctrl.flag.transm_ongoing != 0u)                             \
                                       { lin_timeout_ctrl.flag.transm_ongoing = 0u;                                \
                                         if ((g_lin_frame_index >= LIN_NUMBER_OF_FRAMES) ||                        \
                                            ((g_lin_frame_ctrl[g_lin_frame_index].frame.frame_type.transfer_type & 0x01u) == 0u)) \
                                         { l_Set_Response_Error_Flag(); } }                                        \
                                       lin_timeout_ctrl.flag.frame = 0u;                                           \
                                       lin_frm_timeout_timer = 0u; } } } while (0)
#else
#define l_frame_timeout_control()  do { if(lin_timeout_ctrl.flag.frame != 0u)                                      \
                                   { lin_frm_timeout_timer++;                                                      \
                                     if (lin_frm_timeout_timer >= LIN_FRAME_TIMEOUT)                               \
                                     { lin_slave_state_g = idle;                                                   \
                                       lin_hal_init_uart();                                                        \
                                       lin_timeout_ctrl.flag.frame = 0u;                                           \
                                       lin_frm_timeout_timer = 0u; } } } while (0)
#endif /* end #if defined LIN_PROTOCOL_VERSION_2_0 || defined LIN_PROTOCOL_VERSION_2_1 */

#ifdef AUTOBAUD_ENABLED
#define l_hal_baudrate_control()   do { if (lin_slave_state_g == idle)                                             \
                                   { lin_temp_word = (LIN_UART_BCON_BGH << 3) | (LIN_UART_BCON_BGL & 0x07u);       \
                                     if (lin_temp_word > (MAX_BDREG))                                              \
                                     { LIN_UART_BCON_FDSEL = BRG_FD_SEL; LIN_UART_BCON_BGH = BRGH_VAL;             \
                                       LIN_UART_BCON_BGL = BRGL_VAL;   }                                           \
                                     if (lin_temp_word < (MIN_BDREG))                                              \
                                     { LIN_UART_BCON_FDSEL = BRG_FD_SEL; LIN_UART_BCON_BGH = BRGH_VAL;             \
                                       LIN_UART_BCON_BGL = BRGL_VAL; } } } while (0u)
#else /* end #ifdef AUTOBAUD_ENABLED */
#define l_hal_baudrate_control()   do { } while (0u)
#endif /* end #ifdef AUTOBAUD_ENABLED */

/* ===========================================================================
 *  Variables
 * ==========================================================================*/

extern LIN_hal_stat_reg_t lin_hal_status_g;
extern t_lin_temp_frame_buffer lin_temp_buffer;
extern LIN_TX_DATA_t lin_tx_data_g;
extern LIN_FLAGS_t lin_flag_g;
extern LIN_SLAVE_STATE_t lin_slave_state_g;
extern l_u8 lin_ab_timeout_timer;
extern l_u8 lin_frm_timeout_timer;
extern t_lin_timeout_ctrl lin_timeout_ctrl;
extern l_u8 l_txchk_byte_nr;
extern volatile l_u8 lin_ab_t2_state;
#ifdef AUTOBAUD_ENABLED
  extern volatile l_u16 lin_temp_word;
  extern volatile l_u16 lin_synch_time;
  extern l_u8 lin_hal_rx_data_guc;
#endif /* end #ifdef AUTOBAUD_ENABLED */

#endif /* end #ifndef LIN_HW_DRIVER_H */





