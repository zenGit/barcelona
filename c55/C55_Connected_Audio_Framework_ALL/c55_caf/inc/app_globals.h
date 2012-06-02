/*
 * $$$MODULE_NAME app_globals.h
 *
 * $$$MODULE_DESC app_globals.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _APP_GLOBALS_H_
#define _APP_GLOBALS_H_

#define SAMPLE_BY_SAMPLE_PB	              /* enable sample by sample output to codec */	

#define SAMPLE_BY_SAMPLE_REC              /* enable sample by sample input from codec */

#ifdef SAMPLE_BY_SAMPLE_PB
#define COUNT_REC_SAMPS_IN_USB_FRAME_PB  /* enable counting of record samples in I2S Tx ISR */
#endif

#ifdef SAMPLE_BY_SAMPLE_REC
#define COUNT_REC_SAMPS_IN_USB_FRAME_REC  /* enable counting of record samples in I2S Rx ISR */
#endif

#if defined(SAMPLE_BY_SAMPLE_PB) && defined(SAMPLE_BY_SAMPLE_REC)
//#define COMBINE_I2S_TX_RX_ISR           /* enable combined I2S Rx/Tx ISR */
#endif

#define START_REC_WAIT_FRAMES ( 50 )    /* no. frames to wait before initiating record */

/* Get the most significant byte of a word */
#define MSB(s)          ((Uint8)((0xFF00 & ((Uint16)(s))) >> 8))
/* Get the least significant byte of a word */
#define LSB(s)          ((Uint8)(0x00FF & ((Uint16)(s))))

#endif /* _APP_GLOBALS_H_ */
