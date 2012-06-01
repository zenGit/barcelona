/*
 * $$$MODULE_NAME i2s_sample.h
 *
 * $$$MODULE_DESC i2s_sample.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _I2S_SAMPLE_H_
#define _I2S_SAMPLE_H_

#include "csl_intc.h"
#include "app_globals.h"
#include "ddc_i2s.h"
#include "psp_i2s.h"

/* Playback I2S selection */
#define USE_I2S0_PB /* define to use I2S0 */
//#define USE_I2S1_PB /* define to use I2S1 */
//#define USE_I2S2_PB /* define to use I2S2 */
//#define USE_I2S3_PB /* define to use I2S3 */

/* Record I2S selection */
#define USE_I2S0_REC /* define to use I2S0 */
//#define USE_I2S1_REC /* define to use I2S1 */
//#define USE_I2S2_REC /* define to use I2S2 */
//#define USE_I2S3_REC /* define to use I2S3 */

/* I2S registers */
#define I2S0_I2STXLT0               ( 0x2808 )  /* I2S0 Transmit Left Data 0 Register */
#define I2S0_I2STXLT1               ( 0x2809 )  /* I2S0 Transmit Left Data 1 Register */
#define I2S0_I2STXRT0               ( 0x280C )  /* I2S0 Transmit Right Data 0 Register */
#define I2S0_I2STXRT1               ( 0x280D )  /* I2S0 Transmit Right Data 1 Register */
#define I2S1_I2STXLT0               ( 0x2908 )  /* I2S1 Transmit Left Data 0 Register */
#define I2S1_I2STXLT1               ( 0x2909 )  /* I2S1 Transmit Left Data 1 Register */
#define I2S1_I2STXRT0               ( 0x290C )  /* I2S1 Transmit Right Data 0 Register */
#define I2S1_I2STXRT1               ( 0x290D )  /* I2S1 Transmit Right Data 1 Register */
#define I2S2_I2STXLT0               ( 0x2A08 )  /* I2S2 Transmit Left Data 0 Register */
#define I2S2_I2STXLT1               ( 0x2A09 )  /* I2S2 Transmit Left Data 1 Register */
#define I2S2_I2STXRT0               ( 0x2A0C )  /* I2S2 Transmit Right Data 0 Register */
#define I2S2_I2STXRT1               ( 0x2A0D )  /* I2S2 Transmit Right Data 1 Register */
#define I2S3_I2STXLT0               ( 0x2B08 )  /* I2S3 Transmit Left Data 0 Register */
#define I2S3_I2STXLT1               ( 0x2B09 )  /* I2S3 Transmit Left Data 1 Register */
#define I2S3_I2STXRT0               ( 0x2B0C )  /* I2S3 Transmit Right Data 0 Register */
#define I2S3_I2STXRT1               ( 0x2B0D )  /* I2S3 Transmit Right Data 1 Register */

#define I2S0_I2SRXLT0               ( 0x2828 )  /* I2S0 Receive Left Data 0 Register */
#define I2S0_I2SRXLT1               ( 0x2829 )  /* I2S0 Receive Left Data 1 Register */
#define I2S0_I2SRXRT0               ( 0x282C )  /* I2S0 Receive Right Data 0 Register */
#define I2S0_I2SRXRT1               ( 0x282D )  /* I2S0 Receive Right Data 1 Register */
#define I2S1_I2SRXLT0               ( 0x2928 )  /* I2S1 Receive Left Data 0 Register */
#define I2S1_I2SRXLT1               ( 0x2929 )  /* I2S1 Receive Left Data 1 Register */
#define I2S1_I2SRXRT0               ( 0x292C )  /* I2S1 Receive Right Data 0 Register */
#define I2S1_I2SRXRT1               ( 0x292D )  /* I2S1 Receive Right Data 1 Register */
#define I2S2_I2SRXLT0               ( 0x2A28 )  /* I2S2 Receive Left Data 0 Register */
#define I2S2_I2SRXLT1               ( 0x2A29 )  /* I2S2 Receive Left Data 1 Register */
#define I2S2_I2SRXRT0               ( 0x2A2C )  /* I2S2 Receive Right Data 0 Register */
#define I2S2_I2SRXRT1               ( 0x2A2D )  /* I2S2 Receive Right Data 1 Register */
#define I2S3_I2SRXLT0               ( 0x2B28 )  /* I2S3 Receive Left Data 0 Register */
#define I2S3_I2SRXLT1               ( 0x2B29 )  /* I2S3 Receive Left Data 1 Register */
#define I2S3_I2SRXRT0               ( 0x2B2C )  /* I2S3 Receive Right Data 0 Register */
#define I2S3_I2SRXRT1               ( 0x2B2D )  /* I2S3 Receive Right Data 1 Register */

#ifdef USE_I2S0_PB
#define PSP_I2S_TX_INST_ID  ( PSP_I2S_0 )
#define I2S_TX_EVENT        ( PROG0_EVENT )
#define I2S_I2STXLT1        ( I2S0_I2STXLT1 )
#define I2S_I2STXRT1        ( I2S0_I2STXRT1 )
#elif defined USE_I2S1_PB
#define PSP_I2S_TX_INST_ID  ( PSP_I2S_1 )
#define I2S_TX_EVENT        ( PROG2_EVENT )
#define I2S_I2STXLT1        ( I2S1_I2STXLT1 )
#define I2S_I2STXRT1        ( I2S1_I2STXRT1 )
#elif defined USE_I2S2_PB
#define PSP_I2S_TX_INST_ID  ( PSP_I2S_2 )
#define I2S_TX_EVENT        ( XMT2_EVENT )
#define I2S_I2STXLT1        ( I2S2_I2STXLT1 )
#define I2S_I2STXRT1        ( I2S2_I2STXRT1 )
#elif defined USE_I2S3_PB
#define PSP_I2S_TX_INST_ID  ( PSP_I2S_3 )
#define I2S_TX_EVENT        ( XMT3_EVENT )
#define I2S_I2STXLT1        ( I2S3_I2STXLT1 )
#define I2S_I2STXRT1        ( I2S3_I2STXRT1 )
#endif

#ifdef USE_I2S0_REC
#define PSP_I2S_RX_INST_ID  ( PSP_I2S_0 )
#define I2S_RX_EVENT        ( PROG1_EVENT )
#define I2S_I2SRXLT1        ( I2S0_I2SRXLT1 )
#define I2S_I2SRXRT1        ( I2S0_I2SRXRT1 )
#elif defined USE_I2S1_REC
#define PSP_I2S_RX_INST_ID  ( PSP_I2S_1 )
#define I2S_RX_EVENT        ( PROG3_EVENT )
#define I2S_I2SRXLT1        ( I2S1_I2SRXLT1 )
#define I2S_I2SRXRT1        ( I2S1_I2SRXRT1 )
#elif defined USE_I2S2_REC
#define PSP_I2S_RX_INST_ID  ( PSP_I2S_2 )
#define I2S_RX_EVENT        ( RCV2_EVENT )
#define I2S_I2SRXLT1        ( I2S2_I2SRXLT1 )
#define I2S_I2SRXRT1        ( I2S2_I2SRXRT1 )
#elif defined USE_I2S3_REC
#define PSP_I2S_RX_INST_ID  ( PSP_I2S_3 )
#define I2S_RX_EVENT        ( RCV3_EVENT )
#define I2S_I2SRXLT1        ( I2S3_I2SRXLT1 )
#define I2S_I2SRXRT1        ( I2S3_I2SRXRT1 )
#endif

/* Status return codes */
#define I2SSAMPLE_SOK               ( 0 )
#define I2SSAMPLE_INV_PRMS          ( 1 )
#define I2SSAMPLE_I2SINIT_PB_FAIL   ( 2 )
#define I2SSAMPLE_I2SINIT_REC_FAIL  ( 3 )
#define I2SSAMPLE_DMAINIT_PB_FAIL   ( 4 )
#define I2SSAMPLE_DMAINIT_REC_FAIL  ( 5 )

/* I2S initialization parameters */
typedef struct I2sInitPrms 
{
    Bool enablePlayback;        /* enable/disable playback */
    Bool enableStereoPb;        /* enable/disable stereo playback */
    Bool sampleBySamplePb;      /* enable/disable DMA for playback */
    Bool enableDmaPingPongPb;   /* enable/disable DMA hardware ping/pong for Playback */
    Int16 *pingI2sTxLeftBuf;    /* Left ch. Tx ping buffer for DMA ping/pong hardware disabled, 
                                   Left ch. Tx ping/pong buffer for DMA ping/pong hardware enabled */
    Int16 *pongI2sTxLeftBuf;    /* Left ch. Tx pong buffer for DMA ping/pong hardware disabled */
    Int16 *pingI2sTxRightBuf;   /* Right ch. Tx ping buffer for DMA ping/pong hardware disabled, 
                                   Right ch. Tx ping/pong buffer for DMA ping/pong hardware enabled */
    Int16 *pongI2sTxRightBuf;   /* Right ch. Tx pong buffer for DMA ping/pong hardware disabled */
    PSP_I2SInstanceId i2sPb;    /* I2S for playback */
    Bool enableRecord;          /* enable/disable record */
    Bool enableStereoRec;       /* enable/disable stereo record */
    Bool sampleBySampleRec;     /* enable/disable DMA for record */
    Bool enableDmaPingPongRec;  /* enable/disable DMA hardware ping/pong for Record */
    Int16 *pingI2sRxLeftBuf;    /* Left ch. Rx ping buffer for DMA ping/pong hardware disabled, 
                                   Left ch. Rx ping/pong buffer for DMA ping/pong hardware enabled */
    Int16 *pongI2sRxLeftBuf;    /* Left ch. Rx pong buffer for DMA ping/pong hardware disabled */
    Int16 *pingI2sRxRightBuf;   /* Right ch. Rx ping buffer for DMA ping/pong hardware disabled, 
                                   Right ch. Rx ping/pong buffer for DMA ping/pong hardware enabled */
    Int16 *pongI2sRxRightBuf;   /* Right ch. Rx pong buffer for DMA ping/pong hardware disabled */
    PSP_I2SInstanceId i2sRec;   /* I2S for record */
    Int16 *zeroBuf;
} I2sInitPrms;

/* I2S handles */
extern PSP_Handle i2sHandleTx;
extern PSP_Handle i2sHandleRx;

/* Codec Rx ping/pong buffer (Left ch.) */
extern Uint16 ping_pong_i2sRxLeftBuf[2*I2S_RXBUFF_SZ]; /* 2x for ping/pong */
extern Int16 left_rx_buf_sel;
/* Codec Rx ping/pong buffer (Right ch.) */
extern Uint16 ping_pong_i2sRxRightBuf[2*I2S_RXBUFF_SZ]; /* 2x for ping/pong */
extern Int16 right_rx_buf_sel;

/**< DMA handle for left receive channel   */
extern DMA_ChanHandle   hDmaRxLeft;
/**< DMA handle for right receive channel  */
extern DMA_ChanHandle   hDmaRxRight;

/* Codec input circular buffer */
extern Uint16 codec_input_buffer[];
extern Uint16 codec_input_buffer_input_index;
extern Uint16 codec_input_buffer_output_index;
extern Uint32 codec_input_buffer_underflow;
extern Uint32 codec_input_buffer_overflow;

/* Ping/pong buffers for codec Tx */
extern Int16 ping_pong_i2sTxBuf[2*MAX_I2S_TXBUFF_SZ]; /* 2x for ping/pong */
/* Pointer to current ping/pong buffer */
extern Int16 *codec_output_buffer;
/* Ping/pong buffer selection variable */
extern Int16 tx_buf_sel;
/* Current output buffer sample count */
extern Uint16 codec_output_buf_samp_cnt;
/* Indicates whether codec output buffer available */
extern Bool codec_output_buffer_avail;
extern Uint16 codec_output_buffer_out_error;
extern Uint16 i2sTxBuffSz; /* run-time size of Tx ping/pong buffer */

#if defined(COUNT_REC_SAMPS_IN_USB_FRAME_REC) || defined(COUNT_REC_SAMPS_IN_USB_FRAME_PB)
extern Uint16 i2sRxSampCnt;
#endif

void DDC_I2S_transEnable(DDC_I2SHandle    hI2s, Uint16 enableBit);

void i2sTxStart(void);

/* I2S Tx ISR */
void i2s_txIsr(void); /* SampleBySample */

/* I2S Rx ISR */
void i2s_rxIsr(void); /* SampleBySample */

/* Initializes I2S and associated DMA channels for Playback and Record */
Int16 i2sInit(
    I2sInitPrms *pI2sInitPrms
);

/* Resets codec output buffer */
void reset_codec_output_buffer(void);

#endif /* _I2S_SAMPLE_H_ */
