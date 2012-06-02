/*
 * $$$MODULE_NAME app_asrc.h
 *
 * $$$MODULE_DESC app_asrc.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _APP_ASRC_H_
#define _APP_ASRC_H_

#include "psp_i2s.h"
#include "asrc_dly_fix.h"

//#define ENABLE_ASRC /* define to enable ASRC */

#define ASRC_OUTPUT_FIFO_NUM_BLK    ( 8 )
//#define ASRC_OUTPUT_FIFO_NUM_BLK    ( 16 )
//#define ASRC_OUTPUT_FIFO_NUM_BLK    ( 32 )

#ifdef SAMPLE_RATE_TX_48kHz
#define ASRC_OUTPUT_FIFO_BLK_SIZE   (2*48+48)
#endif

#ifdef SAMPLE_RATE_TX_44_1kHz
#define ASRC_OUTPUT_FIFO_BLK_SIZE   (2*44+44)
#endif

#ifdef SAMPLE_RATE_TX_32kHz
#define ASRC_OUTPUT_FIFO_BLK_SIZE   (2*32+32)
#endif

#ifdef SAMPLE_RATE_TX_16kHz
#define ASRC_OUTPUT_FIFO_BLK_SIZE   (2*16+16)
#endif

/* Combine ASRC return values */
#define CMBASRC_SOK                 ( 0 )   /* indicates successful return status */
#define CMBASRC_INV_PRM             ( 1 )   /* indicates invalid parameter to function */
#define CMBASRC_FIFO_UND            ( 2 )   /* indicates ASRC Output FIFO underflow */

extern ASRC_Obj asrc;       /* ASRC object */
extern ASRC_Handle hAsrc;   /* ASRC handle */
extern ASRC_Prms asrcPrms;  /* ASRC parameters object */
extern Int16 inFifo[ASRC_NUM_CH_STEREO*ASRC_DEF_IN_FIFO_SZ]; /* ASRC input FIFO */

#ifndef ENABLE_ASRC
extern Uint16 no_asrc_buffer[CODEC_OUTPUT_BUFFER_SIZE];
extern Uint16 no_asrc_buffer_input_index;
extern Uint16 no_asrc_buffer_output_index;
extern Uint16 no_asrc_buffer_in_error;
extern Uint16 no_asrc_buffer_out_error;
extern Uint16 no_asrc_buffer_outwrap;
extern Uint16 no_asrc_buffer_inwrap;
#endif

/* ASRC output FIFO */
extern Int16 asrcOutputFifo[][ASRC_OUTPUT_FIFO_BLK_SIZE];
/* Number of output samples in each output FIFO block */
extern Uint16 asrcOutputFifoBlkNumSamps[];
/* Output sample count for current output block */
extern Uint16 asrcOutputFifoOutBlkSampCnt;
/* Index of output FIFO input block */
extern Uint16 asrcOutputFifoInBlk;
/* Index of output FIFO output block */
extern Uint16 asrcOutputFifoOutBlk;
extern Uint16 asrcOutputFifoInError;
extern Uint16 asrcOutputFifoOutError;

/* Resets ASRC output FIFO */
void resetAsrcOutputFifo(
    Uint16 sampleRate
);

/*
 *initialize the sample rate converter - called at power-up and whenever sample rate changes
 ***********************************************************************/
void Init_Sample_Rate_Converter(
    Uint16 sampleRate
);

/* Combines multiple ASRC output frames into one AER input frame */
Int16 combineAsrcOutput(
    Int16   asrcOutputFifo[][ASRC_OUTPUT_FIFO_BLK_SIZE], /* output FIFO */
    Uint16  asrcOutputFifoBlkNumSamps[],    /* # samples in each output FIFO block */
    Uint16  asrcOutputFifoInBlk,            /* output FIFO input block */
    Uint16  *pAsrcOutputFifoOutBlk,         /* output FIFO output block */
    Uint16  *pAsrcOutputFifoOutBlkSampCnt,  /* output FIFO output block sample count */
    Uint16  inNumCh,                        /* # input channels */
    Int16   *outFrame,                      /* output frame */
    Uint16  outFrameNumSamps                /* # samples in output frame (one channel) */
);

#endif /* _APP_ASRC_H_ */
