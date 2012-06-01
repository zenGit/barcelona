/*
 * $$$MODULE_NAME app_asrc.c
 *
 * $$$MODULE_DESC app_asrc.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#include <std.h>
#include <string.h>
#include "data_types.h"
#include "app_globals.h"
#include "app_usbac.h"
#include "asrc_dly_fix.h"
#include "app_asrc.h"

ASRC_Obj asrc;              /* ASRC object */
ASRC_Handle hAsrc = &asrc;  /* ASRC handle */
ASRC_Prms asrcPrms;         /* ASRC parameters object */
#pragma DATA_ALIGN(inFifo, 2);
Int16 inFifo[ASRC_NUM_CH_STEREO*ASRC_DEF_IN_FIFO_SZ]; /* ASRC input FIFO */

#ifndef ENABLE_ASRC
Uint16 no_asrc_buffer[CODEC_OUTPUT_BUFFER_SIZE];
Uint16 no_asrc_buffer_input_index;
Uint16 no_asrc_buffer_output_index;
Uint16 no_asrc_buffer_in_error = 0;
Uint16 no_asrc_buffer_out_error = 0;
Uint16 no_asrc_buffer_outwrap = 0;
Uint16 no_asrc_buffer_inwrap = 0;
#endif

/* ASRC output FIFO */
Int16 asrcOutputFifo[ASRC_OUTPUT_FIFO_NUM_BLK][ASRC_OUTPUT_FIFO_BLK_SIZE];
/* Number of output samples in each output FIFO block */
Uint16 asrcOutputFifoBlkNumSamps[ASRC_OUTPUT_FIFO_NUM_BLK];
/* Output sample count for current output block */
Uint16 asrcOutputFifoOutBlkSampCnt;
/* Index of output FIFO input block */
Uint16 asrcOutputFifoInBlk;
/* Index of output FIFO output block */
Uint16 asrcOutputFifoOutBlk;
Uint16 asrcOutputFifoInError = 0;
Uint16 asrcOutputFifoOutError = 0;

extern Uint16 active_sample_rate;

/* Resets ASRC output FIFO */
void resetAsrcOutputFifo(
    Uint16 sampleRate
)
{
    memset(asrcOutputFifo, 0, ASRC_OUTPUT_FIFO_NUM_BLK * ASRC_OUTPUT_FIFO_BLK_SIZE);
    asrcOutputFifoOutBlkSampCnt = 0;
    asrcOutputFifoInBlk = ASRC_OUTPUT_FIFO_NUM_BLK/2;
    //asrcOutputFifoInBlk = 2;
    asrcOutputFifoOutBlk = 0;
    if (sampleRate == ACTIVE_SAMPLE_RATE_16KHZ)
    {
        memset(asrcOutputFifoBlkNumSamps, 2*NOM_TRANS_SZ_16KHZ, ASRC_OUTPUT_FIFO_NUM_BLK/2); /* 2x for stereo */
    }
    else
    {
        memset(asrcOutputFifoBlkNumSamps, 2*NOM_TRANS_SZ_48KHZ, ASRC_OUTPUT_FIFO_NUM_BLK/2); /* 2x for stereo */
    }
}

/*
 *initialize the sample rate converter - called at power-up and whenever sample rate changes
 ***********************************************************************/
void Init_Sample_Rate_Converter(
    Uint16 sampleRate
)
{
        Uint16 nomTransSz;

        switch (sampleRate)
        {
            case ACTIVE_SAMPLE_RATE_16KHZ:
                nomTransSz = NOM_TRANS_SZ_16KHZ;
                break;

            default:
            case ACTIVE_SAMPLE_RATE_48KHZ:
                nomTransSz = NOM_TRANS_SZ_48KHZ;
                break;
        }

        /* Initialize ASRC object */
        asrcPrms.inNumCh = ASRC_NUM_CH_STEREO;
        asrcPrms.inFmt = ASRC_IN_FMT_INTERLEAVED ;
        asrcPrms.nomTransSz = nomTransSz;
        asrcPrms.k0 = (Uint32)ASRC_DEF_K0;
        asrcPrms.k1 = (Uint32)ASRC_DEF_K1;
        //asrcPrms.nomPhaseIncr = (Int32)ASRC_DEF_NOM_PHASE_INCR;
        asrcPrms.nomPhaseIncr = (Int32)ASRC_NOM_PHASE_INCR_1;
        //asrcPrms.nomPhaseIncr = (Int32)ASRC_NOM_PHASE_INCR_1_3;
        asrcPrms.inFifoSz = ASRC_DEF_IN_FIFO_SZ;
        asrcPrms.numTapsPerPhase = ASRC_NUM_TAPS_PER_PHASE;
        asrcPrms.numPhases = ASRC_NUM_PHASES;
#if 1
        asrcPrms.coefs16b = coefs16b;
        asrcPrms.coefNorm = ASRC_COEFS_16B_NORM;
#else
        asrcPrms.coefs32b = coefs32b;
        asrcPrms.coefNorm = ASRC_COEFS_32B_NORM;
#endif
        asrcPrms.inFifo = inFifo;
        ASRC_init(hAsrc, &asrcPrms);
}

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
)
{
    Uint16 curBlk;
    Uint16 outBlkSampCnt;
    Uint16 sampCnt;
    Uint16 sampCntThresh;
    Uint16 endBlk;
    Int16 *pOutFrame;
    Uint16 addSampCnt;
    Int16 *pAsrcOutSamp;
    Uint16 curBlkNumSamps;
    Uint16 i;
    Int16 status = CMBASRC_SOK;


    outBlkSampCnt = *pAsrcOutputFifoOutBlkSampCnt;

    /* Check if required number of AER samples available in ASRC Output FIFO */
    curBlk = *pAsrcOutputFifoOutBlk;
    sampCnt = asrcOutputFifoBlkNumSamps[curBlk] - outBlkSampCnt;
    sampCntThresh = inNumCh * outFrameNumSamps;
    curBlk = (curBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
    while ((curBlk != asrcOutputFifoInBlk) && (sampCnt < sampCntThresh))
    {
        sampCnt += asrcOutputFifoBlkNumSamps[curBlk];
        curBlk = (curBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
    }

    if (sampCnt >= sampCntThresh) /* samples available */
    {
        endBlk = curBlk;
        curBlk = *pAsrcOutputFifoOutBlk;
        pOutFrame = &outFrame[0];
        addSampCnt = outFrameNumSamps;

        if (inNumCh == 2) /* stereo */
        {
            while (curBlk != endBlk)
            {
                curBlkNumSamps = (asrcOutputFifoBlkNumSamps[curBlk]-outBlkSampCnt)>>1; /* # samples for mono */
                pAsrcOutSamp = &asrcOutputFifo[curBlk][outBlkSampCnt];
                if (addSampCnt >= curBlkNumSamps) /* consume entire block */
                {
                    for (i = 0; i < curBlkNumSamps; i++)
                    {
                        *pOutFrame = *pAsrcOutSamp; /* left ch. */
                        pAsrcOutSamp++;
                        pOutFrame++;

                        *pOutFrame = *pAsrcOutSamp; /* right ch. */
                        pAsrcOutSamp++;
                        pOutFrame++;
                    }
                    addSampCnt -= curBlkNumSamps;
                    outBlkSampCnt = 0;
                }
                else /* consume partial block */
                {
                    for (i = 0; i < addSampCnt; i++)
                    {
                        *pOutFrame = *pAsrcOutSamp; /* left ch. */
                        pAsrcOutSamp++;
                        pOutFrame++;

                        *pOutFrame = *pAsrcOutSamp; /* rigth ch. */
                        pAsrcOutSamp++;
                        pOutFrame++;
                    }
                    //asrcOutputFifoBlkNumSamps[curBlk] = 2*(curBlkNumSamps - addSampCnt);
                    outBlkSampCnt = 2*addSampCnt;
                }

                curBlk = (curBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
            }

            *pAsrcOutputFifoOutBlk = (outBlkSampCnt == 0) ? endBlk : (endBlk-1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
            *pAsrcOutputFifoOutBlkSampCnt = outBlkSampCnt;
        }
        else if (inNumCh == 1) /* mono */
        {
            while (curBlk != endBlk)
            {
                curBlkNumSamps = asrcOutputFifoBlkNumSamps[curBlk]-outBlkSampCnt;
                pAsrcOutSamp = &asrcOutputFifo[curBlk][outBlkSampCnt];
                if (addSampCnt >= curBlkNumSamps) /* consume entire block */
                {
                    for (i = 0; i < curBlkNumSamps; i++)
                    {
                        *pOutFrame = *pAsrcOutSamp;
                        pAsrcOutSamp++;
                        pOutFrame++;
                    }
                    addSampCnt -= curBlkNumSamps;
                    outBlkSampCnt = 0;
                }
                else /* consume partial block */
                {
                    for (i = 0; i < addSampCnt; i++)
                    {
                        *pOutFrame = *pAsrcOutSamp;
                        pAsrcOutSamp++;
                        pOutFrame++;
                    }
                    //asrcOutputFifoBlkNumSamps[curBlk] = 2*(curBlkNumSamps - addSampCnt);
                    outBlkSampCnt = addSampCnt;
                }

                curBlk = (curBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
            }

            *pAsrcOutputFifoOutBlk = (outBlkSampCnt == 0) ? endBlk : (endBlk-1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
            *pAsrcOutputFifoOutBlkSampCnt = outBlkSampCnt;
        }
    }
    else /* not enough samples available */
    {
        status = CMBASRC_FIFO_UND;
    }

    return status;
}
