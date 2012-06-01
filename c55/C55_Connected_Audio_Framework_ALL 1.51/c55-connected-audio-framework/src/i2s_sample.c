/*
 * $$$MODULE_NAME i2s_sample.c
 *
 * $$$MODULE_DESC i2s_sample.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

/**
 *  \file   i2s_sample.c
 *
 *  \brief  I2S application program
 *
 *  I2S sample test file which defines the interfaces for testing the
 *  uasability of the I2S driver.
 *
 *  (C) Copyright 2004, Texas Instruments, Inc
 *
 *  \author        PR Mistral
 *  \version    1.0   Initial implementation
 *  \version    1.1   Modified for review comments
 */

#include <string.h>
#include "psp_i2s.h"
#include "dda_i2s.h"
#include "psp_dma.h"
#include "dda_dma.h"
#include "csl_intc.h"    /* SampleBySample */
#include "app_globals.h"
#include "asrc_dly_fix.h" 
#include "app_asrc.h"
#include "app_usb.h"
#include "app_usbac.h"
#include "i2s_sample.h"
#include "gpio_control.h"

#include "VC5505_CSL_BIOS_cfg.h"
#include <clk.h>

#include "dbg_sdram.h"

#ifndef SAMPLE_BY_SAMPLE_PB

#ifdef ENABLE_STEREO_PLAYBACK
#pragma DATA_ALIGN(ping_i2sTxLeftBuf, 4);
Uint16 ping_i2sTxLeftBuf[I2S_TXBUFF_SIZE+4];
#pragma DATA_ALIGN(ping_i2sTxRightBuf, 4);
Uint16 ping_i2sTxRightBuf[I2S_TXBUFF_SIZE+4];
#pragma DATA_ALIGN(pong_i2sTxLeftBuf, 4);
Uint16 pong_i2sTxLeftBuf[I2S_TXBUFF_SIZE+4];
#pragma DATA_ALIGN(pong_i2sTxRightBuf, 4);
Uint16 pong_i2sTxRightBuf[I2S_TXBUFF_SIZE+4];

#else  // ENABLE_STEREO_PLAYBACK
#pragma DATA_ALIGN(i2sTxLeftBuf, 4);
Uint16 i2sTxLeftBuf[I2S_TXBUFF_SIZE];
#pragma DATA_ALIGN(i2sTxRightBuf, 4);
Uint16 i2sTxRightBuf[I2S_TXBUFF_SIZE];

#endif // ENABLE_STEREO_PLAYBACK

#pragma DATA_ALIGN(i2sNextTxLeftBuf, 4);
Uint16 i2sNextTxLeftBuf[I2S_RXBUFF_SZ];
#pragma DATA_ALIGN(i2sNextTxRightBuf, 4);
Uint16 i2sNextTxRightBuf[I2S_RXBUFF_SZ];

Uint16 ZeroBuf[I2S_TXBUFF_SIZE];
#endif /* SAMPLE_BY_SAMPLE_PB */

#ifdef ENABLE_RECORD
/* Codec input ping/pong buffer (Left ch.) */
#pragma DATA_ALIGN(ping_pong_i2sRxLeftBuf, 4);
Uint16 ping_pong_i2sRxLeftBuf[2*I2S_RXBUFF_SZ]; /* 2x for ping/pong */
Int16 left_rx_buf_sel = 0x0;

/* Codec input ping/pong buffer (Right ch.) */
#pragma DATA_ALIGN(ping_pong_i2sRxRightBuf, 4);
Uint16 ping_pong_i2sRxRightBuf[2*I2S_RXBUFF_SZ]; /* 2x for ping/pong */
Int16 right_rx_buf_sel = 0x0;

#endif  // ENABLE_RECORD

/* I2S handles */
PSP_Handle       i2sHandleTx;
PSP_Handle       i2sHandleRx;

Uint16    fsError1 = 0;    /**< FSYNC gobal parameter                 */
Uint16    ouError1 = 0;    /**< under/over run global parameter    */
//Uint16 outwrap = 0;

DMA_ChanHandle   hDmaTxLeft;
DMA_ChanHandle   hDmaTxRight;
DMA_ChanHandle   hDmaRxLeft;
DMA_ChanHandle   hDmaRxRight;

#if defined(COUNT_REC_SAMPS_IN_USB_FRAME_REC) || defined(COUNT_REC_SAMPS_IN_USB_FRAME_PB)
Uint16 i2sRxSampCnt; /* I2S Rx sample count */
#endif

// codec input buffer
Uint16 codec_input_buffer[CODEC_INPUT_BUFFER_SIZE];
//codec input buffer input index
Uint16 codec_input_buffer_input_index;
//codec input buffer output index
Uint16 codec_input_buffer_output_index;
Uint32 codec_input_buffer_overflow = 0;
Uint32 codec_input_buffer_underflow = 0;

/* Ping/pong buffers for codec Tx */
Int16 ping_pong_i2sTxBuf[2*MAX_I2S_TXBUFF_SZ]; /* 2x for ping/pong */
/* Pointer to current ping/pong buffer */
Int16 *codec_output_buffer;
/* Ping/pong buffer selection variable */
Int16 tx_buf_sel;
/* Current output buffer sample count */
Uint16 codec_output_buf_samp_cnt;
/* Indicates whether codec output buffer available */
Bool codec_output_buffer_avail;
Uint16 codec_output_buffer_out_error;
Uint16 i2sTxBuffSz;

extern Uint32 gi2sNextTxLeftBuf;
extern Uint32 gi2sNextTxRightBuf;
extern Uint32 gi2sNextRxLeftBuf;
extern Uint32 gi2sNextRxRightBuf;
extern Uint32 gZeroBuf;

extern Uint32    gi2sTxLeftBuf;

extern Uint32    gi2sTxRightBuf;
#ifdef ENABLE_RECORD
extern Uint32    gi2sRxLeftBuf;

extern Uint32    gi2sRxRightBuf;
#endif // ENABLE_RECORD

extern Uint32 dma_error;    /* SampleBySample */

extern Uint16 codec_input_buffer[];
extern Uint16 codec_input_buffer_input_index;
extern Uint16 codec_input_buffer_output_index;
extern Int16 codec_input_sample_count;
extern Uint32 codec_input_buffer_overflow;

/* Zeros I2S buffers for Playback */
static void zeroi2sBuf(
    Bool enableStereoPb, 
    Bool enableDmaPingPongPb, 
    Int16 *pingI2sTxLeftBuf, 
    Int16 *pingI2sTxRightBuf, 
    Int16 *pongI2sTxLeftBuf, 
    Int16 *pongI2sTxRightBuf, 
    Int16 *zeroBuf
);

#ifndef SAMPLE_BY_SAMPLE_PB
void i2sTxStart(void);
#endif

void i2sTxRxInit(void);
void i2sTxRxStart(void);

Int16 i2sPlayAudio(PSP_Handle        i2sHandle,
                   Uint32            *i2sNextTxLeftBuf,
                   Uint32            *i2sNextTxRightBuf
                   );

Int16 i2sReceiveData(PSP_Handle       i2sHandle,
                     Uint32           *i2sNextRxLeftBuf,
                     Uint32           *i2sNextRxRightBuf
                     );
Int16 i2sStopTransfer(PSP_Handle    i2sHandle);
void DDC_I2S_write(DDC_I2SHandle hI2s, Uint16 *writeBuff, Uint16 buffLen);

/* Initializes I2S and associated DMA channels for Playback and Record */
Int16 i2sInit(
    I2sInitPrms *pI2sInitPrms
)
{
    PSP_I2SOpMode opMode;
    PSP_I2SConfig i2sTxConfig;
    PSP_DMAConfig dmaTxConfig;
    PSP_I2SConfig i2sRxConfig;
    PSP_DMAConfig dmaRxConfig;

    if (pI2sInitPrms->enablePlayback == TRUE)
    {
        /* Initialize I2S instance for Playback */
        i2sTxConfig.loopBack    = PSP_I2S_LOOPBACK_DISABLE;
        i2sTxConfig.i2sMode     = PSP_I2S_SLAVE;
        i2sTxConfig.word_len    = PSP_I2S_WORDLEN_16;
        i2sTxConfig.signext     = PSP_I2S_SIGNEXT_ENABLE;
        i2sTxConfig.datatype    = PSP_I2S_MONO_DISABLE;
        i2sTxConfig.fsync_pol   = PSP_I2S_FSPOL_LOW;
        i2sTxConfig.clk_pol     = PSP_I2S_FALLING_EDGE;
        i2sTxConfig.datadelay   = PSP_I2S_DATADELAY_ONEBIT;
        i2sTxConfig.dataformat  = PSP_I2S_DATAFORMAT_LJUST;
        i2sTxConfig.fsdiv       = PSP_I2S_FSDIV32;
        i2sTxConfig.clkdiv      = PSP_I2S_CLKDIV2;

        if (pI2sInitPrms->sampleBySamplePb == TRUE)
        {
            i2sTxConfig.datapack = PSP_I2S_DATAPACK_DISABLE;
            opMode = PSP_I2S_INTERRUPT; /* change from PSP_DMA_INTERRUPT to PSP_I2S_INTERRUPT SampleBySample */
        }
        else
        {
            i2sTxConfig.datapack = PSP_I2S_DATAPACK_ENABLE;
            opMode = PSP_DMA_INTERRUPT;
        }

        i2sHandleTx = I2S_INIT(pI2sInitPrms->i2sPb, PSP_I2S_TRANSMIT, 
            PSP_I2S_CHAN_STEREO, opMode, &i2sTxConfig, NULL);
        if (i2sHandleTx == NULL)
        {
            return I2SSAMPLE_I2SINIT_PB_FAIL;
        }

        if (pI2sInitPrms->sampleBySamplePb == FALSE)
        {
            /* Initialize DMA channels for Playback */
            dmaTxConfig.pingPongMode    = pI2sInitPrms->enableDmaPingPongPb;
            dmaTxConfig.autoMode        = (pI2sInitPrms->enableDmaPingPongPb == FALSE) ? PSP_DMA_AUTORELOAD_DISABLE : PSP_DMA_AUTORELOAD_ENABLE;
            dmaTxConfig.burstLen        = PSP_DMA_TXBURST_1WORD;
            dmaTxConfig.chanDir         = PSP_DMA_WRITE;
            dmaTxConfig.trigger         = PSP_DMA_EVENT_TRIGGER;
            dmaTxConfig.trfType         = PSP_DMA_TRANSFER_IO_MEMORY;
            dmaTxConfig.dataLen         = (pI2sInitPrms->enableDmaPingPongPb == FALSE) ? 2*MAX_I2S_TXBUFF_SZ : 2*2*MAX_I2S_TXBUFF_SZ;
            dmaTxConfig.srcAddr         = (Uint32)pI2sInitPrms->pingI2sTxLeftBuf;
            //dmaTxConfig.callback        = I2S_DmaTxCallBack;

            switch (pI2sInitPrms->i2sPb)
            {
            case PSP_I2S_0: 
                dmaTxConfig.dmaEvt      =  PSP_DMA_EVT_I2S0_TX;
                dmaTxConfig.destAddr    =  (Uint32)I2S0_I2STXLT0;
                break;
            case PSP_I2S_1:
                dmaTxConfig.dmaEvt      =  PSP_DMA_EVT_I2S1_TX;
                dmaTxConfig.destAddr    =  (Uint32)I2S1_I2STXLT0;
                break;
            case PSP_I2S_2:
                dmaTxConfig.dmaEvt      =  PSP_DMA_EVT_I2S2_TX;
                dmaTxConfig.destAddr    =  (Uint32)I2S2_I2STXLT0;
                break;
            case PSP_I2S_3:
                dmaTxConfig.dmaEvt      =  PSP_DMA_EVT_I2S3_TX;
                dmaTxConfig.destAddr    =  (Uint32)I2S3_I2STXLT0;
                break;
            default:
                return I2SSAMPLE_INV_PRMS;
            }

            /* Request and configure a DMA channel for left channel data */
            hDmaTxLeft = I2S_DMA_INIT(i2sHandleTx, &dmaTxConfig);
            if (hDmaTxLeft == NULL)
            {
                return I2SSAMPLE_DMAINIT_PB_FAIL;
            }

            /* Request and configure a DMA channel for right channel data */
            dmaTxConfig.srcAddr   =  (Uint32)pI2sInitPrms->pingI2sTxRightBuf;

            switch (pI2sInitPrms->i2sPb)
            {
            case PSP_I2S_0: 
                dmaTxConfig.destAddr    =  (Uint32)I2S0_I2STXRT0;
                break;
            case PSP_I2S_1:
                dmaTxConfig.destAddr    =  (Uint32)I2S1_I2STXRT0;
                break;
            case PSP_I2S_2:
                dmaTxConfig.destAddr    =  (Uint32)I2S2_I2STXRT0;
                break;
            case PSP_I2S_3:
                dmaTxConfig.destAddr    =  (Uint32)I2S3_I2STXRT0;
                break;
            default:
                return I2SSAMPLE_INV_PRMS;
            }

            /* Request and configure a DMA channel for right data */
            hDmaTxRight  =  I2S_DMA_INIT(i2sHandleTx, &dmaTxConfig);
            if (hDmaTxRight == NULL)
            {
                return I2SSAMPLE_DMAINIT_PB_FAIL;
            }

            if (pI2sInitPrms->enableDmaPingPongPb == FALSE)
            {
                gi2sTxLeftBuf      = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pingI2sTxLeftBuf);
                gi2sTxRightBuf     = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pingI2sTxRightBuf);
                gi2sNextTxLeftBuf  = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pongI2sTxLeftBuf);
                gi2sNextTxRightBuf = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pongI2sTxRightBuf);
                gZeroBuf           = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->zeroBuf);
            }

            /* Zero buffers */
            zeroi2sBuf(pI2sInitPrms->enableStereoPb, 
                pI2sInitPrms->enableDmaPingPongPb, 
                pI2sInitPrms->pingI2sTxLeftBuf, 
                pI2sInitPrms->pingI2sTxRightBuf, 
                pI2sInitPrms->pongI2sTxLeftBuf, 
                pI2sInitPrms->pongI2sTxRightBuf, 
                pI2sInitPrms->zeroBuf);
        }
    }

    if (pI2sInitPrms->enableRecord == TRUE)
    {
        /* Initialize I2S instance for Record */
        i2sRxConfig.loopBack     = PSP_I2S_LOOPBACK_DISABLE;
        i2sRxConfig.i2sMode      = PSP_I2S_SLAVE;
        i2sRxConfig.word_len     = PSP_I2S_WORDLEN_16;
        i2sRxConfig.signext      = PSP_I2S_SIGNEXT_ENABLE;
        i2sRxConfig.datatype     = PSP_I2S_MONO_DISABLE;
        i2sRxConfig.fsync_pol    = PSP_I2S_FSPOL_LOW;
        i2sRxConfig.clk_pol      = PSP_I2S_FALLING_EDGE;
        i2sRxConfig.datadelay    = PSP_I2S_DATADELAY_ONEBIT;
        i2sRxConfig.dataformat   = PSP_I2S_DATAFORMAT_LJUST;
        i2sRxConfig.fsdiv        = PSP_I2S_FSDIV32;
        i2sRxConfig.clkdiv       = PSP_I2S_CLKDIV2;

        /* If datapack disabled on Tx, we need it disabled on Rx 
        => we get 2 words per DMA transfer so the DMA buffers have to be twice as big */
        i2sRxConfig.datapack     =  (pI2sInitPrms->sampleBySamplePb == TRUE) ? PSP_I2S_DATAPACK_DISABLE : PSP_I2S_DATAPACK_ENABLE;

        if (pI2sInitPrms->sampleBySampleRec == TRUE)
        {
            opMode = PSP_I2S_INTERRUPT;
        }
        else
        {
            opMode = PSP_DMA_INTERRUPT;
        }

        i2sHandleRx = I2S_INIT(pI2sInitPrms->i2sRec, PSP_I2S_RECEIVE, 
            PSP_I2S_CHAN_STEREO, opMode, &i2sRxConfig, NULL);
        if (i2sHandleRx == NULL)
        {
            return I2SSAMPLE_I2SINIT_REC_FAIL;
        }

        if (pI2sInitPrms->sampleBySampleRec == FALSE)
        {
            /* Initialize DMA channels for Record */
            dmaRxConfig.pingPongMode    = pI2sInitPrms->enableDmaPingPongRec;
            dmaRxConfig.autoMode        = (pI2sInitPrms->enableDmaPingPongRec == FALSE) ? PSP_DMA_AUTORELOAD_DISABLE : PSP_DMA_AUTORELOAD_ENABLE;
            dmaRxConfig.burstLen        = PSP_DMA_TXBURST_1WORD;
            dmaRxConfig.chanDir         = PSP_DMA_READ;
            dmaRxConfig.trigger         = PSP_DMA_EVENT_TRIGGER;
            dmaRxConfig.trfType         = PSP_DMA_TRANSFER_IO_MEMORY;
            dmaRxConfig.dataLen         = (pI2sInitPrms->enableDmaPingPongPb == FALSE) ? (2*I2S_RXBUFF_SZ) : 2*(2*I2S_RXBUFF_SZ);
            dmaRxConfig.destAddr        = (Uint32)pI2sInitPrms->pingI2sRxLeftBuf;
            dmaRxConfig.callback        = I2S_DmaRxCallBack;

            switch (pI2sInitPrms->i2sRec)
            {
            case PSP_I2S_0:
                dmaRxConfig.dmaEvt      =  PSP_DMA_EVT_I2S0_RX;
                dmaRxConfig.srcAddr     =  (Uint32)I2S0_I2SRXLT0;
                break;
            case PSP_I2S_1:
                dmaRxConfig.dmaEvt      =  PSP_DMA_EVT_I2S1_RX;
                dmaRxConfig.srcAddr     =  (Uint32)I2S1_I2SRXLT0;
                break;
            case PSP_I2S_2:
                dmaRxConfig.dmaEvt      =  PSP_DMA_EVT_I2S2_RX;
                dmaRxConfig.srcAddr     =  (Uint32)I2S2_I2SRXLT0;
                break;
            case PSP_I2S_3:
                dmaRxConfig.dmaEvt      =  PSP_DMA_EVT_I2S3_RX;
                dmaRxConfig.srcAddr     =  (Uint32)I2S3_I2SRXLT0;
                break;
            default:
                return I2SSAMPLE_INV_PRMS;
            }

            /* Request and configure a DMA channel for left channel data */
            hDmaRxLeft = I2S_DMA_INIT(i2sHandleRx, &dmaRxConfig);
            if (hDmaRxLeft == NULL)
            {
                return I2SSAMPLE_DMAINIT_REC_FAIL;
            }

            /* Request and configure a DMA channel for right channel data */
            switch (pI2sInitPrms->i2sPb)
            {
            case PSP_I2S_0: 
                dmaRxConfig.srcAddr   =  (Uint32)I2S0_I2SRXRT0;
                break;
            case PSP_I2S_1: 
                dmaRxConfig.srcAddr   =  (Uint32)I2S1_I2SRXRT0;
                break;
            case PSP_I2S_2: 
                dmaRxConfig.srcAddr   =  (Uint32)I2S2_I2SRXRT0;
                break;
            case PSP_I2S_3: 
                dmaRxConfig.srcAddr   =  (Uint32)I2S3_I2SRXRT0;
                break;
            default:
                return I2SSAMPLE_INV_PRMS;
            }

            dmaRxConfig.destAddr  =  (Uint32)pI2sInitPrms->pingI2sRxRightBuf;
            dmaRxConfig.callback = I2S_DmaRxCallBack; /* NOTE: currently stereo record not handled in this fxn */

            /* Request and configure a DMA channel for right data */
            hDmaRxRight = I2S_DMA_INIT(i2sHandleRx, &dmaRxConfig);
            if (hDmaRxRight == NULL)
            {
                return I2SSAMPLE_DMAINIT_REC_FAIL;
            }

            if (pI2sInitPrms->enableDmaPingPongPb == FALSE)
            {
                gi2sRxLeftBuf      = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pingI2sRxLeftBuf);
                gi2sRxRightBuf     = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pingI2sRxRightBuf);
                gi2sNextRxLeftBuf  = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pongI2sRxLeftBuf);
                gi2sNextRxRightBuf = DDC_DMAAdjustAddrOffset((Uint32)pI2sInitPrms->pongI2sRxRightBuf);
            }
        }
    }

    return I2SSAMPLE_SOK;
}

/* Zeros I2S buffers for Playback */
void zeroi2sBuf(
    Bool enableStereoPb, 
    Bool enableDmaPingPongPb, 
    Int16 *pingI2sTxLeftBuf, 
    Int16 *pingI2sTxRightBuf, 
    Int16 *pongI2sTxLeftBuf, 
    Int16 *pongI2sTxRightBuf, 
    Int16 *zeroBuf
)
{
    Uint16 i;

    if (enableStereoPb == FALSE)
    {
        if (enableDmaPingPongPb == FALSE)
        {
            for (i = 0; i < MAX_I2S_TXBUFF_SZ; i++)
            {
                pingI2sTxLeftBuf[i] = 0;
                pongI2sTxLeftBuf[i] = 0;
                zeroBuf[i] = 0;
            }
        }
        else
        {
            for (i = 0; i < 2*MAX_I2S_TXBUFF_SZ; i++)
            {
                pingI2sTxLeftBuf[i] = 0;
            }
        }
    }
    else
    {
        if (enableDmaPingPongPb == FALSE)
        {
            for (i = 0; i < MAX_I2S_TXBUFF_SZ; i++)
            {
                pingI2sTxLeftBuf[i] = 0;
                pongI2sTxLeftBuf[i] = 0;
                pingI2sTxRightBuf[i] = 0;
                pongI2sTxRightBuf[i] = 0;
                zeroBuf[i] = 0;
            }
        }
        else
        {
            for (i = 0; i < 2*MAX_I2S_TXBUFF_SZ; i++)
            {
                pingI2sTxLeftBuf[i] = 0;
                pingI2sTxRightBuf[i] = 0;
            }
        }
    }
}

#ifndef SAMPLE_BY_SAMPLE_PB
void i2sTxStart(void)
{
    Uint16  status;

    status  =  i2sPlayAudio(i2sHandleTx, (Uint32*)i2sNextTxLeftBuf,
                            (Uint32*)i2sNextTxRightBuf);
    if(status != 0)
    {
        LOG_printf(&trace, "I2S Tx Failed\n");
    }
}
#endif

/* Function to play an audio on I2S */
Int16 i2sPlayAudio(PSP_Handle        i2sHandle,
                   Uint32            *i2sNextTxLeftBuf,
                   Uint32            *i2sNextTxRightBuf
                  )
{
    Int16 status = PSP_E_INVAL_PARAM;

    if((i2sHandle != NULL)        &&
       (i2sNextTxLeftBuf != NULL) &&
       (i2sNextTxRightBuf != NULL))
    {
        status = I2S_TransmitData(i2sHandle, i2sNextTxLeftBuf, i2sNextTxRightBuf,
                                  hDmaTxLeft, hDmaTxRight);
    }

    return status;
}

/* Stops the I2S data transfer */
Int16 i2sStopTransfer(PSP_Handle    i2sHandle)
{
    Int16 status;

    if(i2sHandle != NULL)
    {
        /* Release the DMA channels */
        status   =  I2S_DMA_Deinit(hDmaTxLeft);
        status  |=  I2S_DMA_Deinit(hDmaTxRight);

        #ifdef ENABLE_RECORD
    //    status   =  I2S_DMA_Deinit(hDmaRxLeft);
    //    status  |=  I2S_DMA_Deinit(hDmaRxRight);
        #endif // ENABLE_RECORD

        /* Deinitialize the I2S instance */
    //    status  |=  I2S_DeInit(i2sHandle);
    }

    return status;
}


#if defined(SAMPLE_RATE_TX_48kHz) && defined(SAMPLE_RATE_RX_16kHz)

#ifndef COMBINE_I2S_TX_RX_ISR

/*
 * interrupt routine to test output to I2S on a SampleBySample basis
 ***********************************************************************/
void i2s_txIsr(void) /* dispatcher used */
//interrupt void i2s_txIsr(void) /* no dispatcher used */
{ /* SampleBySample */
#ifdef SAMPLE_BY_SAMPLE_PB
    Int16     status;
    static Uint16 sample_cnt = 0;

    /* Get sample from output FIFO, place in I2S output */
    *(ioport volatile unsigned *)I2S_I2STXLT1 = codec_output_buffer[codec_output_buf_samp_cnt];
    *(ioport volatile unsigned *)I2S_I2STXRT1 = codec_output_buffer[codec_output_buf_samp_cnt+1];

    /* Instead of changing codec sample rate to 16KHz, output same sample 3 times (when codec is set to 48KHz) */
    if ((active_sample_rate == ACTIVE_SAMPLE_RATE_48KHZ) || (sample_cnt >= 2))
    {
        if (usb_play_mode == TRUE)
        {
            /* Accumulate phase */
            status = ASRC_accPhase(hAsrc);
            if (status != ASRC_SOK)
            {
                dma_error++;
                LOG_printf(&trace, "ERROR: ASRC_accPhase() failed: %d\n", status);
                //while(1);
            }
        }

        codec_output_buf_samp_cnt += 2;

        /* Check if moving to next output block in FIFO */
        if (codec_output_buf_samp_cnt >= i2sTxBuffSz)
        {
            if (codec_output_buffer_avail == FALSE) /* check output FIFO underflow */
            {
                codec_output_buffer_out_error++;
                LOG_printf(&trace, "ERROR: codec output FIFO UNDERFLOW");
                //while(1);
            }

            tx_buf_sel ^= 0x1;
            codec_output_buffer = ping_pong_i2sTxBuf + tx_buf_sel*i2sTxBuffSz;
            codec_output_buf_samp_cnt = 0;
            codec_output_buffer_avail = FALSE;

            SEM_post(&SEM_PbAudioAlg);
        }
    }

    if ((set_record_interface >= 2) && (sample_cnt >= 2))
    {
#ifdef COUNT_REC_SAMPS_IN_USB_FRAME_PB
        i2sRxSampCnt++;
#endif
    }

    if (++sample_cnt >= 3)
    {
        sample_cnt = 0;
    }

#endif /* SAMPLE_BY_SAMPLE_PB */
}

/*
 * interrupt routine to test input from I2S on a SampleBySample basis
 ***********************************************************************/
//void i2s_rxIsr(void) /* dispatcher used */
interrupt void i2s_rxIsr(void) /* no dispatcher used */
{ /* SampleBySample */
#ifdef SAMPLE_BY_SAMPLE_REC
    static Uint16 sample_cnt = 0;

    //if (set_record_interface >= START_REC_WAIT_NUM_FRAMES)
    if (h_usb_int_tcount > 1) /* wait for IN tokens from Host */
    {
        /* Get sample from I2S input, place in input circular buffer */
        codec_input_buffer[codec_input_buffer_input_index] = *(ioport volatile unsigned *)I2S2_I2SRXLT1;

        // instead of changing codec sample rate to 16KHz, get same input sample 3 times (when codec is set to 48KHz)
        if (sample_cnt >= 2)
        {
            codec_input_buffer_input_index++;
            if (codec_input_buffer_input_index >= CODEC_INPUT_BUFFER_SIZE)
            {
                codec_input_buffer_input_index = 0;
            }

            codec_input_sample_count++;

            // check for overflow
            if (codec_input_sample_count > CODEC_INPUT_BUFFER_SIZE)
            {
                codec_input_buffer_overflow++;
                //LOG_printf(&trace, "ERROR: codec_input_buffer_overflow: %d", codec_input_buffer_overflow);
                LOG_printf(&trace, "ERROR: codec input buffer OVERFLOW: %d", codec_input_sample_count);
            }
        }
    }

    if ((set_record_interface >= 2) && (++sample_cnt >= 3))
    {
#ifdef COUNT_REC_SAMPS_IN_USB_FRAME_REC
        i2sRxSampCnt++;
#endif
        sample_cnt = 0;
    }

#endif /* SAMPLE_BY_SAMPLE_REC */
}

#else /* COMBINE_I2S_TX_RX_ISR */

/*
 * interrupt routine to test output to I2S on a SampleBySample basis
 ***********************************************************************/
//void i2s_txIsr(void) /* dispatcher used */
interrupt void i2s_txIsr(void) /* no dispatcher used */
{ /* SampleBySample */

    Int16     status;
    static Uint16 sample_cnt = 0;

    if (usb_play_mode == TRUE)
    {
        /* Get sample from output FIFO, place in I2S output */

        // instead of changing codec sample rate to 16KHz, output same sample 3 times(when codec is set to 48KHz)
        *(ioport volatile unsigned *)I2S_I2STXLT1 = asrcOutputFifo[asrcOutputFifoOutBlk][asrcOutputFifoOutBlkSampCnt];
        *(ioport volatile unsigned *)I2S_I2STXRT1 = asrcOutputFifo[asrcOutputFifoOutBlk][asrcOutputFifoOutBlkSampCnt+1];
        if ((active_sample_rate == ACTIVE_SAMPLE_RATE_48KHZ) || (sample_cnt >= 2))
        {
            /* Accumulate phase */
            status = ASRC_accPhase(hAsrc);
            if (status != ASRC_SOK)
            {
                dma_error++;
                LOG_printf(&trace, "ERROR: ASRC_accPhase() failed: %d\n", status);
                //while(1);
            }

            asrcOutputFifoOutBlkSampCnt += 2;

            /* Check if moving to next output block in FIFO */
            if (asrcOutputFifoOutBlkSampCnt >= asrcOutputFifoBlkNumSamps[asrcOutputFifoOutBlk])
            {
                //asrcOutputFifoBlkNumSamps[asrcOutputFifoOutBlk] = 0;
                asrcOutputFifoOutBlkSampCnt = 0;
                asrcOutputFifoOutBlk = (asrcOutputFifoOutBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
                if (asrcOutputFifoOutBlk == asrcOutputFifoInBlk) /* check output FIFO underflow */
                {
                    asrcOutputFifoOutError++;
                    LOG_printf(&trace, "ERROR: ASRC output FIFO UNDERFLOW");
                    //while(1);
                }
            }
        }
    }
    else
    {
        // not playing - output 0's
        *(ioport volatile unsigned *)I2S_I2STXLT1 = 0;
        *(ioport volatile unsigned *)I2S_I2STXRT1 = 0;
    }

    //if (set_record_interface >= START_REC_WAIT_NUM_FRAMES)
    if (h_usb_int_tcount > 1) /* wait for IN tokens from Host */
    {
        /* Get sample from I2S input, place in input circular buffer */
        codec_input_buffer[codec_input_buffer_input_index] = *(ioport volatile unsigned *)I2S_I2SRXLT1;

        // instead of changing codec sample rate to 16KHz, get same input sample 3 times (when codec is set to 48KHz)
        if (sample_cnt >= 2)
        {
            codec_input_buffer_input_index++;
            if (codec_input_buffer_input_index >= CODEC_INPUT_BUFFER_SIZE)
            {
                codec_input_buffer_input_index = 0;
            }

            codec_input_sample_count++;

            // check for overflow
            if (codec_input_sample_count > CODEC_INPUT_BUFFER_SIZE)
            {
                codec_input_buffer_overflow++;
                //LOG_printf(&trace, "ERROR: codec_input_buffer_overflow: %d", codec_input_buffer_overflow);
                LOG_printf(&trace, "ERROR: codec input buffer OVERFLOW: %d", codec_input_sample_count);
            }
        }
    }

    if ((set_record_interface >= 2) && (sample_cnt >= 2))
    {
#ifdef COUNT_REC_SAMPS_IN_USB_FRAME_REC
        i2sRxSampCnt++;
#endif
    }

    if (++sample_cnt >= 3)
    {
        sample_cnt = 0;
    }
}

/*
 * interrupt routine to test input from I2S on a SampleBySample basis
 ***********************************************************************/
//void i2s_rxIsr(void) /* dispatcher used */
interrupt void i2s_rxIsr(void) /* no dispatcher used */
{ /* SampleBySample */
}

#endif /* COMBINE_I2S_TX_RX_ISR */


#else /* defined(SAMPLE_RATE_TX_48kHz) && defined(SAMPLE_RATE_RX_16kHz) */

/*
 * interrupt routine to test output to I2S on a SampleBySample basis
 ***********************************************************************/
//interrupt void i2s_txIsr(void) /* no dispatcher used */
void i2s_txIsr(void) /* dispatcher used */
{ /* SampleBySample */
#ifdef SAMPLE_BY_SAMPLE_PB
    Int16     status;

    /* Get sample from output FIFO, place in I2S output */
    *(ioport volatile unsigned *)I2S_I2STXLT1 = codec_output_buffer[codec_output_buf_samp_cnt];
    *(ioport volatile unsigned *)I2S_I2STXRT1 = codec_output_buffer[codec_output_buf_samp_cnt+1];

    if (usb_play_mode == TRUE)
    {
        /* Accumulate phase */
        status = ASRC_accPhase(hAsrc);
        if (status != ASRC_SOK)
        {
            dma_error++;
            LOG_printf(&trace, "ERROR: ASRC_accPhase() failed: %d\n", status);
            //while(1);
        }
    }

    codec_output_buf_samp_cnt += 2;

    /* Check if moving to next output block in FIFO */
    if (codec_output_buf_samp_cnt >= i2sTxBuffSz)
    {
        if (codec_output_buffer_avail == FALSE) /* check output FIFO underflow */
        {
            codec_output_buffer_out_error++;
            LOG_printf(&trace, "ERROR: codec output FIFO UNDERFLOW");
            //while(1);
        }

        tx_buf_sel ^= 0x1;
        codec_output_buffer = ping_pong_i2sTxBuf + tx_buf_sel*i2sTxBuffSz;
        codec_output_buf_samp_cnt = 0;
        codec_output_buffer_avail = FALSE;

        SEM_post(&SEM_PbAudioAlg);
    }

    if (set_record_interface >= 2)
    {
#ifdef COUNT_REC_SAMPS_IN_USB_FRAME_PB
        i2sRxSampCnt++;
#endif
    }
#endif /* SAMPLE_BY_SAMPLE_PB */
}


/*
 * interrupt routine to test input from I2S on a SampleBySample basis
 ***********************************************************************/
void i2s_rxIsr(void) /* dispatcher used */
//interrupt void i2s_rxIsr(void) /* no dispatcher used */
{ /* SampleBySample */
}

#endif /*  defined(SAMPLE_RATE_TX_48kHz) && defined(SAMPLE_RATE_RX_16kHz) */


/* Resets codec output buffer */
void reset_codec_output_buffer(void)
{
    memset(ping_pong_i2sTxBuf, 0, 2*MAX_I2S_TXBUFF_SZ);
    codec_output_buffer = &ping_pong_i2sTxBuf[0];
    tx_buf_sel = 0x0;
    codec_output_buf_samp_cnt = 0;
    codec_output_buffer_avail = TRUE;
    codec_output_buffer_out_error = 0;
}

/* SampleBySample
 * copy of CSL I2S_transEnable function to work with DDC_I2SObj type handle
 ***********************************************************************/
void DDC_I2S_transEnable(DDC_I2SHandle    hI2s, Uint16 enableBit)
{
    ioport    CSL_I2sDrvRegs      *localregs;

    localregs =     hI2s->regs;
    //localregs->SCRL = 0x2A00;

    if(enableBit == TRUE)
    {
        /*  Enable the transmit and receive bit */
        CSL_FINST(localregs->SCRL, I2S_I2SSCTRL_ENABLE, SET);
    }
    else
    {
        /*  Disable the transmit and receive bit */
        CSL_FINST(localregs->SCRL, I2S_I2SSCTRL_ENABLE, CLEAR);
    }
}

/* SampleBySample
 * copy of CSL I2S_write function to work with DDC_I2SObj type handle
 ***********************************************************************/
void DDC_I2S_write(DDC_I2SHandle    hI2s,
                Uint16 *writeBuff, Uint16 buffLen)
{
    ioport    CSL_I2sDrvRegs      *localregs;
    Uint16    i2sIrStatus;

    if((NULL == writeBuff) || (0 == buffLen))
    {
        return;
    }

    localregs = hI2s->regs;

        while(buffLen > 0)
        {
            /* Copy data from local buffer to transmit register  */
            localregs->TRW0M = *writeBuff++;
            if(hI2s->chanType == PSP_I2S_CHAN_STEREO) //I2S_CHAN_STEREO)
            {
                localregs->TRW1M = *writeBuff++;
                buffLen -= 1;
            }
            buffLen -= 1;
        }
        // check for errors
        i2sIrStatus = localregs->IRL;

        if(i2sIrStatus & CSL_I2S_I2SINTFL_FERRFL_MASK)
        {
            fsError1++;
        }

        if(i2sIrStatus & CSL_I2S_I2SINTFL_OUERRFL_MASK)
        {
            ouError1++;
        }
}
