/*
 * $$$MODULE_NAME app_audio_alg.c
 *
 * $$$MODULE_DESC app_audio_alg.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#include <std.h>
#include <string.h>
#include "i2s_sample.h"
#include "app_usb.h"
#include "app_usbac.h"
#include "app_asrc.h"
#include "app_audio_alg.h"
#include "VC5505_CSL_BIOS_cfg.h"
#include "dbg_sdram.h"
#include "psp_i2s.h"

static Int16 recInLeftBuf[RXBUFF_SZ_ADCSAMPS];
//static Int16 recOutLeftBuf[RXBUFF_SZ_ADCSAMPS];

#ifdef ENABLE_STEREO_RECORD
static Int16 recInRightBuf[RXBUFF_SZ_ADCSAMPS];
//static Int16 recOutRightBuf[RXBUFF_SZ_ADCSAMPS];
#endif

static Int16 tempPbOutBuf[MAX_I2S_TXBUFF_SZ];

/* Perform Record (Rx) audio algorithm processing */
void RecAudioAlgTsk(void)
{
    Uint16 *ptrRxLeft;
    Int16 codec_input_sample_count;
    Uint16 i;
#ifdef ENABLE_STEREO_RECORD
    Uint16 *ptrRxRight;
#endif

    while (1)
    {
        SEM_pend(&SEM_DmaRxLeftComplete, SYS_FOREVER);
#ifdef ENABLE_STEREO_RECORD
        SEM_pend(&SEM_DmaRxRightComplete, SYS_FOREVER);
#endif

        /* Get pointer to ping/pong buffer */
        ptrRxLeft = &ping_pong_i2sRxLeftBuf[0];
        if (left_rx_buf_sel == 0x1) /* check ping or pong buffer */
        {
            /* this buffer has data to be processed */
            ptrRxLeft += I2S_RXBUFF_SZ;
        }
        left_rx_buf_sel ^= 0x1; /* update ping/pong */

#ifdef ENABLE_STEREO_RECORD
        /* Get pointer to right ping/pong buffer */
        ptrRxRight = &ping_pong_i2sRxRightBuf[0];
        if (right_rx_buf_sel == 0x1) /* check ping or pong buffer */
        {
            /* this buffer has data to be processed */
            ptrRxRight+= I2S_RXBUFF_SZ;
        }
        right_rx_buf_sel ^= 0x1; /* update ping/pong */
#endif

        /* Get data from ping/pong buffers */
        for (i = 0; i < RXBUFF_SZ_ADCSAMPS; i++)
        {
            // NOTE: since we need datapack to be disabled on I2S tx, we need it disabled on I2S rx therefore
            // we get 2 words per DMA transfer so the offset into DMA buffers has to be twice as big
            recInLeftBuf[i] = *ptrRxLeft;
            ptrRxLeft += 2;

            #if defined(SAMPLE_RATE_RX_16kHz) && defined(SAMPLE_RATE_I2S_48kHz)
            // DMA operates at 48KHz but sample rate is
            // set to 16kHz so store every third sample
            ptrRxLeft += 4;
            #endif

#ifdef ENABLE_STEREO_RECORD
            recInRightBuf[i] = *ptrRxRight;
            ptrRxRight += 2;

            #if defined(SAMPLE_RATE_RX_16kHz) && defined(SAMPLE_RATE_I2S_48kHz)
            // DMA operates at 48KHz but sample rate is
            // set to 16kHz so store every third sample
            ptrRxRight += 4;
            #endif
#endif
        }

        /*                                 */
        /* Insert Record audio algorithm here */
        /*                                    */
        //memcpy(recOutLeftBuf, recInLeftBuf, RXBUFF_SZ_ADCSAMPS); /* dummy */

#ifdef ENABLE_STEREO_RECORD
        //memcpy(recOutRightBuf, recInRightBuf, RXBUFF_SZ_ADCSAMPS); /* dummy */
#endif

        /* Store data in circular buffer */
        if (h_usb_int_tcount > 1) /* wait for IN tokens from Host */
        {
            /* Compute number of samples in circular buffer */
            codec_input_sample_count = codec_input_buffer_input_index - codec_input_buffer_output_index;
            if (codec_input_sample_count < 0)
            {
                codec_input_sample_count += CODEC_INPUT_BUFFER_SIZE;
            }

            /* Check for overflow */
            if (codec_input_sample_count > (CODEC_INPUT_BUFFER_SIZE-CODEC_INPUT_BUFFER_FRAME_SZ-2))
            {
                codec_input_buffer_overflow++;
                LOG_printf(&trace, "ERROR: codec input buffer OVERFLOW: %d\n", codec_input_sample_count);
            }

            for (i = 0; i < RXBUFF_SZ_ADCSAMPS; i++)
            {
                //codec_input_buffer[codec_input_buffer_input_index++] = recOutLeftBuf[i];
                codec_input_buffer[codec_input_buffer_input_index++] = 0; //recInLeftBuf[i];

#ifdef ENABLE_STEREO_RECORD
                //codec_input_buffer[codec_input_buffer_input_index++] = recOutRightBuf[i];
                codec_input_buffer[codec_input_buffer_input_index++] = recInRightBuf[i];
#endif

                if (codec_input_buffer_input_index >= CODEC_INPUT_BUFFER_SIZE)
                {
                    codec_input_buffer_input_index = 0;
                }
            }
        }
    }
}

/* Perform Playback (Tx) audio algorithm processing */
void PbAudioAlgTsk(void)
{
    Int16 status;
    Int16 *pbOutBuf;
    Uint16 tempInBlk;

    while (1)
    {
        SEM_pend(&SEM_PbAudioAlg, SYS_FOREVER);

        /* Select AER output buffer */
        HWI_disable();
        pbOutBuf = ping_pong_i2sTxBuf + (!tx_buf_sel)*i2sTxBuffSz;
        HWI_enable();

        if ((usb_play_mode == TRUE) && (rdy_to_consume_asrc_output == TRUE))
        {
            /* Combine ASRC output */
            SWI_disable();
            tempInBlk = asrcOutputFifoInBlk;
            SWI_enable();
            status = combineAsrcOutput(asrcOutputFifo, 
                asrcOutputFifoBlkNumSamps, 
                tempInBlk, 
                &asrcOutputFifoOutBlk, 
                &asrcOutputFifoOutBlkSampCnt, 
                ASRC_NUM_CH_STEREO, 
                tempPbOutBuf, 
                i2sTxBuffSz>>1);
            if (status == CMBASRC_FIFO_UND)
            {
                asrcOutputFifoOutError++;
                LOG_printf(&trace, "ERROR: ASRC output FIFO UNDERFLOW");
                //LOG_printf(&trace, "%04x %d", (asrcOutputFifoInBlk<<8) | asrcOutputFifoOutBlk, asrcOutputFifoOutBlkSampCnt); // debug
            }
        }
        else if (usb_play_mode == TRUE)
        {
            /* Output zeros */
            memset(tempPbOutBuf, 0, i2sTxBuffSz);

            rdy_to_consume_asrc_output = TRUE;
        }
        else
        {
            /* Output zeros */
            memset(tempPbOutBuf, 0, i2sTxBuffSz);
        }

        /*                                      */
        /* Insert Playback audio algorithm here */
        /*                                      */
        memcpy(pbOutBuf, tempPbOutBuf, i2sTxBuffSz); /* dummy */

        HWI_disable();
        codec_output_buffer_avail = TRUE; /* indicate buffer available to I2S Tx ISR */
        HWI_enable();
    }
}
