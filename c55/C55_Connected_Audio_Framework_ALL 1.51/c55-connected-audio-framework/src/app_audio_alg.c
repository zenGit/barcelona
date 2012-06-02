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

static Int16 recInBuf[RXBUFF_SZ_ADCSAMPS];
static Int16 recOutBuf[RXBUFF_SZ_ADCSAMPS];

static Int16 tempPbOutBuf[MAX_I2S_TXBUFF_SZ];

/* Perform Record (Rx) audio algorithm processing */
void RecAudioAlgTsk(void)
{
    Uint16 *ptr1_rx;
    Int16 codec_input_sample_count;
    Uint16 i;


    while (1)
    {
        SEM_pend(&SEM_RecAudioAlg, SYS_FOREVER);

        /* Get pointer to ping/pong buffer */
        ptr1_rx = &ping_pong_i2sRxLeftBuf[0];
        if (left_rx_buf_sel == 0x1) /* check ping or pong buffer */
        {
            /* this buffer has data to be processed */
            ptr1_rx += I2S_RXBUFF_SZ;
        }
        left_rx_buf_sel ^= 0x1; /* update ping/pong */

        /* Get data from ping/pong buffer */
        for (i = 0; i < RXBUFF_SZ_ADCSAMPS; i++)
        {
            // NOTE: since we need datapack to be disabled on I2S tx, we need it disabled on I2S rx therefore
            // we get 2 words per DMA transfer so the offset into DMA buffers has to be twice as big
            recInBuf[i] = *ptr1_rx;
            ptr1_rx += 2;

            #if defined(SAMPLE_RATE_RX_16kHz) && defined(SAMPLE_RATE_I2S_48kHz)
            // DMA operates at 48KHz but sample rate is
            // set to 16kHz so store every third sample
            ptr1_rx += 4;
            #endif
        }

        /*                                 */
        /* Insert Record audio algorithm here */
        /*                                 */
        memcpy(recOutBuf, recInBuf, RXBUFF_SZ_ADCSAMPS); /* dummy */

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
            if (codec_input_sample_count > (CODEC_INPUT_BUFFER_SIZE-RXBUFF_SZ_ADCSAMPS-1))
            {
                codec_input_buffer_overflow++;
                LOG_printf(&trace, "ERROR: codec input buffer OVERFLOW: %d\n", codec_input_sample_count);
            }

            for (i = 0; i < RXBUFF_SZ_ADCSAMPS; i++)
            {
                codec_input_buffer[codec_input_buffer_input_index++] = recOutBuf[i];

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
            /* Combine ASRC output */
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
                //LOG_printf(&trace, "%04x %04x", tempInBlk, asrcOutputFifoOutBlk<<8 | asrcOutputFifoOutBlkSampCnt); // debug
            }
        }
        else if (usb_play_mode == TRUE)
        {
            rdy_to_consume_asrc_output = TRUE;
            memset(tempPbOutBuf, 0, i2sTxBuffSz);
        }
        else
        {
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
