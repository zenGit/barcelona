/*
 * $$$MODULE_NAME app_usbac.c
 *
 * $$$MODULE_DESC app_usbac.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#include <std.h>
#include <string.h>
#include "csl_audioClass.h"
#include "csl_audioClassAux.h"
#include "app_globals.h"
#include "app_asrc.h"
#include "app_usb.h"
#include "app_usbac_descs.h"
#include "i2s_sample.h"
#include "app_usbac.h"
#include "VC5505_CSL_BIOS_cfg.h"

/* Define to send known data on record path to USB */
#define SEND_KNOWN_DATA_TO_USB // debug

extern CSL_UsbContext  gUsbContext;

CSL_AcInitStructApp AC_AppHandle;

Uint16 active_sample_rate;
Uint16 rx_pkt_size_16K_playback;

Uint16 pId = DEV_PID;
Uint16 vId = DEV_VID;

Uint32 usb_error = 0; // debug

/* Allocating Memory for Use by the Module */
#pragma DATA_ALIGN(ACAppBuffer, 4);
Uint32 ACAppBuffer[APP_USBAC_AC_APP_BUF_SIZE];

/* Buffer used to store the playback data sent from USB */
#pragma DATA_ALIGN(lbaBufferPbApp, 4);
Uint16 lbaBufferPbApp[APP_USBAC_LBA_BUF_PB_SIZE];

/* Buffer used to store the record data to send to USB */
#pragma DATA_ALIGN(lbaBufferRecApp, 4);
Uint16 lbaBufferRecApp[APP_USBAC_LBA_BUF_REC_SIZE];

/* Buffer used to store the HID Report data to send to USB */
#pragma DATA_ALIGN(lbaBufferHidReportApp, 4);
Uint16 lbaBufferHidReportApp[HID_REPORT_SIZE_WORDS+1];

CSL_AcRequestStruct USB_ReqTable[] =
{
  { CSL_AC_REQUEST_GET_STATUS             , AC_reqGetStatus },
  { CSL_AC_REQUEST_CLEAR_FEATURE          , AC_reqClearFeature },
  { CSL_AC_REQUEST_SET_FEATURE            , AC_reqSetFeature },
  { CSL_AC_REQUEST_SET_ADDRESS            , AC_reqSetAddress },
  { CSL_AC_REQUEST_GET_DESCRIPTOR         , AC_reqGetDescriptor },
  { CSL_AC_REQUEST_SET_DESCRIPTOR         , AC_reqUnknown },
  { CSL_AC_REQUEST_GET_CONFIGURATION      , AC_reqGetConfiguration },
  { CSL_AC_REQUEST_SET_CONFIGURATION      , AC_reqSetConfiguration },
  { CSL_AC_REQUEST_GET_MAX_LUN            , AC_reqGetMaxLUN },
  { CSL_AC_REQUEST_GET_INTERFACE          , AC_reqGetInterface },
  { CSL_AC_REQUEST_SET_INTERFACE          , AC_reqSetInterface },
  { CSL_AC_REQUEST_SYNC_FRAME             , AC_reqUnknown },
  { CSL_AC_REQUEST_SET_CUR                , AC_reqSetCurrent },
  { CSL_AC_REQUEST_GET_CUR                , AC_reqGetCurrent },
  { CSL_AC_REQUEST_SET_MIN                , AC_reqUnknown },
  { CSL_AC_REQUEST_GET_MIN                , AC_reqGetMinimum },
  { CSL_AC_REQUEST_GET_MAX                , AC_reqGetMaximum },
  { CSL_AC_REQUEST_SET_MAX                , AC_reqUnknown },
  { CSL_AC_REQUEST_GET_RES                , AC_reqGetResolution },
  { CSL_AC_REQUEST_SET_RES                , AC_reqUnknown },
  { CSL_AC_REQUEST_HID_GET_REPORT         , AC_reqHidGetReport },
  { CSL_AC_REQUEST_HID_GET_IDLE           , AC_reqHidGetIdle },
  { CSL_AC_REQUEST_HID_GET_PROTOCOL       , AC_reqUnknown },
  { CSL_AC_REQUEST_HID_SET_REPORT         , AC_reqUnknown },
  { CSL_AC_REQUEST_HID_SET_IDLE           , AC_reqHidSetIdle },
  { CSL_AC_REQUEST_HID_SET_PROTOCOL       , AC_reqUnknown },
  { 0, NULL }
};

#ifdef SEND_KNOWN_DATA_TO_USB // debug
#define DATA_TO_USB_SIZE ( 32 )
Int16 data_to_usb_idx = 8;
const Int16 data_to_usb[DATA_TO_USB_SIZE] = {
    -32760, -28664, -24568, -20472, -16376, -12280,  -8184,  -4088,      8,   4104,   8200,  12296,  16392,  20488,  24584,  28680, 
    -32760, -28664, -24568, -20472, -16376, -12280,  -8184,  -4088,      8,   4104,   8200,  12296,  16392,  20488,  24584,  28680
};
#endif // SEND_KNOWN_DATA_TO_USB

static void MSCTask(void);

/**
 *  \brief  Function to play the audio
 *
 *  \param  dataLength    - Length of the data to be sent to audio device (In words)
 *  \param  leftDataBuf   - Left data buffer pointer
 *  \param  rightDataBuf  - Right data buffer pointer
 *  \param  ptr           - Pointer to hold the palyback status
 *
 *  \return Media status
 */
CSL_AcMediaStatus appPlayAudio(
    Uint16    dataLength,
    Uint16    *leftDataBuf,
    Uint16    *rightDataBuf,
    void      *ptr
)
{
    Uint16    *playBackActive;
    Int16     asrcOutputFifoNumBlk;
#ifdef ENABLE_ASRC
    Uint16    numOutSamps;
    Int16     status;
#else
    Uint16    looper;
    Uint16    tmpIdx;
#endif

#ifdef C5535_EZDSP_DEMO
	int i, numSample;
	int *samplePtr;
	extern int bufferIn[1024];
	extern int bufferInIdx;
	// get the number of samples
	numSample = dataLength/2;
	samplePtr = (int*)leftDataBuf;
#endif

    // NOTE: leftDataBuf and rightDataBuf point to the same buffer

    playBackActive = (Uint16*)ptr;
    *playBackActive = TRUE;

    // the first byte is the number of bytes in the buffer followed by the first left audio sample
    leftDataBuf++;

#ifdef C5535_EZDSP_DEMO
	samplePtr++;
	for (i=0; i<numSample; i++)
	{
		if (bufferInIdx<256)
		{
			/* Take average of left and right channels. */
			//temp = (signed long)samplePtr[i*2] + (signed long)samplePtr[i*2+1];
			//temp >>= 1;    /* Divide by 2 to prevent overload at output */
			//bufferIn[bufferInIdx] = (int)temp;
			// copy the audio sample from the USB buffer into bufferIn (left channel only)
			bufferIn[bufferInIdx] = (int)samplePtr[i*2]*2;
			bufferInIdx++;
			// if the bufferIn is filled, then send a semaphore to the SpectrumDisplayTask
			if (bufferInIdx==256)
			{
				// send semaphore
				SEM_post(&SEM_BufferInReady);
			}
		}
	}
#endif

#ifdef ENABLE_STEREO_PLAYBACK
    // stereo USB audio - the audio data is L R L R ........

#ifdef ENABLE_ASRC
    //STS_add(&mySts2, hAsrc->numOutSamps); // debug

    /* Update ASRC phase increment */
    status = ASRC_updatePhaseIncr(hAsrc);
    //STS_add(&mySts2, hAsrc->deltaPhaseIncr);
    if (status != ASRC_SOK)
    {
        usb_error++;
        LOG_printf(&trace, "ERROR: ASRC_updatePhaseIncr() failed: %d\n", status);
    }

    /* Update ASRC input FIFO */
    status = ASRC_updateInFifo(hAsrc, (Int16 *)leftDataBuf, dataLength/2);
    if (status != ASRC_SOK)
    {
        usb_error++;
        LOG_printf(&trace, "ERROR: ASRC_updateInFifo() failed: %d\n", status);
    }

    /* Get ASRC number of output samples to generate */
    status = ASRC_getNumOutSamps(hAsrc, &numOutSamps);
    if (status != ASRC_SOK)
    {
        usb_error++;
        LOG_printf(&trace, "ERROR: ASRC_getNumOutSamps() failed.");
    }

    //HWI_disable();
    /* Check output FIFO overflow -- require at least one free buffer in output FIFO */
    asrcOutputFifoNumBlk = asrcOutputFifoInBlk - asrcOutputFifoOutBlk + 1;
    //HWI_enable();
    if (asrcOutputFifoNumBlk < 0)
    {
        asrcOutputFifoNumBlk += ASRC_OUTPUT_FIFO_NUM_BLK;
    }
    //STS_add(&mySts2, asrcOutputFifoNumBlk); // debug
    //LOG_printf(&trace, "%d", asrcOutputFifoNumBlk); // debug
    if (asrcOutputFifoNumBlk > (ASRC_OUTPUT_FIFO_NUM_BLK - 2))
    {
        asrcOutputFifoInError++;
        LOG_printf(&trace, "ERROR: ASRC output FIFO OVERFLOW");
        //LOG_printf(&trace, "%04x %d", (asrcOutputFifoInBlk<<8)|asrcOutputFifoOutBlk, asrcOutputFifoNumBlk);
    }

    /* Generate ASRC output samples */
    status = ASRC_genOutInterpCoefs16b(hAsrc, &asrcOutputFifo[asrcOutputFifoInBlk][0], numOutSamps);
    if (status != ASRC_SOK)
    {
        usb_error++;
        LOG_printf(&trace, "ERROR: ASRC_genOutInterpCoefs16b() failed.");
    }

    /* Update ASRC output FIFO */
    if (rdy_to_consume_asrc_output == TRUE)
    {
        //HWI_disable();
        asrcOutputFifoBlkNumSamps[asrcOutputFifoInBlk] = 2 * numOutSamps; // outFifoNumCh * numOutSamps;
        //HWI_enable();
        asrcOutputFifoInBlk = (asrcOutputFifoInBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
    }

    if (usb_play_mode == FALSE)
    {
        usb_play_mode = TRUE;
    }

#else /* ENABLE_ASRC */

    /* Copy audio samples to circular buffer */
    for (looper = 0; looper < (dataLength/2); looper++)
    {
        /* Check for overflow */
        tmpIdx = no_asrc_buffer_input_index + 1;
        if (tmpIdx >= CODEC_OUTPUT_BUFFER_SIZE)
        {
            tmpIdx = 0;
        }
        if(tmpIdx == no_asrc_buffer_output_index)
        {
            no_asrc_buffer_in_error++;
            LOG_printf(&trace, "ERROR: no_asrc_buffer OVERFLOW");
        }

        /* Copy left */
        no_asrc_buffer[no_asrc_buffer_input_index++] = *leftDataBuf++;
        /* Copy right */
        no_asrc_buffer[no_asrc_buffer_input_index++] = *leftDataBuf++;

        if (no_asrc_buffer_input_index >= CODEC_OUTPUT_BUFFER_SIZE)
        {
            no_asrc_buffer_inwrap++;
            no_asrc_buffer_input_index = 0;
        }
    }

    //HWI_disable();
    /* Check output FIFO overflow -- require at least one free buffer in output FIFO */
    asrcOutputFifoNumBlk = asrcOutputFifoInBlk - asrcOutputFifoOutBlk + 1;
    //HWI_enable();
    if (asrcOutputFifoNumBlk < 0)
    {
        asrcOutputFifoNumBlk += ASRC_OUTPUT_FIFO_NUM_BLK;
    }
    if (asrcOutputFifoNumBlk > (ASRC_OUTPUT_FIFO_NUM_BLK - 2))
    {
        asrcOutputFifoInError++;
        LOG_printf(&trace, "ERROR: ASRC output FIFO OVERFLOW");
    }

    /* Copy audio samples to codec output FIFO */
    for (looper = 0; looper < (dataLength/2); looper++)
    {
        /* Check for underflow */
        if (no_asrc_buffer_output_index == no_asrc_buffer_input_index)
        {
            no_asrc_buffer_out_error++;
            LOG_printf(&trace, "ERROR: no_asrc_buffer UNDERFLOW");
        }

        asrcOutputFifo[asrcOutputFifoInBlk][2*looper] = no_asrc_buffer[no_asrc_buffer_output_index++];
        asrcOutputFifo[asrcOutputFifoInBlk][2*looper+1] = no_asrc_buffer[no_asrc_buffer_output_index++];
        if(no_asrc_buffer_output_index >= CODEC_OUTPUT_BUFFER_SIZE)
        {
            no_asrc_buffer_outwrap++;
            no_asrc_buffer_output_index = 0;
        }
    }

    if (rdy_to_consume_asrc_output == TRUE)
    {
        /* Update codec output FIFO */
        //HWI_disable();
        asrcOutputFifoBlkNumSamps[asrcOutputFifoInBlk] = dataLength;
        //HWI_enable();
        asrcOutputFifoInBlk = (asrcOutputFifoInBlk + 1) & (ASRC_OUTPUT_FIFO_NUM_BLK - 1);
    }

    if (usb_play_mode == FALSE)
    {
        usb_play_mode = TRUE;
    }
#endif /* ENABLE_ASRC */

#else //ENABLE_STEREO_PLAYBACK
    LOG_printf(&trace, "ERROR: assumed stereo playback, but ENABLE_STEREO_PLAYBACK is not defined");
#endif // ENABLE_STEREO_PLAYBACK

    return(CSL_AC_MEDIACCESS_SUCCESS);
}

/**
 *  \brief  Function to record the audio
 *
 *  \param  dataLength    - Length of the data to be received audio device
 *  \param  leftDataBuf   - Left data buffer pointer
 *  \param  rightDataBuf  - Right data buffer pointer
 *  \param  ptr           - Pointer for future use
 *
 *  \return Media status
 */
CSL_AcMediaStatus appRecordAudio(
    Uint16    dataLength,
    Uint16    *leftDataBuf,
    Uint16    *rightDataBuf,
    void      *ptr
)
{
#ifdef ENABLE_RECORD
    Int16 codec_input_sample_count;
    Uint16 i;

    //STS_add(&mySts1, dataLength); // debug

    /* Compute number of samples in circular buffer */
    codec_input_sample_count = codec_input_buffer_input_index - codec_input_buffer_output_index;
    if (codec_input_sample_count < 0)
    {
        codec_input_sample_count += CODEC_INPUT_BUFFER_SIZE;
    }

    /* First word is length in bytes */
    *leftDataBuf++ = 2*dataLength;

//#ifndef SEND_KNOWN_DATA_TO_USB
#if 0
     // check for underflow
    if (codec_input_sample_count >= dataLength)
    {
        // get the data from the circular buffer
        for(i = 0; i < dataLength; i++)
        {
            *leftDataBuf++ = codec_input_buffer[codec_input_buffer_output_index++];
            if (codec_input_buffer_output_index >= CODEC_INPUT_BUFFER_SIZE)
            {
                codec_input_buffer_output_index = 0;
            }
        }
    }
    else
    {
        //codec input underflow - send 0's
        for (i = 0; i < dataLength; i++)
        {
            *leftDataBuf++ = 0;
        }
        if (set_record_interface >= START_REC_WAIT_FRAMES)
        {
            codec_input_buffer_underflow++;
            LOG_printf(&trace, "ERROR: codec input buffer UNDERFLOW: %d\n", codec_input_sample_count);
        }
    }

#else // SEND_KNOWN_DATA_TO_USB // debug
    for(i = 0; i < dataLength; i++)
    {
        *leftDataBuf++ = data_to_usb[data_to_usb_idx] / 8;
        data_to_usb_idx = (data_to_usb_idx + 1) & (DATA_TO_USB_SIZE-1);
        
        codec_input_buffer_output_index++;
        if (codec_input_buffer_output_index >= CODEC_INPUT_BUFFER_SIZE)
        {
            codec_input_buffer_output_index = 0;
        }
    }

#endif // SEND_KNOWN_DATA_TO_USB

#endif // ENABLE_RECORD

    return(CSL_AC_MEDIACCESS_SUCCESS);
}

/**
 *  \brief  Function to initialize the audio playback
 *
 *  \param  status    - Status of audio playback
 *  \param  ptr       - Pointer for future use
 *
 *  \return Media status
 */
CSL_Status appInitPlayAudio(
    Uint16    status,
    void      *ptr
)
{
#ifndef ENABLE_ASRC
    /* Reset passthrough buffer */
    memset(no_asrc_buffer, 0, CODEC_OUTPUT_BUFFER_SIZE);
    no_asrc_buffer_input_index = MAX_I2S_TXBUFF_SZ;
    no_asrc_buffer_output_index = 0;
#endif

    /* Clear ASRC input FIFO */
    ASRC_resetInFifo(hAsrc);

    /* Reset ASRC output FIFO */
    resetAsrcOutputFifo(active_sample_rate);
    //LOG_printf(&trace, "R 0x%04X", (asrcOutputFifoInBlk<<8)|asrcOutputFifoOutBlk); // debug

    return CSL_SOK;
}

/**
 *  \brief  Function to initialize the audio record
 *
 *  \param  status    - Status of audio record
 *  \param  ptr       - Pointer for future use
 *
 *  \return Media status
 */
CSL_Status appInitRecordAudio(
    Uint16    status,
    void      *ptr
)
{
#if !defined(COUNT_REC_SAMPS_IN_USB_FRAME_REC) && !defined(COUNT_REC_SAMPS_IN_USB_FRAME_PB)
    Uint16 *pMaxPktSiz = (Uint16 *)ptr;
#endif    

    // reset codec input circular buffer
    memset(codec_input_buffer, 0, CODEC_INPUT_BUFFER_SIZE);
    codec_input_buffer_input_index = CODEC_INPUT_BUFFER_SIZE/2;
    codec_input_buffer_output_index = 0;
    codec_input_buffer_underflow = 0;
    codec_input_buffer_overflow = 0;

#if defined(COUNT_REC_SAMPS_IN_USB_FRAME_REC) || defined(COUNT_REC_SAMPS_IN_USB_FRAME_PB)
    // reset I2S Rx sample count
    i2sRxSampCnt = 0;
#else
    numRecSamps = *pMaxPktSiz/2;
#endif

    set_record_interface = 1;

    return CSL_SOK;
}

/**
 *  \brief  Function to stop the audio playback
 *
 *  \param  status    - Status of audio playback
 *  \param  ptr       - Pointer for future use
 *
 *  \return Media status
 */
CSL_Status appStopPlayAudio(
    Uint16    status,
    void      *ptr
)
{
    usb_play_mode = FALSE;
    rdy_to_consume_asrc_output = FALSE;

    return CSL_SOK;
}

/**
 *  \brief  Function to stop the audio record
 *
 *  \param  status    - Status of audio record
 *  \param  ptr       - Pointer for future use
 *
 *  \return Media status
 */
CSL_Status appStopRecordAudio(
    Uint16    status,
    void      *ptr
)
{
    set_record_interface = 0;
    h_usb_int_tcount = 0;

    return CSL_SOK;
}

/**
 *  \brief  Function to send HID report
 *
 *  \param  dataLength    - Length of the report data (16-bit words)
 *  \param  hidReport     - Report buffer pointer
 *
 *  \return CSL status
 */
CSL_Status appGetHidReport(
    Uint16 dataLength, // bytes 
    Uint16 *hidReport
)
{
    /* First word is length in bytes */
    *hidReport = dataLength;
    hidReport++;

    /* Get report data */
    memcpy(hidReport, gHidReport, (dataLength+1)/2);

    return CSL_SOK;
}

/**
 *  \brief  Function to get the media size
 *
 *  \param  lunNo    - Logical unit number
 *
 *  \return size of the media
 */
Uint32 AppGetMediaSize(
    Uint16 lunNo
)
{
    Uint32 size;

    size = 0;

    return(size);
}

/**
 *  \brief  Function to initialize the media
 *
 *  \param  lunNo    - Logical unit number
 *
 *  \return Media status
 */
CSL_AcMediaStatus AppMediaInit(
    Uint16 lunNo
)
{
    return CSL_AC_MEDIACCESS_SUCCESS;
}

/**
 *  \brief  Function to know the media status
 *
 *  \param  lunNo    - Logical unit number
 *
 *  \return Media status
 */
CSL_AcMediaStatus AppGetMediaStatus(
    Uint16    lunNo
)
{
    return CSL_AC_MEDIACCESS_SUCCESS;
}

/**
 *  \brief  Function to Eject media
 *
 *  \param  lunNo  - Logical unit number
 *
 *  \return Media status
 */
CSL_AcMediaStatus AppMediaEject(
    Uint16    lunNo
)
{
    return(CSL_AC_MEDIACCESS_SUCCESS);
}

/**
 *  \brief  Function to lock media
 *
 *  \param  lunNo   - Logical unit number
 *  \param  status  - Media lock status
 *
 *  \return Media status
 */
CSL_AcMediaStatus AppLockMedia(
    Uint16  lunNo, 
    CSL_AcMediaLockStatus   status
)
{
    return(CSL_AC_MEDIACCESS_SUCCESS);
}

/*
 * process the data received from the USB  - called by SWI_Process_USB_Input
 ***********************************************************************/
void process_usb_input(void)
{
    pAcClassHandle      pAcClassHdl;
    CSL_AcObject        *pAcHandle;
    pUsbEpHandle        hUsbOutEp;
    CSL_Status          status;

    pAcClassHdl = AC_AppHandle.pAcObj;
    pAcHandle   = &pAcClassHdl->acHandle;
    hUsbOutEp   = &pAcHandle->isoOutEpObj;

    if(active_sample_rate == ACTIVE_SAMPLE_RATE_16KHZ)
    {
        // playback 16KHz USB data
        status = pAcHandle->playAudio(rx_pkt_size_16K_playback/2, &pAcHandle->lbaBufferPb[0],
                                      &pAcHandle->lbaBufferPb[0], (void*)(&pAcHandle->playBackActive));
    }
    else
    {
        // playback 48KHz USB data
        status = pAcHandle->playAudio(hUsbOutEp->maxPktSiz/2, &pAcHandle->lbaBufferPb[0],
                                      &pAcHandle->lbaBufferPb[0], (void*)(&pAcHandle->playBackActive));
    }
    if(status != CSL_AC_MEDIACCESS_SUCCESS)
    {
        LOG_printf(&trace, "ERROR: process_usb_input() failed:\n");
    }
}

/**
 *  \brief  Application call back function for control transactions
 *
 *  \param  None
 *
 *  \return None
 */
void appCtrlFxn(void)
{
  AC_Ctrl(AC_AppHandle.pAcObj);
}

/**
 *  \brief  Application call back function for isochronous transactions
 *
 *  \param  None
 *
 *  \return None
 */
void appIsoFxn(void)
{
   AC_Iso(AC_AppHandle.pAcObj);
}

/**
 *  \brief  Application call back function for HID transactions
 *
 *  \param  None
 *
 *  \return None
 */
void appHidFxn(void)
{
  AC_Hid(AC_AppHandle.pAcObj);
}

/**
 *  \brief  Wraper for USB Msc Task
 *
 *  \param  None
 *
 *  \return None
 */
void USBMSCTask()
{
    /* Call the USB Mass storage class task */
    MSCTask();
}

/**
 *  \brief  USB Msc task
 *
 *  \param  None
 *
 *  \return None
 */
static void MSCTask(void)
{
    CSL_UsbMscMsg        wMSCMsg;
    pUsbContext     pContext = &gUsbContext;
    volatile WORD     Msg;
    pUsbEpStatus         peps;

    //TSK_settime(TSK_self()); // statistic collection
    while(TRUE)
    {
        /* wait for mailbox to be posted */
        MBX_pend(&MBX_msc, &wMSCMsg, SYS_FOREVER);
        Msg = wMSCMsg;
        if(Msg == CSL_USB_MSG_MSC_TASK_EXIT)
            break;

        switch(Msg)
        {
            case CSL_USB_MSG_MSC_CTL:
                peps = &pContext->pEpStatus[CSL_USB_EP0];
                if(peps->hEventHandler)
                    (*peps->hEventHandler)();
                break;

            case CSL_USB_MSG_ISO_IN:
                peps = &pContext->pEpStatus[CSL_USB_EP1];
                if(peps->hEventHandler)
                    (*peps->hEventHandler)();
                break;

            case CSL_USB_MSG_ISO_OUT:
                peps = &pContext->pEpStatus[CSL_USB_EP2];
                if(peps->hEventHandler)
                    (*peps->hEventHandler)();
                break;

            case CSL_USB_MSG_HID_INT_IN:
                peps = &pContext->pEpStatus[CSL_USB_EP3];
                if(peps->hEventHandler)
                    (*peps->hEventHandler)();
                break;

            default:
                break;
        }

        //TSK_deltatime(TSK_self()); // statistic collection
    }
    /* Ack for exit this task */
    SEM_post(&SEM_MUSBMSCTaskExited);

}

/* Intializes active sample rate and I2S Tx buffer size */
Int16 initSampleRate(
    Uint16 usbSampleRate, 
    Uint16 *pActiveSampleRate, 
    Uint16 *pI2sTxBuffSz
)
{
    Int16 status = APP_USBAC_SOK;

    switch (usbSampleRate)
    {
    case RATE_16_KHZ:
        *pActiveSampleRate = ACTIVE_SAMPLE_RATE_16KHZ;
        *pI2sTxBuffSz = 2*TXBUFF_SZ_I2SSAMPS_16KHZ; /* 2x for stereo */
        break;
    case RATE_48_KHZ:
        *pActiveSampleRate = ACTIVE_SAMPLE_RATE_48KHZ;
        *pI2sTxBuffSz = 2*TXBUFF_SZ_I2SSAMPS_48KHZ; /* 2x for stereo */
        break;
    default:
        status = APP_USBAC_INV_USBSR;
        break;
    }

    return status;
}
