/*
 * $$$MODULE_NAME csl_usb_iso_fullspeed_example.c
 *
 * $$$MODULE_DESC csl_usb_iso_fullspeed_example.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

/** @file csl_usb_iso_fullspeed_example.c
 *
 *  @brief USB Audio Class functional layer full speed mode example source file
 *
 *  This example tests the operation of VC5505 usb in full speed mode.
 *  NOTE: For Testing Audio class module a macro CSL_AC_TEST needs to be defined
 *  This includes some code in csl_usbAux.h file which is essential for Audio class
 *  operation and not required for MUSB stand alone testing.
 *  define this macro in pre defined symbols in project biuld options
 *  (Defined in the current usb audio class example pjt).
 *  Semaphores and mail boxes are used in the Audio class example code as the USB operation
 *  is not possible with out OS calls. DSP BIOS version 5.32.03 is used for this purpose.
 *  Definig Start transfer and complete transfer call back functions is must
 *  and Audio class module does not work if they are not implemeted properly. A call back
 *  is sent to this functions from MUSB module.
 *
 *  NOTE: Message boxes and semaphores are reused from MSC module.
 *  Name MSC is not replaced with Auidio class at some places for quick reusability
 *
 * NOTE: THIS TEST HAS BEEN DEVELOPED TO WORK WITH CHIP VERSIONS C5505 AND
 * C5515. MAKE SURE THAT PROPER CHIP VERSION MACRO CHIP_5505/CHIP_5515 IS
 * DEFINED IN THE FILE c55xx_csl\inc\csl_general.h.
 *
 *  Path: \(CSLPATH)\example\usb\example5
 */

/* ============================================================================
 * Revision History
 * ================
 * 20-Dec-2008 Created
 * ============================================================================
 */

#include "csl_types.h"
#include "csl_error.h"
#include "csl_intc.h"
#include "csl_gpio.h"
#include "csl_usb.h"
#include "csl_audioClass.h"
#include "soc.h"
#include "dda_dma.h"
#include "i2s_sample.h"
#include "gpio_control.h"
#include "pll_control.h"
#include "app_globals.h"
#include "app_usb.h"
#include "app_usbac.h"
#include "app_usbac_descs.h"
#include "codec_aic3254.h"
#include "user_interface.h"
#include "app_asrc.h"
#include "psp_i2s.h"

#ifdef C5535_EZDSP_DEMO
#include "lcd_osd.h"
#include "dsplib.h"
#include "soc.h"
#include "cslr.h"
#include "cslr_sysctrl.h"
CSL_Status  CSL_i2cPowerTest(void);
void calculate_FFT(int *input, int size);
// buffer for data collection from USB Audio plauback
int bufferIn[256];
// index for bufferIn
int bufferInIdx = 0;
// buffer for perform FFT
#pragma DATA_ALIGN(bufferFFT, 4)
DATA bufferFFT[512];
// scarch buffer for FFT
DATA bufferScrach[512];
// display buffer for spectrum display
int display_buffer[128];
// Demo switch flag: 0 - power display, 1 - spectrum analyzer
Uint16 DemoSwitchFlag = 1;
#endif

#include "VC5505_CSL_BIOS_cfg.h"

 /* Debug: enable run-time storage of data to SDRAM */
//#define STORE_PARAMETERS_TO_SDRAM

// Clock gating for unused peripherals
void ClockGating(void);

/* Initializes application */
void CSL_acTest(void);

/* Resets C5515 */
void C5515_reset(void);

/**
 *  \brief  CSL Audio Class main function
 *
 *  \param  None
 *
 *  \return None
 */
void main(void)
{
    CSL_Status status;
    Uint32 gpioIoDir;

    // Disable trace to reduce MHz load
    //TRC_disable(TRC_GBLTARG);
    // disable the message log
    //LOG_disable(&trace);

    /* Clock gate all peripherals */
    CSL_SYSCTRL_REGS->PCGCR1 = 0x7FFF;
    CSL_SYSCTRL_REGS->PCGCR2 = 0x007F;

    /* Configure EBSR */
#if defined(USE_I2S0_PB) || defined(USE_I2S0_REC)
    /* SP0 Mode 1 (I2S0 and GP[5:4]) */
    CSL_FINST(CSL_SYSCTRL_REGS->EBSR, SYS_EBSR_SP0MODE, MODE1);
#else
    /* SP0 Mode 2 (GP[5:0]) -- GPIO02/GPIO04 for debug  */
    CSL_FINST(CSL_SYSCTRL_REGS->EBSR, SYS_EBSR_SP0MODE, MODE2);
#endif
#if defined(USE_I2S1_PB) || defined(USE_I2S1_REC)
    /* SP1 Mode 1 (I2S1 and GP[11:10]) */
    CSL_FINST(CSL_SYSCTRL_REGS->EBSR, SYS_EBSR_SP1MODE, MODE1);
#else
    /* SP1 Mode 2 (GP[11:6]) */
    CSL_FINST(CSL_SYSCTRL_REGS->EBSR, SYS_EBSR_SP1MODE, MODE2); /* need GPIO10 for AIC3204 reset */
#endif
    /* PP Mode 1 (SPI, GPIO[17:12], UART, and I2S2) -- note this allows UART */
    CSL_FINST(CSL_SYSCTRL_REGS->EBSR, SYS_EBSR_PPMODE, MODE1);
    /* Reset C5515 -- ungates all peripherals */
    C5515_reset();

    /* Initialize DSP PLL */
    status = pll_sample();
    if (status != CSL_SOK)
    {
        LOG_printf(&trace, "ERROR: Unable to initialize PLL");
    }

    /* Clear pending timer interrupts */
    CSL_SYSCTRL_REGS->TIAFR = 0x7;

    /* Initialize GPIO module */
#if !defined(USE_I2S0_PB) && !defined(USE_I2S0_REC)
    /* GPIO02 and GPIO04 for debug */
    /* GPIO10 for AIC3204 reset */
    gpioIoDir = (((Uint32)CSL_GPIO_DIR_OUTPUT)<<CSL_GPIO_PIN2) | 
        (((Uint32)CSL_GPIO_DIR_OUTPUT)<<CSL_GPIO_PIN4) |
        (((Uint32)CSL_GPIO_DIR_OUTPUT)<<CSL_GPIO_PIN10);
#else
    /* GPIO10 for AIC3204 reset */
    gpioIoDir = (((Uint32)CSL_GPIO_DIR_OUTPUT)<<CSL_GPIO_PIN10);
#endif
    status = gpioInit(gpioIoDir, 0x00000000, 0x00000000);
    if (status != GPIOCTRL_SOK)
    {
        LOG_printf(&trace, "ERROR: Unable to initialize GPIO");
    }

    /* Enable the USB LDO */
    *(volatile ioport unsigned int *)(0x7004) |= 0x0001;
}

/**
 *  \brief  Audio Class intialization function
 *
 *  \param  None
 *
 *  \return None
 */
void CSL_acTest(void)
{
    I2sInitPrms i2sInitPrms;
    CSL_UsbConfig usbConfig;
    PSP_Result result;
    Int16 status;
    HWI_Attrs attrs;


    LOG_printf(&trace, "USB ISO FULL SPEED MODE\n");

    /* Initialize audio module */
    result = AIC3254_init();
    if(result != 0)
    {
        LOG_printf(&trace, "ERROR: Unable to configure audio codec");
    }
    else
    {
#if !defined(SAMPLE_BY_SAMPLE_PB) || !defined(SAMPLE_BY_SAMPLE_REC)
        DMA_HwInit();
        DMA_DrvInit();
#endif

        /* Initialize I2S and associated DMA channels for Playback and Record */
        i2sInitPrms.enablePlayback = TRUE;
        i2sInitPrms.enableStereoPb = TRUE;
#ifdef SAMPLE_BY_SAMPLE_PB
        i2sInitPrms.sampleBySamplePb = TRUE;
#else /* Configuration untested since ASRC only works with I2S in sample-by-sample mode */
        i2sInitPrms.sampleBySamplePb = FALSE;
        i2sInitPrms.enableDmaPingPongPb = FALSE;
        i2sInitPrms.pingI2sTxLeftBuf = ping_i2sTxLeftBuf;
        i2sInitPrms.pongI2sTxLeftBuf = pong_i2sTxLeftBuf;
        i2sInitPrms.pingI2sTxRightBuf = ping_i2sTxRightBuf;
        i2sInitPrms.pongI2sTxRightBuf = pong_i2sTxRightBuf;
        i2sInitPrms.zeroBuf = ZeroBuf;
#endif
        i2sInitPrms.i2sPb = PSP_I2S_TX_INST_ID;
        i2sInitPrms.enableRecord = TRUE;
#ifdef ENABLE_STEREO_RECORD
        i2sInitPrms.enableStereoRec = TRUE;
#else
        i2sInitPrms.enableStereoRec = FALSE;
#endif
#ifdef SAMPLE_BY_SAMPLE_REC
        i2sInitPrms.sampleBySampleRec = TRUE;
#else
        i2sInitPrms.sampleBySampleRec = FALSE;
        i2sInitPrms.enableDmaPingPongRec = TRUE;
        i2sInitPrms.pingI2sRxLeftBuf = (Int16 *)ping_pong_i2sRxLeftBuf;
        i2sInitPrms.pongI2sRxLeftBuf = NULL;
        i2sInitPrms.pingI2sRxRightBuf = (Int16 *)ping_pong_i2sRxRightBuf;
        i2sInitPrms.pongI2sRxRightBuf = NULL;
#endif
        i2sInitPrms.i2sRec = PSP_I2S_RX_INST_ID;
        status = i2sInit(&i2sInitPrms);
        if (status != I2SSAMPLE_SOK)
        {
            LOG_printf(&trace, "ERROR: Unable to initialize I2S");
        }

#ifdef C5535_EZDSP_DEMO
        // initialize the OLED display        
        oled_init();
#endif
        
        /* Initialising the Pointer to the Audio Class Handle to the Buffer Allocated */
        AC_AppHandle.pAcObj = &ACAppBuffer[0];

        usbConfig.devNum                = CSL_USB0;
        usbConfig.opMode                = CSL_USB_OPMODE_POLLED;
#ifdef APP_USB_SELF_POWERED
        usbConfig.selfPowered           = TRUE;
#else
        usbConfig.selfPowered           = FALSE;
#endif
        usbConfig.maxCurrent            = APP_USB_MAX_CURRENT;
        usbConfig.appSuspendCallBack    = (CSL_USB_APP_CALLBACK)CSL_suspendCallBack;
        usbConfig.appWakeupCallBack     = (CSL_USB_APP_CALLBACK)CSL_selfWakeupCallBack;
        usbConfig.startTransferCallback  = StartTransfer;
        usbConfig.completeTransferCallback = CompleteTransfer;

        USB_init(&usbConfig);

        USB_setFullSpeedMode(0x40); /* parameter is EP0 data size in bytes */

        USB_resetDev(CSL_USB0);

        /* Calling init routine */
        /* Giving all the table hanldes and the buffers to the Audio Class module */
        AC_AppHandle.strDescrApp = (char **)string_descriptor;
        AC_AppHandle.lbaBufferPbApp = &lbaBufferPbApp[0];
        AC_AppHandle.lbaBufferRecApp = &lbaBufferRecApp[0];
        AC_AppHandle.lbaBufferHidReportApp = &lbaBufferHidReportApp[0];
        AC_AppHandle.acReqTableApp = USB_ReqTable;
        AC_AppHandle.pId = pId;
        AC_AppHandle.vId = vId;

#ifndef ENABLE_PLAYBACK_TWO_SAMPLE_RATES
        #ifdef SAMPLE_RATE_TX_48kHz
        LOG_printf(&trace, "PLAYBACK: 48KHZ ");
        #ifdef ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "STEREO\n");
        AC_AppHandle.rxPktSize = EP_PB_MAXP; // max packet size for 48K stereo
        #else // ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "MONO\n");
        AC_AppHandle.rxPktSize = 0x60; // max packet size for 48K mono
        #endif // ENABLE_STEREO_PLAYBACK
        #endif // SAMPLE_RATE_TX_48kHz

        #ifdef SAMPLE_RATE_TX_44_1kHz
        LOG_printf(&trace, "PLAYBACK: 44.1KHZ ");
        #ifdef ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "STEREO\n");
        AC_AppHandle.rxPktSize = 0xB0; // max packet size for 44.1 stereo
        #else // ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "MONO\n");
        AC_AppHandle.rxPktSize = 0x58; // max packet size for 44.1 mono
        #endif // ENABLE_STEREO_PLAYBACK
        #endif // SAMPLE_RATE_TX_44_1kHz

        #ifdef SAMPLE_RATE_TX_32kHz
        LOG_printf(&trace, "PLAYBACK: 32KHZ ");
        #ifdef ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "STEREO\n");
        AC_AppHandle.rxPktSize = 0x80; // max packet size for 32K stereo
        #else // ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "MONO\n");
        AC_AppHandle.rxPktSize = 0x40; // max packet size for 32K mono
        #endif // ENABLE_STEREO_PLAYBACK
        #endif // SAMPLE_RATE_TX_32kHz

        #ifdef SAMPLE_RATE_TX_16kHz
        LOG_printf(&trace, "PLAYBACK: 16KHZ ");
        #ifdef ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "STEREO\n");
        AC_AppHandle.rxPktSize = RX_PKT_SIZE_16K_PLAYBACK_STEREO; // max packet size for 16K stereo
        rx_pkt_size_16K_playback = RX_PKT_SIZE_16K_PLAYBACK_STEREO; // max packet size for 16K stereo
        #else // ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "MONO\n");
        AC_AppHandle.rxPktSize = RX_PKT_SIZE_16K_PLAYBACK_MONO; // max packet size for 16K mono
        rx_pkt_size_16K_playback = RX_PKT_SIZE_16K_PLAYBACK_MONO;  // max packet size for 16K mono
        #endif // ENABLE_STEREO_PLAYBACK
        #endif // SAMPLE_RATE_TX_16kHz

#else /* ENABLE_PLAYBACK_TWO_SAMPLE_RATES */
        LOG_printf(&trace, "PLAYBACK: 48KHZ ");
        #ifdef ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "STEREO\n");
        AC_AppHandle.rxPktSize = EP_PB_MAXP; // max packet size for 48K stereo
        #else // ENABLE_STEREO_PLAYBACK
        LOG_printf(&trace, "MONO\n");
        AC_AppHandle.rxPktSize = 0x60; // max packet size for 48K mono
        #endif // ENABLE_STEREO_PLAYBACK

        LOG_printf(&trace, "PLAYBACK: 16KHZ ");
        #ifdef ENABLE_STEREO_PLAYBACK
        rx_pkt_size_16K_playback = RX_PKT_SIZE_16K_PLAYBACK_STEREO; // max packet size for 16K stereo
        LOG_printf(&trace, "STEREO\n");
        #else // ENABLE_STEREO_PLAYBACK
        rx_pkt_size_16K_playback = RX_PKT_SIZE_16K_PLAYBACK_MONO;  // max packet size for 16K mono
        LOG_printf(&trace, "MONO\n");
        #endif // ENABLE_STEREO_PLAYBACK
#endif /* ENABLE_PLAYBACK_TWO_SAMPLE_RATES */

        AC_AppHandle.txPktSize = EP_REC_MAXP; // max packet size for 16K mono
        AC_AppHandle.hidTxPktSize = EP_HID_MAXP; // max packet size for HID output report

        /* All Function Handlers need to be Initialised */
        AC_AppHandle.playAudioApp = appPlayAudio;
        AC_AppHandle.recordAudioApp = appRecordAudio;
        AC_AppHandle.initPlayAudioApp = appInitPlayAudio;
        AC_AppHandle.initRecordAudioApp = appInitRecordAudio;
        AC_AppHandle.stopPlayAudioApp = appStopPlayAudio;
        AC_AppHandle.stopRecordAudioApp = appStopRecordAudio;
        AC_AppHandle.mediaGetPresentStateApp = AppGetMediaStatus;
        AC_AppHandle.mediaInitApp = AppMediaInit;
        AC_AppHandle.mediaEjectApp = AppMediaEject;
        AC_AppHandle.mediaLockUnitApp = AppLockMedia;
        AC_AppHandle.getMediaSizeApp = AppGetMediaSize;
        AC_AppHandle.getHidReportApp = appGetHidReport;
        AC_AppHandle.ctrlHandler  = appCtrlFxn;
        AC_AppHandle.isoHandler   = appIsoFxn;
        AC_AppHandle.hidHandler = appHidFxn;

        AC_AppHandle.numLun = 2;

        /* Initialize End point descriptors */
        AC_initDescriptors(AC_AppHandle.pAcObj, (Uint16 *)deviceDescriptorB,
                            CSL_AC_DEVICE_DESCR, sizeof(deviceDescriptorB));

        AC_initDescriptors(AC_AppHandle.pAcObj, (Uint16 *)deviceQualifierDescr,
                            CSL_AC_DEVICE_QUAL_DESCR, 10);

        AC_initDescriptors(AC_AppHandle.pAcObj, (Uint16 *)configDescriptor,
                            CSL_AC_CONFIG_DESCR, sizeof(configDescriptor));

        AC_initDescriptors(AC_AppHandle.pAcObj, (Uint16 *)acHidReportDescriptor,
                            CSL_AC_HID_REPORT_DESC, sizeof(acHidReportDescriptor));

        /* Initialize HID */
        AC_AppHandle.acHidIfNum = IF_NUM_HID; // HID interface number
        AC_AppHandle.acHidReportId = HID_REPORT_ID; // HID report ID
        AC_AppHandle.acHidReportLen = HID_REPORT_SIZE_BYTES; // HID report length (bytes)
        genHidReport(UI_PUSH_BUTTON_NONE, gHidReport); // init. HID report for Get Report 

        /* Call Init API */
        AC_Open(&AC_AppHandle);

        /* Enable CPU USB interrupts */
        CSL_FINST(CSL_CPU_REGS->IER1, CPU_IER1_USB, ENABLE);

        /* Initialize active playback sample rate */
#ifdef SAMPLE_RATE_TX_48kHz
        initSampleRate(RATE_48_KHZ, &active_sample_rate, 
            &i2sTxBuffSz);
#elif defined (SAMPLE_RATE_TX_16kHz)
        initSampleRate(RATE_16_KHZ, &active_sample_rate, 
            &i2sTxBuffSz);
#endif
        /* Initialize ASRC */ 
        Init_Sample_Rate_Converter(active_sample_rate);

        /* Reset codec output buffer */
        reset_codec_output_buffer();

        #ifdef ENABLE_RECORD
        #ifdef SAMPLE_RATE_RX_48kHz
        LOG_printf(&trace, "RECORD: 48KHZ ");
        #else
        LOG_printf(&trace, "RECORD: 16KHZ ");
        #endif // SAMPLE_RATE_RX_48kHz
        /* Start left Rx DMA */
        DMA_StartTransfer(hDmaRxLeft);

        #ifdef ENABLE_STEREO_RECORD
        LOG_printf(&trace, "STEREO\n");
        /* Start right Rx DMA */
        DMA_StartTransfer(hDmaRxRight);
         #else
        LOG_printf(&trace, "MONO\n");
        #endif

        #endif // ENABLE_RECORD

#ifdef STORE_PARAMETERS_TO_SDRAM
        initSdram(FALSE, 0x0000);
#endif // STORE_PARAMETERS_TO_SDRAM
#ifdef SAMPLE_BY_SAMPLE_PB
        /* SampleBySample, init interrupt */       
        /* Use with compiler "interrupt" keyword */
        //IRQ_plug(I2S_TX_EVENT, i2s_txIsr);
        
        /* Use with dispatcher, no "interrupt" keyword */
        attrs.ier0mask = 0xFFFF;
        attrs.ier1mask = 0xFFFF;
        HWI_dispatchPlug(I2S_TX_EVENT, (Fxn)i2s_txIsr, &attrs);

        IRQ_enable(I2S_TX_EVENT);   /* SampleBySample, enable IRQ for I2S Tx */
#endif
#if defined(SAMPLE_BY_SAMPLE_REC) && !defined(COMBINE_I2S_TX_RX_ISR)
        /* SampleBySample, init interrupt */
        /* Use with compiler "interrupt" keyword */
        //IRQ_plug(I2S_RX_EVENT, i2s_rxIsr);
        
        /* Use with dispatcher, no "interrupt" keyword */
        attrs.ier0mask = 0xFFFF;
        attrs.ier1mask = 0xFFFF;
        HWI_dispatchPlug(I2S_RX_EVENT, (Fxn)i2s_rxIsr, &attrs);

        IRQ_enable(I2S_RX_EVENT);    /* SampleBySample, enable IRQ for I2S Rx */
#endif
#if defined(SAMPLE_BY_SAMPLE_PB) || defined(SAMPLE_BY_SAMLE_REC)
        DDC_I2S_transEnable((DDC_I2SHandle)i2sHandleTx, TRUE);    /* SampleBySample, enable I2S transmit and receive */
#endif
#ifndef SAMPLE_BY_SAMPLE_PB
        i2sTxStart(); // - moved from appPlayAudio()
#endif

        /* Set HWAI and IPORT in ICR */
        *(volatile unsigned int *)0x0001 = (1<<9) | (1<<8);
        asm("   idle");     

        /* Clock gate usused peripherals */
		ClockGating();
    }
}

/* Resets C5515 */
void C5515_reset(void)
{
    volatile int i;

    // disable all interrupts (IER0 and IER1)
    *(volatile ioport unsigned int *)(0x0000) = 0x0000;
    *(volatile ioport unsigned int *)(0x0045) = 0x0000;

    // clear all interrupts (IFR0 and IFR1)
    *(volatile ioport unsigned int *)(0x0001) = 0xFFFF;
    *(volatile ioport unsigned int *)(0x0046) = 0xFFFF;

    // enable all peripherials
    *(volatile ioport unsigned int *)(0x1c02) = 0;
    *(volatile ioport unsigned int *)(0x1c03) = 0;

    // reset peripherals
    *(volatile ioport unsigned int *)(0x1c04) = 0x0020;
    *(volatile ioport unsigned int *)(0x1c05) = 0x00BF;
    // some delay
    for (i=0; i<0xFFF; i++);

    // clear all interrupts (IFR0 and IFR1)
    *(volatile ioport unsigned int *)(0x0001) = 0xFFFF;
    *(volatile ioport unsigned int *)(0x0046) = 0xFFFF;
}

#ifdef C5535_EZDSP_DEMO
// Power Display task code
void PowerDisplayTask(void)
{
	while (1)
	{
		// sleep for one second
		TSK_sleep(1000);
		// read and display the power usage
		if (DemoSwitchFlag==0)
		{
			CSL_i2cPowerTest();
		}
	}
}

// Spectrum Display Task code
void SpectrumDisplayTask(void)
{
	// display the play audio message
	print_playaudio();

	while (1)
	{
		// wait on bufferIn ready semaphore
		SEM_pend(&SEM_BufferInReady, SYS_FOREVER);
		// compute and display the bargraph
		if (DemoSwitchFlag)
		{
			calculate_FFT(bufferIn, 256);
			// clear the bufferInIdx to 0
			bufferInIdx = 0;
		}		
	}
}

// PRD function. Runs every 10 second to switch the demo mode between
// power display mode and spectrum analyzer mode
void DemoSwitch(void)
{
	DemoSwitchFlag++;
	if (DemoSwitchFlag==1)
	{
		// if we were in power display mode, swtch to spectrum analyzer mode
		// clear the bufferInIdx
		bufferInIdx = 0; 
	}
	else if (DemoSwitchFlag==3)
	{
		// if we were in spectrum analyzer mode, switch to power display mode
		DemoSwitchFlag = 1;
		// stop data collection for spectrum analyzer
		bufferInIdx = 0;
	}
}
#endif

// Clock gating for unused peripherals
void ClockGating(void)
{
	Uint16 pcgcr_value, clkstop_value;
	
	// set PCGCR1
	pcgcr_value = 0; 
	// clock gating SPI
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_SPICG, DISABLED);
	// clock gating SD/MMC
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_MMCSD0CG, DISABLED);
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_MMCSD1CG, DISABLED);
	// clock stop request for UART
	clkstop_value = CSL_FMKT(SYS_CLKSTOP_URTCLKSTPREQ, REQ);
	// write to CLKSTOP
	CSL_FSET(CSL_SYSCTRL_REGS->CLKSTOP, 15, 0, clkstop_value);
	// wait for acknowledge
	while (CSL_FEXT(CSL_SYSCTRL_REGS->CLKSTOP, SYS_CLKSTOP_URTCLKSTPACK)==0);
	// clock gating UART
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_UARTCG, DISABLED);
	// clock stop request for EMIF
	//clkstop_value = CSL_FMKT(SYS_CLKSTOP_EMFCLKSTPREQ, REQ);
	// write to CLKSTOP
	//CSL_FSET(CSL_SYSCTRL_REGS->CLKSTOP, 15, 0, clkstop_value);
	// wait for acknowledge
	//while (CSL_FEXT(CSL_SYSCTRL_REGS->CLKSTOP, SYS_CLKSTOP_EMFCLKSTPACK)==0);
	// clock gating EMIF
	//pcgcr_value |= CSL_FMKT(SYS_PCGCR1_EMIFCG, DISABLED);
	// clock gating unused I2S (I2S 0, 1, 3)
	//pcgcr_value |= CSL_FMKT(SYS_PCGCR1_I2S0CG, DISABLED);
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_I2S1CG, DISABLED);
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_I2S2CG, DISABLED);
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_I2S3CG, DISABLED);
	// clock gating DMA0
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_DMA0CG, DISABLED);
	// clock gating Timer 1
	//pcgcr_value |= CSL_FMKT(SYS_PCGCR1_TMR1CG, DISABLED);
	// clock gating Timer 2
	pcgcr_value |= CSL_FMKT(SYS_PCGCR1_TMR2CG, DISABLED);
	// write to PCGCR1
	CSL_FSET(CSL_SYSCTRL_REGS->PCGCR1, 15, 0, pcgcr_value);
	
	// set PCGCR2
	pcgcr_value = 0; 
	// clock gating LCD
	pcgcr_value |= CSL_FMKT(SYS_PCGCR2_LCDCG, DISABLED);
	// clock gating SAR
	//pcgcr_value |= CSL_FMKT(SYS_PCGCR2_SARCG, DISABLED);
	// clock gating DMA1
	//pcgcr_value |= CSL_FMKT(SYS_PCGCR2_DMA1CG, DISABLED);
	// clock gating DMA2
	pcgcr_value |= CSL_FMKT(SYS_PCGCR2_DMA2CG, DISABLED);
	// clock gating DMA3
	pcgcr_value |= CSL_FMKT(SYS_PCGCR2_DMA3CG, DISABLED);
	// clock analog registers
	//pcgcr_value |= CSL_FMKT(SYS_PCGCR2_ANAREGCG, DISABLED);
    // write to PCGCR2
	CSL_FSET(CSL_SYSCTRL_REGS->PCGCR2, 15, 0, pcgcr_value);
	
	// disable the CLKOUT. It is on reset
	// set bit 2 of ST3_55 to 1
	asm("	bit(ST3, #ST3_CLKOFF) = #1");
	
	// turn off the XF
	// set bit 13 of ST1_55 to 0
	asm("	bit(ST1, #ST1_XF) = #0");

#ifdef C5535_EZDSP
    // turn off the DS3-6
	// set the GPIO pin 14 - 15 to output, set SYS_GPIO_DIR0 (0x1C06) bit 14 and 15 to 1 
    *(volatile ioport unsigned int *)(0x1C06) |= 0xC000;
	// set the GPIO pin 16 - 17 to output, set SYS_GPIO_DIR1 (0x1C07) bit 0 and 1 to 1 
    *(volatile ioport unsigned int *)(0x1C07) |= 0x0003;
	
	// set the GPIO 14 - 15 to 0, set SYS_GPIO_DATAOUT0 (0x1C0A) bit 14 and 15 to 0
    //*(volatile ioport unsigned int *)(0x1C0A) &= 0x3FFF;
    *(volatile ioport unsigned int *)(0x1C0A) |= 0xC000;
	// set the GPIO 16 - 17 to 0, set SYS_GPIO_DATAOUT1 (0x1C0B) bit 0 and 1 to 0
    //*(volatile ioport unsigned int *)(0x1C0B) &= 0xFFFC;
    *(volatile ioport unsigned int *)(0x1C0B) |= 0x0003;
#endif

	return;
}

#if 0
void userIdle(void)
{
	// set CPUI bit in ICR
	*(ioport volatile unsigned int *)(0x0001) = 0x000F;
	// execute idle instruction to make CPU idle
	asm("	idle");		
}
#endif

