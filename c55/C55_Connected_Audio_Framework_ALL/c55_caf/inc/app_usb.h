/*
 * $$$MODULE_NAME app_usb.h
 *
 * $$$MODULE_DESC app_usb.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _APP_USB_H_
#define _APP_USB_H_

#include <std.h>
#include "csl_usb.h"
#include "app_globals.h"
#include "psp_i2s.h"

/* USB interface numbers */
#define IF_NUM_AC       (0) /* Interface Number Audio Control, must be zero */
#define IF_NUM_REC      (1) /* Interface Number Audio Record */
#define IF_NUM_PLAY     (2) /* Interface Number Audio Playback */
#define IF_NUM_HID      (3) /* Interface Number HID */

/* USB descriptor sample rate settings */
#define RATE_16_KHZ     0x3E80
#define RATE_32_KHZ     0x7D00
#define RATE_44_1_KHZ   0xAC44
#define RATE_48_KHZ     0xBB80

#if defined(COUNT_REC_SAMPS_IN_USB_FRAME_REC) || defined(COUNT_REC_SAMPS_IN_USB_FRAME_PB)
#ifndef ENABLE_STEREO_RECORD
#define EP_REC_MAXP     ( 0x0064 ) /* maximum packet size for record endpoint */
//#define EP_REC_MAXP     ( 0x0040 ) /* maximum packet size for record endpoint */
#else /* ENABLE_STEREO_RECORD */
#define EP_REC_MAXP     ( 0x00C8 ) /* maximum packet size for record endpoint */
#endif /* ENABLE_STEREO_RECORD */

#else
#define EP_REC_MAXP     ( 0x0020 ) /* maximum packet size for record endpoint */

#endif
#define EP_PB_MAXP      ( 0x00C0 ) /* maximum packet size for playback endpoint */ // note can't currently be changed
#define EP_HID_MAXP     ( 0x0003 ) /* maximum packet size for HID endpoint */

#define APP_USB_SELF_POWERED    /* define if self-powered USB device */
#ifdef APP_USB_SELF_POWERED
#define APP_USB_MAX_CURRENT     ( 200/2 ) /* max current in units of 2 mA */
#else
#define APP_USB_MAX_CURRENT     ( 500/2 ) /* max current in units of 2 mA */
#endif

/* USB Setup packet bit fields for USB bmRequestType */
#define CSL_USB_REQTYPE_XFER_HTOD      ( 0x0 << 7 )
#define CSL_USB_REQTYPE_XFER_DTOH      ( 0x1 << 7 )
#define CSL_USB_REQTYPE_STANDARD       ( 0x0 << 5 )
#define CSL_USB_REQTYPE_CLASS          ( 0x1 << 5 )
#define CSL_USB_REQTYPE_VENDOR         ( 0x2 << 5 )
#define CSL_USB_REQTYPE_RCVR_DEVICE    ( 0x0 << 0 )
#define CSL_USB_REQTYPE_RCVR_INTERFACE ( 0x1 << 0 )
#define CSL_USB_REQTYPE_RCVR_ENDPOINT  ( 0x2 << 0 )

extern Uint16 h_usb_int_tcount;

// used to delay the start of the record process
extern Uint8 set_record_interface;

extern Bool usb_play_mode;
extern Bool rdy_to_consume_asrc_output;

extern Bool  no_main_task;

/* HID endpoint control variable */
extern Bool hidIntInEpReady;

#if 0 // debug
extern Uint16 gFaddr_PowerDbg;
extern Uint16 gRxFifoSz_TxFifoSzDbg;
extern Uint16 gTxFifoAddrDbg;
extern Uint16 gTxMaxpDbg;
extern Uint16 gTxCsrDbg;
extern Uint16 gRxFifoAddrDbg;
extern Uint16 gRxMaxpDbg;
extern Uint16 gRxCsrDbg;
extern Uint16 gCsr0Dbg;
extern Uint16 gSaveIndex;
#endif

Bool StartDevice(
    pUsbContext pContext
);

Bool StopDevice(
    pUsbContext pContext
);

Bool HandleUSBInterrupt(
    pUsbContext pContext
);

void  MUSB_Handle_EP0_Intr(
    pUsbContext pContext
);

void MUSB_Handle_Resume_Intr(void);

void CSL_suspendCallBack(
    CSL_Status    status
);

void CSL_selfWakeupCallBack(
    CSL_Status    status
);

CSL_Status StartTransfer(
    void    *vpContext,
    void    *vpeps
);

CSL_Status CompleteTransfer(
    void    *vpContext,
    void    *vpeps
);

/**
 *  \brief  Function to send device notification
 *
 *  \param  pContext    - USB context structure pointer
 *  \param  wUSBEvents  - USB events
 *
 *  \return None
 */
void DeviceNotification(
    pUsbContext    pContext,
    WORD           wUSBEvents
);

void USB_configEndpointDataSize(
    pUsbContext    pContext,
    DWORD          dwEndpoint
);

/* Reads TXMAXP for selected endpoint */
Uint16 USB_getTxMaxp(Uint16 epNum);
/* Writes TXMAXP for selected endpoint */
Uint16 USB_setTxMaxp(Uint16 epNum, Uint16 val);
/* Reads RXMAXP for selected endpoint */
Uint16 USB_getRxMaxp(Uint16 epNum);
/* Writes RXMAXP for selected endpoint */
Uint16 USB_setRxMaxp(Uint16 epNum, Uint16 val);
/* Reads TXCSR for selected endpoint */
Uint16 USB_getTxCsr(Uint16 epNum);
/* Writes TXCSR for selected endpoint */
Uint16 USB_setTxCsr(Uint16 epNum, Uint16 val);
/* Reads RXCSR for selected endpoint */
Uint16 USB_getRxCsr(Uint16 epNum);
/* Writes RXCSR for selected endpoint */
Uint16 USB_setRxCsr(Uint16 epNum, Uint16 val);

#endif /* _APP_USB_H_ */
