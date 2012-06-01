/*
 * $$$MODULE_NAME app_usbac_descs.h
 *
 * $$$MODULE_DESC app_usbac_descs.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _APP_USBAC_DESCS_H_
#define _APP_USBAC_DESCS_H_

#include <std.h>

/* Enable adding second playback sample rate */
#define ENABLE_PLAYBACK_TWO_SAMPLE_RATES

#define DEVICE_DESCR_SZ             ( 9 )
#define DEVICE_DESCR_B_SZ           ( 18 )
#define DEVICE_QUALIFIER_DESCR_SZ   ( 5 )
#define STRING_LAN_ID_SZ            ( 4 )
#define STRING_DESCR_SZ             ( 7 )
#define AC_HID_REPORT_DESCR_SZ      ( 53 )
#ifdef ENABLE_PLAYBACK_TWO_SAMPLE_RATES
#define CONFIG_DESCR_SZ             ( 224 )
#else
#define CONFIG_DESCR_SZ             ( 221 )
#endif

/* Vendor ID (idVendor) */
#define DEV_VID ( 0x0451 )
/* Product ID (idProduct) */
#define DEV_PID ( 0x9010 )
/* bcdDevice USB Device Firmware version number example: 0x1234 = version 12.34 */
#define DEV_BCD ( 0x0001 )

/* constants for total number of interface cout */
#define IF_AUDIO_CTRL   (1) /* audio control interface */
#define IF_AUDIO_REC    (1) /* audio record interface */
#define IF_AUDIO_PLAY   (1) /* audio playback interface */
#define IF_HID          (1) /* HID interface */
#define IF_COUNT        (IF_AUDIO_CTRL + IF_AUDIO_REC + IF_AUDIO_PLAY + IF_HID)

/* constant for audio streaming interface count, NOTE: audio control interface is not included */
#define IF_STRM_CNT     (IF_AUDIO_REC + IF_AUDIO_PLAY)

/* USB audio terminal ID definitions, MUST be one based and unique */
#define T_IT_MIC_REC    (0x04)  /* Input Terminal ID Microphone Record */
#define T_FU_REC        (0x05)  /* Feature Unit Terminal ID Record */
#define T_OT_STRM_REC   (0x06)  /* Output Terminal ID Streaming Record */

#define T_IT_STRM_PLAY  (0x01)  /* Input Terminal ID Streaming Playback */
#define T_FU_PLAY       (0x02)  /* Feature Unit Terminal ID Plyaback */
#define T_OT_SPKR       (0x03)  /* Output Terminal ID Playback Speaker */

//#define REC_SAMPLE_RATE (RATE_48_KHZ)    /* audio record sample rate */
#define REC_SAMPLE_RATE (RATE_16_KHZ)    /* audio record sample rate */

#ifdef ENABLE_PLAYBACK_TWO_SAMPLE_RATES
#define PLAY_SAMPLE_RATE_1 (RATE_48_KHZ)   /* audio playback sample rate */
#define PLAY_SAMPLE_RATE_2 (RATE_16_KHZ)   /* audio playback sample rate */
#else
#define PLAY_SAMPLE_RATE (RATE_48_KHZ)   /* audio playback sample rate */
//#define PLAY_SAMPLE_RATE (RATE_16_KHZ)   /* audio playback sample rate */
#endif

/* full speed endpoint Direction and Address numbers */
#define EP_NUM_REC      (0x81)  /* EP1-IN Direction, Address 1 */
#define EP_NUM_PLAY     (0x02)  /* EP2-OUT Direction, Address 2 */
#define EP_NUM_HID      (0x83)  /* EP3-IN Direction, Address 3 */

extern const Uint16 deviceDescriptor[DEVICE_DESCR_SZ];
extern const Uint16 deviceDescriptorB[DEVICE_DESCR_B_SZ];
extern const Uint16 deviceQualifierDescr[DEVICE_QUALIFIER_DESCR_SZ];
extern const Uint16 configDescriptor[CONFIG_DESCR_SZ];
extern const char *string_descriptor[STRING_DESCR_SZ];
extern const Uint16 stringLanId[STRING_LAN_ID_SZ];
extern const Uint16 acHidReportDescriptor[AC_HID_REPORT_DESCR_SZ];

#endif /* _APP_USBAC_DESCS_H_ */
