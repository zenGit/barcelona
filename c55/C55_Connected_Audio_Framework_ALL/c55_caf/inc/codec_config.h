/*
 * $$$MODULE_NAME codec_config.h
 *
 * $$$MODULE_DESC codec_config.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _CODEC_CFG_H_
#define _CODEC_CFG_H_

typedef enum {
	/** Adjust left volume message                                           */
    CODEC_CFG_MSG_ADJ_VOL_L = 0,
	/** Adjust right volume message                                          */
    CODEC_CFG_MSG_ADJ_VOL_R,
	/** Adjust mute message                                                  */
    CODEC_CFG_MSG_ADJ_MUTE
} CodecConfigMsg;

typedef struct CodecCfgMsgObj {
	/** \brief  Message posted                                               */
    Int16    wMsg;
    /** \brief  Message data                                                 */
    void    *wData;
} CodecCfgMsgObj;


#endif /* _CODEC_CFG_H_ */
