/*
 * $$$MODULE_NAME codec_config.c
 *
 * $$$MODULE_DESC codec_config.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#include <std.h>
#include "VC5505_CSL_BIOS_cfg.h"
#include "codec_aic3254.h"
#include "codec_config.h"

/* Reconfigures codec in response to USB message */
void CodecConfigTask(void)
{
    CodecCfgMsgObj codecCfgMsg;
    Int16 *pData;

    while (1)
    {
        //TSK_settime(TSK_self()); // statistic collection

        if (MBX_pend(&MBX_codecConfig, &codecCfgMsg, SYS_FOREVER))
        {
            switch (codecCfgMsg.wMsg)
            {
                case CODEC_CFG_MSG_ADJ_VOL_L:
                    pData =  (Int16 *)codecCfgMsg.wData;
                    Adjust_Volume(*pData, 0);
                    break;

                case CODEC_CFG_MSG_ADJ_VOL_R:
                    pData =  (Int16 *)codecCfgMsg.wData;
                    Adjust_Volume(*pData, 1);
                    break;

                case CODEC_CFG_MSG_ADJ_MUTE:
                    pData =  (Int16 *)codecCfgMsg.wData;
                    if ((*pData & 0xff) == 0)
                    {
                        // un-mute
                        //STS_set(&mySts1, CLK_gethtime());
                        if(Set_Mute_State(FALSE) == FALSE)
                        {
                            LOG_printf(&trace, "FAILED MUTE CLEAR\n");
                        }
                        //STS_delta(&mySts1, CLK_gethtime());
                        //TSK_deltatime(TSK_self()); // statistic collection
                        break;
                    }
                    else if ((*pData & 0xff) == 1)
                    {
                        // mute
                        //STS_set(&mySts1, CLK_gethtime());
                        if(Set_Mute_State(TRUE) == FALSE)
                        {
                            LOG_printf(&trace, "FAILED MUTE SET\n");
                        }
                        //STS_delta(&mySts1, CLK_gethtime());
                        //TSK_deltatime(TSK_self()); // statistic collection
                    }

                    break;

                default:
                    break;
            }
        }
    }
}
