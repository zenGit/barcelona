/*
 * $$$MODULE_NAME psp_i2c.h
 *
 * $$$MODULE_DESC psp_i2c.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

/**
 *  \file   psp_i2c.h
 *
 *  \brief  I2C interface definition
 *
 *  This file contains the interfaces, data types and symbolic definitions
 *  that are needed by the application to utilize the serivces of the I2C
 *  device driver.
 *
 *  (C) Copyright 2005, Texas Instruments, Inc
 *
 *  \author    psg
 *
 *  \version   1.x - Anant Gole         - previous versions of interfaces
 *             2.0 - Nagarjuna Kristam  - cleanup effort and MISRAC compliance
 *             3.0 - NAgarjuna Kristam  - New implementation
 */

#ifndef _PSP_I2C_H_
#define _PSP_I2C_H_

#include <psp_common.h>
#include <tistdtypes.h>

/**
 * \brief  I2C Driver Error codes
 */
#define PSP_I2C_ERROR_BASE              (-11)
/**< Error Code base                                                         */
#define PSP_I2C_BUS_BUSY_ERR            (PSP_I2C_ERROR_BASE - 1)
/**< Returned when the I2C bus find that the bus is busy                     */
#define PSP_I2C_ARBITRATION_LOSS_ERR    (PSP_I2C_ERROR_BASE - 2)
/**< Returned when the I2C driver lost the bus arbitration                   */
#define PSP_I2C_NACK_ERR                (PSP_I2C_ERROR_BASE - 3)
/**< Returned when the I2C slave did not generate an acknowledge             */
#define PSP_I2C_TRANSMIT_UNDERFLOW_ERR  (PSP_I2C_ERROR_BASE - 4)
/**< Returned in case of an transmit underflow error                         */
#define PSP_I2C_RECEIVE_OVERFLOW_ERR    (PSP_I2C_ERROR_BASE - 5)
/**< Returned in case of an rcv overflow error                               */
#define PSP_I2C_CANCEL_IO_ERROR         (PSP_I2C_ERROR_BASE - 6)
/**< Returned in case of an Cancelling IO error                              */

/* Note: The following flags offer the user maximum flexibility in terms
 * of making the right I2C transaction. In case the user does not want
 * to set the flags on his own, the default read/write flag can be specified
 *
 * NOTE: If no flag is specified by default in the flags, read is assumed
 */
#define PSP_I2C_READ                0x1u
/**< Read from I2C bus (device)                                              */
#define PSP_I2C_WRITE               0x2u
/**< Write to I2C bus (device)                                               */
#define PSP_I2C_ADDR_FORMAT_10_BIT  0x10u
/**< If this flag is not set, default is 7 bit address                       */
#define PSP_I2C_MASTER              0x20u
/**< If this flag is not set, default is slave mode                          */
#define PSP_I2C_START               0x100u
/**< Generate Start - valid in master mode only                              */
#define PSP_I2C_STOP                0x200u
/**< Generate Stop - valid in master mode only                               */
#define PSP_I2C_RESTART             0x400u
/**< Re-Start is generated by Master                                         */
#define PSP_I2C_START_BYTE          0x800u
/**< Start Byte as per Phillips I2C specs                                    */
#define PSP_I2C_FREE_DATA_FORMAT    0x1000u
/**< Free Data Format                                                        */
#define PSP_I2C_REPEAT              0x2000u
/**< Repeat mode as per TI I2C specs                                         */
#define PSP_I2C_IGNORE_BUS_BUSY     0x20000u
/**< Ignore Bus Busy condition                                               */

/* NOTE: Debug option: User should know that this will result into not checking
         if bus is busy and may result into other errors */

/* Use these flags for simple read/write transactions on the I2C bus */
#define PSP_I2C_DEFAULT_READ \
                (PSP_I2C_READ | PSP_I2C_MASTER | PSP_I2C_START | PSP_I2C_STOP)
/**< Default read flag                                                       */
#define PSP_I2C_DEFAULT_WRITE \
                (PSP_I2C_WRITE | PSP_I2C_MASTER | PSP_I2C_START | PSP_I2C_STOP)
/**< Default write flag                                                      */

/**
 *  \brief  PSP I2C Ioctl commands
 *
 *  I2C Ioctl commands
 */
typedef enum
{
    PSP_I2C_IOCTLE_SET_RX_SLAVE_CB,
    /**< Set callback to be called when i2c is working in Slave mode         */
    PSP_I2C_IOCTL_CANCEL_PENDING_IO,
    /**< To cancel pending IO                                                */
    PSP_I2C_IOCTL_TESTS,
    /**< For testing static configurations                                   */
    PSP_I2C_IOCTL_BIT_COUNT,
    /**< To set bit Count value                                              */
    PSP_I2C_IOCTL_NACK
    /**< To enable or disable NACK dynamically                               */
} PSP_I2cIoctlCmd;

/**
 *  \brief I2C PSP Configuration Object
 *
 *  This structure basically holds the mode of operation and passes
 *  it on to the driver.
 */
typedef struct _PSP_I2cConfig
{
    Uint32      i2cOwnAddr;     /**< Own address (7 or 10 bit)               */
    Uint32      numBits;        /**< Number of bits/byte to be sent/received */
    Uint32      i2cBusFreq;     /**< I2C Bus Frequency                       */
    Uint32      addressing;     /**< 7bit/10bit Addressing mode              */
    Uint32      repeatMode;     /**< Repeat Mode                             */
    Uint32      dlb;            /**< Digital Loob Back (DLB) mode            */
    PSP_OpMode  mode;           /**< Driver operating mode - polled/intr/dma */
} PSP_I2cConfig;

/**
 *  \brief  I2C Transaction structure
 *
 *  This structure holds the information needed to carry out a transaction on an
 *  I2C bus to a slave device.
 */
typedef struct _PSP_I2cTransaction
{
    Uint32              slaveAddr;
    /**< Address of the slave to talk to, not valid in Slave mode            */
    Uint16              *buffer;
    /**< Data Buffer                                                         */
    Uint32              bufLen;
    /**< Length of buffer                                                    */
    Uint32              flags;
    /**< Flags to indicate the various modes of operation                    */
    Ptr                 param;
    /**< Extra parameter for future use                                      */
} PSP_I2cTransaction;

/**
 *  \brief  IOM_PACKET element structure
 *
 *  Structure for the i2c specific buffer address to be passed to the GIO.
 */
typedef struct _PSP_I2cDataParam
{
    PSP_I2cTransaction  i2cTrans;
    /**< i2c transaction structure used to pass to submit                    */
    Int                 timeout;
    /**< Timeout value                                                       */
} PSP_I2cDataParam;

/**
 *  \brief CallBack function for I2C.
 *   Note:This is not supported currently as driver does not support
 *        async mode of operation currently.
 *
 *  This function will be called when any transation
 *  has been completed when driver is operating in asynch mode.
 *
 *  \param  appHandle[OUT] Regsitered by application and will be returned
 *                         along with call back.
 *  \param  event    [OUT] Event associated for which call back has been called
 *  \param  Param    [OUT] Any parameter returned from driver
 */
typedef void (*PSP_I2cAppCallback)(Ptr appHandle,Int event, Ptr param);

/**
 *  \brief hwEventCallBack function for I2C.
 *   Note: This is not supported currently as driver does not support
 *   slave mode of operation currently for I2C.
 *
 *  This function will be called when any transation
 *  has been completed when driver is operating in asynch mode.
 *
 *  \param  appSlvHandle [OUT] Regsitered by application and will be returned
 *                             along with call back.
 *  \param  event        [OUT] Event assocaited for which call back
 *                             has been called
 *  \param  data         [OUT] Data returned from driver
 */
typedef void (*PSP_I2cHwEventCallback)(Ptr   appSlvHandle,
                                       Int8  event,
                                       Uint8 data);

/**
 *  \brief Initialize a given I2C driver (instance)
 *
 *  Typically, software bookkeeping functions are performed in this call.
 *  Memory for device instance specific data structures may be allocated
 *  and initialized. Configuration information may be passed in the call
 *  and initialization based upon this information is done.
 *
 *  \param  instanceId [IN]     I2C instance number
 *  \param  initConfig [IN] Initialization configuration
 *
 *  \return PSP_SOK or PSP Error code
 */
PSP_Result  PSP_i2cCreate(Uint32  instanceId,const PSP_I2cConfig *initConfig);

/**
 *  \brief Delete a given I2C driver (instance)
 *
 *  The instance of I2C is deleted in software. If there are open handles
 *  then they are invalidated
 *
 *  \param  instanceId [IN]        I2C instance number
 *  \return PSP_SOK or PSP Error code
 */
PSP_Result PSP_i2cDelete(Uint32 instanceId);

/**
 *  \brief Open Instance of the I2C device
 *
 *  This function prepares the device hardware for data transfers and
 *  usage by the upper layer driver software. The DDA layer is expected
 *  to install the ISR and enable it only after the completion of this
 *  call. The function prepares the driver instance for data transfers
 *  and returns an handle to the driver instance.
 *
 * Note: The driver can be opened multiple times
 *  \param  instanceId [IN]    I2C instance number
 *  \param  callback   [IN]    Callback function for events notification
 *  \param  appHandle  [IN]    This appHandle will be provided by user and
 *                             will be returned along with call back
 *  \return PSP_Handle [OUT]   If successful positive Driver Instance Handle
 *                             else return the NULL pointer.
 */
PSP_Handle  PSP_i2cOpen(Uint32 instanceId,
                        PSP_I2cAppCallback callback,
                        Ptr appHandle);

/**
 *  \brief Close Instance of the I2C device
 *
 *  This function closes the device for data transfers and usage by the
 *  upper layer driver software. The hardware is programmed to stop/abort data
 *  transfer (depending upon the type of device and its specifics) and the
 *  device ISR is "disabled" by the upper layer driver software after the
 *  completion of this call. After the successful completion of this call, the
 *  device cannot perform any data IO.
 *
 *  \param  handle [IN]         I2C Driver Instance Handle
 *  \return PSP_SOK or PSP Error code
 */
PSP_Result  PSP_i2cClose(PSP_Handle handle);

/**
 *  \brief Read/Write (single transaction) using I2C
 *
 *  This function read/writes single transaction data using the I2C peripheral.
 *
 *  \param  handle    [IN]    I2C Driver Instance Handle
 *  \param  slaveAddr [IN]    Slave address (only valid in Master mode)
 *  \param  buffer    [IN]    Buffer (to read into or write from)
 *  \param  bufLen    [IN]    Buffer length - number of bytes to transact
 *  \param  flags     [IN]    Flags to specify mode of operation
 *  \param  timeout   [IN]    Timeout for i2c transaction
 *  \param  param     [IN]    Extra paramter for future usage
 *
 *  \return On success returns PAL_SOK, else negative error code
 */
Int PSP_i2cTransfer (PSP_Handle handle,
                     Uint32     slaveAddr,
                     Uint16     *buffer,
                     Uint32     bufLen,
                     Uint32     flags,
                     Int32      timeout,
                     Ptr        param);

/**
 *  \brief Read/Write multiple transactions using I2C
 *
 *  This function read/writes multiple transactions data using the I2C peripheral.
 *  Though nothing prevents the user from using multiple invokations of the
 *  single transaction transfer function, this function helps in making sure that
 *  all the transactions are completed in one go before another application gets
 *  hold of the i2c bus (from the applications perspective)
 *
 *  \param  handle  [IN]    I2C Driver Instance Handle
 *  \param  xfer    [IN]    I2C Transaction array - first element pointer
 *  \param  numXfer [IN]    Number of transactions to be done
 *  \param  param   [IN]    Extra paramter for future usage
 *  \param  timeout [IN]    Timeout for whole i2c transaction
 *  \return PSP_SOK or Transmission number where error occured
 */
PSP_Result  PSP_i2cTransferMultiple (PSP_Handle     handle,
                                     const PSP_I2cTransaction *xfer,
                                     Uint32         numXfer,
                                     Int32          timeout,
                                     Ptr            param);

/**
 *  \brief IO Control for I2C controller.
 *
 *  This function supports various IOCTLs for the I2C controller.
 *
 *  \param  handle [IN]    I2C Driver Instance Handle
 *  \param  cmd    [IN]    IOCTL command
 *  \param  cmdArg [IN]    Arguments, if any, for the command
 *  \param  param  [IN]    IOCTL specific parameter (driver specific).
 *
 *  \return PSP_SOK or PSP Error code.
 */
PSP_Result  PSP_i2cIoctl(PSP_Handle      handle,
                         PSP_I2cIoctlCmd cmd,
                         Ptr             cmdArg,
                         Ptr             param);

#endif  /* _PSP_I2C_H_ */
