/*
 * $$$MODULE_NAME asrc_fix.h
 *
 * $$$MODULE_DESC asrc_fix.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef _ASRC_FIX_H_
#define _ASRC_FIX_H_

#include "data_types.h"

#define ASRC_SOK                        ( 0 )           /* indicates successful return status */
#define ASRC_INV_PRM                    ( 1 )           /* indicates invalid parameter to function */
#define ASRC_SRC_FIFO_UND               ( 2 )           /* indicates input FIFO underflow */
#define ASRC_SRC_FIFO_OVR               ( 3 )           /* indicates input FIFO overflow */

#define ASRC_NUM_CH_MONO                ( 1 )           /* stream is mono */
#define ASRC_NUM_CH_STEREO              ( 2 )           /* stream is stereo */

#define ASRC_IN_FMT_SPLITBUF            ( 0 )           /* input stream is split buffer: left block, right block, ... */
#define ASRC_IN_FMT_INTERLEAVED         ( 1 )           /* input stream is interleaved: left samp, right samp, left samp, right samp, ... */

/* Loop filter gains */
/* No SOF jitters */
#define ASRC_DEF_K0                     ( 0x346DC )     /* 1.0/20000.0 in U32Q32 */
#define ASRC_DEF_K1                     ( 0xD1B7 )      /* 1.0/80000.0 in U32Q32 */
/* SOF Jitters */
//#define ASRC_DEF_K0                     ( 0x10624D )    /* 1.0/4000.0  in U32Q32 */
//#define ASRC_DEF_K1                     ( 0x0 )
//#define ASRC_DEF_K0                     ( 0x28F5C29 )   /* 0.01 in U32Q32 */
//#define ASRC_DEF_K1                     ( 0x418937 )    /* 0.001 in U32Q32 */

#define ASRC_DEF_NOM_PHASE_INCR         ( ASRC_NOM_PHASE_INCR_1 )   /* default nominal phase increment */
#define ASRC_NOM_PHASE_INCR_1           ( 0x10000 )     /* nominal phase increment == 1.0 in S32Q16, use for (fso == fsi) */
#define ASRC_NOM_PHASE_INCR_1_3         ( 0x5555 )      /* nominal phase increment == 1.0/3.0 in S32Q16, use for (fso == 3*fsi) */

#define ASRC_DEF_IN_FIFO_SZ             (5760) // CODEC_INPUT_BUFFER_SIZE  //( 256 )         /* default input FIFO size (one channel) */

#define ASRC_NUM_PHASES                 ( 32 )          /* number of phases in polyphase filter */
#define ASRC_NUM_TAPS_PER_PHASE         ( 7 )           /* number of coefficients per phase in polyphase filter */

#define ASRC_COEFS_16B_NORM             ( 2048 )        /* coefficient normalization constant */
#define ASRC_COEFS_32B_NORM             ( 134217728 )   /* coefficient normalization constant */
extern Int16 coefs16b[];
extern Int32 coefs32b[];


/**
 *  ASRC structure
 *
 *  Contains ASRC algorithm variables.
 *  Variables are read/written by ASRC during algorithm execution.
 */
typedef struct
{
    Uint16 inNumCh;             /* input stream number of channels: 0 => mono, 1 => stereo */
    Uint16 inFmt;               /* input stream format: 0 => split buffer, 1 => interleaved */
    Uint32 k0;                  /* loop filter gain K0 */
    Uint32 k1;                  /* loop filter gain K1 */
    Int32 z;                    /* loop filter accumulator */
    Int32 phaseErr;             /* phase error -- debug */
    Int32 nomPhaseIncr;         /* nominal phase increment */
    Int32 deltaPhaseIncr;       /* delta phase increment output from loop filter */
    Int32 phaseIncr;            /* nominal + delta; input to NCO */
    Int32 nco;                  /* NCO phase output */
    Int16 phaseLeakageCap;      /* phase leakage capacitor used for adaptive resampling */
    Uint16 inFifoSz;            /* input FIFO size (one channel) */
    Uint16 inFifoInIdx;         /* input FIFO input index */
    Uint16 inFifoHfThr;         /* input FIFO threshold */
    Uint16 numTapsPerPhase;     /* number of polyphase filter taps */
    Uint16 numPhases;           /* number polyphase filter phases */
    Int16 *coefs16b;            /* 16-bit polyphase filter coefficients; used for interpolation and adaptive modes */
    Int32 *coefs32b;            /* 32-bit polyphase filter coefficients; used for interpolation and adaptive modes */
    Uint32 coefNorm;            /* coefficient normalization constant */
    Int16 *inFifo;              /* input FIFO */
} ASRC_Obj;

typedef ASRC_Obj *ASRC_Handle;

extern ASRC_Handle hAsrc;

/**
 *  ASRC parameters structure
 *
 *  Contains parameters for ASRC algorithm initialization.
 *  Parameters are written by application before algorithm initialization.
 */
typedef struct
{
    Uint16 inNumCh;             /* input stream number of channels: 0 => mono, 1 => stereo */
    Uint16 inFmt;               /* input stream format: 0 => split buffer, 1 => interleaved */
    Uint16 nomTransSz;          /* nominal transaction size = (1 msec. * nominal input sampling rate) */
    Uint32 k0;                  /* loop filter gain K0, e.g. 1/20000 */
    Uint32 k1;                  /* loop filter gain K1, e.g. 1/80000 */
    Int32 nomPhaseIncr;         /* nominal phase increment; typically set to 1 */
    Uint16 inFifoSz;            /* input FIFO size (one channel) */
    Uint16 numTapsPerPhase;     /* number of polyphase filter taps, e.g. 7 */
    Uint16 numPhases;           /* number polyphase filter phases, e.g. 32 */
    Int16 *coefs16b;            /* 16-bit polyphase filter coefficients; used for interpolation and adaptive modes */
    Int32 *coefs32b;            /* 32-bit polyphase filter coefficients; used for interpolation and adaptive modes */
    Uint32 coefNorm;            /* coefficient normalization constant */
    Int16 *inFifo;              /* input FIFO */
} ASRC_Prms;


/** ===========================================================================
 *  ASRC_init
 *
 *  Description
 *      Initializes the ASRC algorithm referenced through the ASRC handle.
 *      Initialization parameters are written to the ASRC parameters structure.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pAsrcPrms       pointer to ASRC parameter structure
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters, e.g. null pointer to
 *      input FIFO
 *
 *  Pre Condition       None
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  ASRC_Obj object structure is initialized
 *
 *  Modifies
 *      1. The status variable
 *      2. ASRC_Obj object structure
 *
 *  ===========================================================================
 */
Int16 ASRC_init(
    ASRC_Handle hAsrc,      /* ASRC handle */
    ASRC_Prms *pAsrcPrms    /* pointer to ASRC parameters structure */
);

// NOTE: Output samples from polyphase filter are written to I2S in I2S ISR.
/** ===========================================================================
 *  ASRC_genOutInterpCoefs16b
 *
 *  Description
 *      Generates output for Interpolation mode using 16-bit filter
 *      coefficients.
 *      Function operates on sample-by-sample basis and must be called for
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pOut            pointer to L/R output samples
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *      Indicates failure in case of FIFO underflow
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  One output sample per channel is computed.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_genOutInterpCoefs16b(
    ASRC_Handle hAsrc,      /* ASRC handle */
    Int16 *pOut             /* output sample (L/R) */
);

// NOTE: Output samples from polyphase filter are written to I2S in I2S ISR.
/** ===========================================================================
 *  ASRC_genOutInterpCoefs32b
 *
 *  Description
 *      Generates output for Interpolation mode using 32-bit filter
 *      coefficients.
 *      Function operates on sample-by-sample basis and must be called for
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pOut            pointer to L/R output samples
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *      Indicates failure in case of FIFO underflow
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  One output sample per channel is computed.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_genOutInterpCoefs32b(
    ASRC_Handle hAsrc,      /* ASRC handle */
    Int16 *pOut             /* output sample (L/R) */
);

// NOTE: Output samples from polyphase filter are written to I2S in I2S ISR.
/** ===========================================================================
 *  ASRC_genOutAdaptCoefs16b
 *
 *  Description
 *      Generates output for Adaptive mode using 16-bit filter coefficients.
 *      Function operates on sample-by-sample basis and must be called for
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pOut            pointer to L/R output samples
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *      Indicates failure in case of FIFO underflow
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  One output sample per channel is computed.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_genOutAdaptCoefs16b(
    ASRC_Handle hAsrc,      /* ASRC handle */
    Int16 *pOut             /* output sample (L/R) */
);

// NOTE: Output samples from polyphase filter are written to I2S in I2S ISR.
/** ===========================================================================
 *  ASRC_genOutAdaptCoefs32b
 *
 *  Description
 *      Generates output for Adaptive mode using 32-bit filter coefficients.
 *      Function operates on sample-by-sample basis and must be called for
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pOut            pointer to L/R output samples
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *      Indicates failure in case of FIFO underflow
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  One output sample per channel is computed.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_genOutAdaptCoefs32b(
    ASRC_Handle hAsrc,      /* ASRC handle */
    Int16 *pOut             /* output sample (L/R) */
);

// NOTE: Output samples from polyphase filter are written to I2S in I2S ISR.
/** ===========================================================================
 *  ASRC_genOutNb
 *
 *  Description
 *      Generates output for Nearest Neighbor mode
 *      Function operates on sample-by-sample basis and must be called for
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pOut            pointer to L/R output samples
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *      Indicates failure in case of FIFO underflow
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  One output sample per channel is computed.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_genOutNb(
    ASRC_Handle hAsrc,      /* ASRC handle */
    Int16 *pOut             /* output sample (L/R) */
);

/** ===========================================================================
 *  ASRC_accPhase
 *
 *  Description
 *      Updates Phase Accumulator (Numerically Controlled Oscillator (NCO))
 *      for Interpolation and Nearest Neighbor modes.
 *      Function operates on sample-by-sample basis and must be called after
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Phase Accumulator is updated
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_accPhase(
    ASRC_Handle hAsrc       /* ASRC handle */
);

/** ===========================================================================
 *  ASRC_accPhaseAdapt
 *
 *  Description
 *      Updates Phase Accumulator (Numerically Controlled Oscillator (NCO))
 *      for Adaptive mode.
 *      Function operates on sample-by-sample basis and must be called after
 *      computation of each output sample.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Phase Accumulator is updated
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_accPhaseAdapt(
    ASRC_Handle hAsrc       /* ASRC handle */
);

// NOTE: Called once per USB input frame.
/** ===========================================================================
 *  ASRC_updatePhaseIncr
 *
 *  Description
 *      Updates phase increment input to Phase Accumulator using loop filter.
 *      Function operates on input data transaction basis.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *      Indicates failure in case of FIFO overflow
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Status is returned in the status variable.
 *      2.  Loop filter is updated.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_updatePhaseIncr(
    ASRC_Handle hAsrc       /* ASRC handle */
);

// NOTE: Called once per USB input frame after phase increment update.
/** ===========================================================================
 *  ASRC_updateInFifo
 *
 *  Description
 *      Updates input FIFO.
 *      Function operates on input data transaction basis.
 *
 *  Arguments
 *      hAsrc           ASRC handle
 *      pIStream        pointer to input stream data
 *      transSz         transaction size
 *
 *  Return Value        Int16
 *      Indicates failure in case of invalid parameters
 *
 *  Pre Condition
 *      ASRC_init() has been called.
 *
 *  Post Condition
 *      1.  Input FIFO is updated.
 *
 *  Modifies
 *      1. The status variable
 *
 *  ===========================================================================
 */
Int16 ASRC_updateInFifo(
    ASRC_Handle hAsrc,      /* ASRC handle */
    Int16 *pIStream,        /* pointer to input stream data */
    Uint16 transSz          /* transaction size (one channel) */
);


#endif /* _ASRC_FIX_H_ */
