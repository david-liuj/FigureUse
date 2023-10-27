
#ifndef _FBL_CAN_TP_CFG_H_
#define _FBL_CAN_TP_CFG_H_
#include "FblCanTp.h"

/*****************************************************************************
 *  Macro Definitions
 *****************************************************************************/
/** \brief The schedule period of the cantp module.*/
#define CANTP_SCHEDULE_PERIOD           (2)

/** \brief The number of rx channels of the cantp module.*/
#define CANTP_NUMBER_OF_RX_CHANNEL      (2)
/** \brief The number of tx channels of the cantp module.*/
#define CANTP_NUMBER_OF_TX_CHANNEL      (1)

/** \brief The frame padding function.*/
#define CANTP_FUN_TX_FRAME_PADDING         ON
#define CANTP_FUN_RX_FRAME_PADDING         ON
/** \brief The frame padding value.*/
#define CANTP_FRAME_PADDING_VALUE       (CANTP_FILLER_BYTE)

/** \brief full duplex*/
#define CANTP_FULL_DUPLEX               (0)
/** \brief half duplex*/
#define CANTP_HALF_DUPLEX               (1)
/** \brief support full- or half-duplex communication*/
#define CANTP_COMMUNICATION_DUPLEX      CANTP_HALF_DUPLEX

/*****************************************************************************
 *  Structure Definitions
 *****************************************************************************/

/*****************************************************************************
 *  External Global Variable Declarations
 *****************************************************************************/
extern const bl_CanTpChannelCfg_t g_CanTpRxChnsCfg[CANTP_NUMBER_OF_RX_CHANNEL];

extern const bl_CanTpChannelCfg_t g_CanTpTxChnsCfg[CANTP_NUMBER_OF_TX_CHANNEL];

/*****************************************************************************
 *  External Function Prototype Declarations
 *****************************************************************************/

#endif

