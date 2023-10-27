
#include "FblCanTpCfg.h"

/*****************************************************************************
 *  Verify The Configurations of Function Macro
 *****************************************************************************/

/*****************************************************************************
 *  Internal Macro Definitions
 *****************************************************************************/
#define CANTP_MAKE_TIMEOUT(ms)  ((UINT16)((ms)/CANTP_SCHEDULE_PERIOD)) 

/*****************************************************************************
 *  Internal Type Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Internal Structure Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Internal Function Declarations
 *****************************************************************************/

/*****************************************************************************
 *  Internal Variable Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Global Variable Definitions
 *****************************************************************************/
 
/** \brief the configurations of the Rx channels.*/
const bl_CanTpChannelCfg_t g_CanTpRxChnsCfg[CANTP_NUMBER_OF_RX_CHANNEL] =
{
    /*Channel 0:Phsical & Standars addressing*/
    {
        CANTP_TYPE_STANDARD,
        CANTP_TATYPE_PHYSICAL,
        FBL_CAN_RX_ID_PHY,  /* RX ID */
        FBL_CAN_TX_ID_PHY,  /* TX ID */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_AR),     /* TIME A */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_BR),     /* TIME B */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_CR),    /* TIME C */
        0,  /* TA */
        STMIN_ECU,  /* STmin */
        BS_ECU,  /* BS */
        15u,  /* WFT */
    },
    /*Channel 1:functional & Standars addressing*/
    {
        CANTP_TYPE_STANDARD,
        CANTP_TATYPE_FUNCTIONAL,
        FBL_CAN_RX_ID_FUN,  /* RX ID */
        FBL_CAN_TX_ID_PHY,  /* TX ID */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_AR),     /* TIME A */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_BR),     /* TIME B */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_CR),    /* TIME C */
        0,  /* TA */
        STMIN_ECU,  /* STmin */
        BS_ECU,  /* BS */
        15u,  /* WFT */
    },
};

/** \brief the configurations of the Tx channels.*/
const bl_CanTpChannelCfg_t g_CanTpTxChnsCfg[CANTP_NUMBER_OF_TX_CHANNEL] =
{
    /*Channel 0:Phsical & Standars addressing*/
    {
        CANTP_TYPE_STANDARD,
        CANTP_TATYPE_PHYSICAL,
        FBL_CAN_RX_ID_PHY,  /* RX HANDLE */
        FBL_CAN_TX_ID_PHY,  /* TX HANDLE */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_AS),     /* TIME A */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_BS),    /* TIME B */
        CANTP_MAKE_TIMEOUT(TPL_TIMER_CS),     /* TIME C */
        0,  /* TA */
        STMIN_ECU,  /* STmin */
        BS_ECU,  /* BS */
        15u,  /* WFT */
    },
};

/*****************************************************************************
 *  Function Definitions
 *****************************************************************************/


