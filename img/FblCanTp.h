/*************************************************************************************************************
*    FileName   :    FblCanTp.h
*    Description:    FblCanTp module header file. 

*    UpdateDate :    2022/3/7
*    Version    :    1.0.0
*    History    :         
        1. V1.0.0, 2022/3/7, R553, Initial version.

*************************************************************************************************************/
#ifndef _FBLCANTP_H_   
#define _FBLCANTP_H_

/*************************************************************************************************************
                                          Header File Includes
*************************************************************************************************************/
#include "typedef.h"
#include "FblConfig.h"


/*************************************************************************************************************
                                                Macros
*************************************************************************************************************/

/*****************************************************************************
 *  Macro Definitions
 *****************************************************************************/
#define CANTP_TYPE_STANDARD         (0u)
#define CANTP_TYPE_EXTENDED         (1u)
#define CANTP_TYPE_MIXED            (2u)
#define CANTP_TYPE_STANDARD_CANFD   (3u)
#define CANTP_TYPE_EXTENDED_CANFD   (4u)
#define CANTP_TYPE_MIXED_CANFD      (5u)

#define CANTP_TATYPE_PHYSICAL       (0u)
#define CANTP_TATYPE_FUNCTIONAL     (1u)

#if (ENABLE_CANFD == ON)
    #define CANTP_MAX_FRAME_SIZE    (0x40u)    /* CANFD frame max size*/

#else
    #define CANTP_MAX_FRAME_SIZE    (0x08u)    /* CAN frame max size*/
#endif

#define CANTP_CHNNEL_RX_CR          (g_CanTpRxChnsCfg[0].timerC)

#define GET_LOW_HALF(byte)          ((byte) & 0x0Fu)
#define GET_HIGH_HALF(byte)         ((byte) & 0xF0u)

#define CANTP_PHYSICAL_CHANNEL_RX   (0u)
#define CANTP_FUNCATION_CHANNEL_RX  (1u)
#define CANTP_PHYSICAL_CHANNEL_TX   (0u)


/*****************************************************************************
 *  Structure Definitions
 *****************************************************************************/
/** \brief The channel configuration informations for the CAN TP.*/
struct _tag_CanTpChannelCfg
{
    UINT8 type;   /**< The type of a tp channel.*/
    UINT8 taType; /**< The TA type of a tp channel.*/
    UINT16 rxId;  /**< RX ID from CanIf.*/
    UINT16 txId;  /**< TX ID to CanIf.*/
    UINT16 timerA;        /**< The timer A is used to send a can frame.*/
    UINT16 timerB;        /**< The timer B is used to wait for the FC frame.*/
    UINT16 timerC;        /**< The timer C is used to wait for the CF frame.*/
    UINT8 ta;     /**< The TA of a tp channel.*/
    UINT8 st;     /**< The STmin of a tp channel.*/
    UINT8 bs;     /**< The block size of a tp channel.*/
    UINT8 wft;    /**< The max wft of a tp channel.*/
};



/*****************************************************************************
 *  Type Declarations
 *****************************************************************************/
/** \brief A alias of the struct _tag_CanTpChannelCfg.*/
typedef struct _tag_CanTpChannelCfg bl_CanTpChannelCfg_t;
/** \brief The can tp handle.*/
typedef UINT16 bl_CanTpHandle_t;
/** \brief The Result of the Rx or Tx.*/
typedef UINT8 bl_CanTpResult_t;

typedef UINT16 bl_BufferSize_t;   /**< The size of a buffer.*/
typedef UINT8 bl_Buffer_t;        /**< The type for buffer.*/

/*****************************************************************************
 *  External Global Variable Declarations
 *****************************************************************************/

/*****************************************************************************
 *  External Function Prototype Declarations
 *****************************************************************************/
 
extern UINT8 FblCanTpColdInit(void);

extern UINT8 FblCanTpWarmInit(void);

extern void FblCanTpTask(UINT16 uwEventId,UINT8 *pucData);

/** \brief the period function of the can tp module*/
extern void Cantp_PeriodFunction(void);
/** \brief Transmit a data.*/
extern UINT8 Cantp_Transmit(bl_CanTpHandle_t handle,
                                    bl_BufferSize_t size);
/** \brief Indicate a frame to be received.*/
extern void Cantp_RxIndication(UINT16 handle,
                                bl_BufferSize_t size,
                                const bl_Buffer_t *buffer);
/** \brief Confire a frame to be transmitted.*/
extern void Cantp_TxConfirmation(UINT16 id);

/*************************************************************************************************************
                                               End Of File
*************************************************************************************************************/
#endif 

