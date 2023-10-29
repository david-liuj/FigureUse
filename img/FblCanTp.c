/*************************************************************************************************************
*    FileName   :    FblCanTp.c
*    Description:    CAN Transport Protocol Layer for FBL. 

*    UpdateDate :    2022/3/8
*    Version    :    1.0.0
*    History    :         
        1. V1.0.0, 2022/3/8, R515, Initial version.

*************************************************************************************************************/

/*************************************************************************************************************
                                             Include Files
*************************************************************************************************************/
#include "FblCanTp.h"
#include "FblCanTpCfg.h"
#include "FblUdsDiag.h"
#include "FblString.h"

#include "FblDrvApi.h"
#include "OsCoreCfg.h" 
#include "OsCore.h" 

/*************************************************************************************************************
                                     Local Function Declarations
 ************************************************************************************************************/
/** \brief The CTS used for the FS field in the Flow Control Frame.*/
#define CANTP_FC_FRAME_CTS              (0x00u)
/** \brief The WAIT used for the FS field in the Flow Control Frame.*/
#define CANTP_FC_FRAME_WAIT             (0x01u)
/** \brief The OVERFLOW used for the FS field in the Flow Control Frame.*/
#define CANTP_FC_FRAME_OVERFLOW         (0x02u)

/** \brief The number of the PCI used for different types of cantp channel.*/
#define CANTP_NUMBER_OF_PCI_INFO        (0x02u)
/** \brief This PCI used for standard cantp channel.*/
#define CANTP_STANDARD_PCI_INFO         (0x00u)
/** \brief This PCI used for extended or mixed cantp channel.*/
#define CANTP_EXTENDED_PCI_INFO         (0x01u)

/** \brief The number of status of the Rx channels.*/
#define CANTP_NUMBER_OF_RX_STATUS       (0x05u)
/** \brief The number of status of the Tx channels.*/
#define CANTP_NUMBER_OF_TX_STATUS       (0x05u)

/** \brief The number of frame types according to ISO 15765-2.*/
#define CANTP_NUMBER_OF_FRAME_TYPE      (0x04u)
/** \brief The Single Frame.*/
#define CANTP_FRAME_TYPE_SF             (0x00u)
/** \brief The First Frame.*/
#define CANTP_FRAME_TYPE_FF             (0x01u)
/** \brief The Consecutive Frame.*/
#define CANTP_FRAME_TYPE_CF             (0x02u)
/** \brief The Flow Control Frame.*/
#define CANTP_FRAME_TYPE_FC             (0x03u)

/** \brief The type value of the Single Frame.*/
#define CANTP_FRAME_SF_VALUE            (0x00u)
/** \brief The type value of the First Frame.*/
#define CANTP_FRAME_FF_VALUE            (0x10u)
/** \brief The type value of the Consecutive Frame.*/
#define CANTP_FRAME_CF_VALUE            (0x20u)
/** \brief The type value of the Flow Control Frame.*/
#define CANTP_FRAME_FC_VALUE            (0x30u)

/** \brief The 16bits type value of the First Frame.*/
#define CANTP_FRAME_FF_VAULE_16BITS     (0x1000u)

/** \brief The ta offset in a frame.*/
#define CANTP_TA_OFFSET                 (0x00u)
/** \brief The max STmin value.*/
#define CANTP_MAX_STMIN_VALUE           (0x7Fu)
#define CANTP_MIN_STMIN_VALUE_US        (0xF0u)
#define CANTP_MAX_STMIN_VALUE_US        (0xFAu)

/** \brief The Idle status of a channel.*/
#define CANTP_STATUS_IDLE               (0x00u)
/** \brief The recvSF status of a Rx channel.*/
#define CANTP_STATUS_RECEIVING_SF       (0x01u)
/** \brief The recvFF status of a Rx channel.*/
#define CANTP_STATUS_RECEIVING_FF       (0x02u)
/** \brief The recvCF status of a Rx channel.*/
#define CANTP_STATUS_RECEIVING_CF       (0x03u)
/** \brief The tranFC status of a Rx channel.*/
#define CANTP_STATUS_TRANSMITTING_FC    (0x04u)

/** \brief The tranSF status of a Tx channel.*/
#define CANTP_STATUS_TRANSMITTING_SF    (0x01u)
/** \brief The tranFF status of a Tx channel.*/
#define CANTP_STATUS_TRANSMITTING_FF    (0x02u)
/** \brief The tranCF status of a Tx channel.*/
#define CANTP_STATUS_TRANSMITTING_CF    (0x03u)
/** \brief The recvFC status of a Tx channel.*/
#define CANTP_STATUS_RECEIVING_FC       (0x04u)

/** \brief The sub Idle status of a channel.*/
#define CANTP_SUB_STATUS_IDLE           (0x00u)
/** \brief The sub transmitting status of a channel.*/
#define CANTP_SUB_STATUS_TRANSMITTING   (0x01u)
/** \brief The sub receiving status of a channel.*/
#define CANTP_SUB_STATUS_RECEIVING      (0x02u)

/** \brief The mask of data size in the First Frame.*/
#define CANTP_FRAME_FF_DATASIZE_MASK    (0x0FFFu)
/** \brief the mask of data size in the Single Frame.*/
#define CANTP_FRAME_SF_DATASIZE_MASK    (0x0Fu)

/** \brief Get the data size in the Single Frame.*/
#define CANTP_GET_SF_DATASIZE(data)     GET_LOW_HALF(data)
/** \brief Get the data size in the First Frame.*/
#define CANTP_GET_FF_DATASIZE(data,data2)  (((UINT16)(GET_LOW_HALF(data) << 8)| (UINT16)data2) \
                                                & CANTP_FRAME_FF_DATASIZE_MASK)
#define CANTP_GET_FF_DATASIZE2(data1,data2,data3,data4)  (((UINT32)((data1) << 24)) | ((UINT32)data2 << 16) \
                                                    | ((UINT32)data3 << 8)| ((UINT32)data4))

/** \brief Get the SN in the Consecutive Frame.*/
#define CANTP_GET_CF_SN(pci,buf)      GET_LOW_HALF((buf)[(pci)->pciPos])

/** \brief Get the frame type in the a frame.*/
#define CANTP_GET_FRAME_TYPE(pci,buf) GET_LOW_HALF((buf)[(pci)->pciPos]>> 4)
/** \brief Get the FlowStatus in the a Flow Control Frame.*/
#define CANTP_GET_FC_FS(pci,buf)      GET_LOW_HALF((buf)[(pci)->pciPos])
/** \brief Get the BlockSize in the a Flow Control Frame.*/
#define CANTP_GET_FC_BS(pci,buf)        ((buf)[(pci)->fcBsPos])
/** \brief Get the STmin in the a Flow Control Frame.*/
#define CANTP_GET_FC_STMIN(pci,buf)     ((buf)[(pci)->fcStPos])


/** \brief If the status of a channel is Idle return TRUE.*/
#define CANTP_STATUS_IS_IDLE(chn)       (CANTP_STATUS_IDLE == (chn)->status)
/** \brief If the status of a channel is recvSF return TRUE.*/
#define CANTP_STATUS_IS_RECVSF(chn)     (CANTP_STATUS_RECEIVING_SF \
                                            == (chn)->status)
/** \brief If the status of a channel is recvFF return TRUE.*/
#define CANTP_STATUS_IS_RECVFF(chn)     (CANTP_STATUS_RECEIVING_FF \
                                            == (chn)->status)
/** \brief If the status of a channel is recvCF return TRUE.*/
#define CANTP_STATUS_IS_RECVCF(chn)     (CANTP_STATUS_RECEIVING_CF \
                                            == (chn)->status)
/** \brief If the status of a channel is recvFC return TRUE.*/
#define CANTP_STATUS_IS_RECVFC(chn)     (CANTP_STATUS_RECEIVING_FC \
                                            == (chn)->status)
/** \brief If the status of a channel is tranSF return TRUE.*/
#define CANTP_STATUS_IS_TRANSF(chn)     (CANTP_STATUS_TRANSMITTING_SF \
                                            == (chn)->status)
/** \brief If the status of a channel is tranFF return TRUE.*/
#define CANTP_STATUS_IS_TRANFF(chn)     (CANTP_STATUS_TRANSMITTING_FF \
                                            == (chn)->status)
/** \brief If the status of a channel is tranCF return TRUE.*/
#define CANTP_STATUS_IS_TRANCF(chn)     (CANTP_STATUS_TRANSMITTING_CF \
                                            == (chn)->status)
/** \brief If the status of a channel is tranFC return TRUE.*/
#define CANTP_STATUS_IS_TRANFC(chn)     (CANTP_STATUS_TRANSMITTING_FC \
                                            == (chn)->status)

/*  When continuous FF or SF is received in one channel during
    a timeout period,It maybe break other physical channel.
    So If a channel is receiving a FF or a SF, do NOT indicate.*/
/** \brief Check if the buffer is gotten from Dcm module.*/
#define CANTP_IS_GETTING_BUFFER(chn)    ((chn)->status > \
                                            CANTP_STATUS_RECEIVING_FF)

/** \brief If the status of a channel is NOT Idle return TRUE.*/
#define CANTP_STATUS_IS_NOT_IDLE(chn)   (!CANTP_STATUS_IS_IDLE(chn))

/** \brief If the status of a channel is NOT recvSF return TRUE.*/
#define CANTP_STATUS_IS_NOT_RECVSF(chn) (!CANTP_STATUS_IS_RECVSF(chn))
/** \brief If the status of a channel is NOT recvFF return TRUE.*/
#define CANTP_STATUS_IS_NOT_RECVFF(chn) (!CANTP_STATUS_IS_RECVFF(chn))
/** \brief If the status of a channel is NOT recvCF return TRUE.*/
#define CANTP_STATUS_IS_NOT_RECVCF(chn) (!CANTP_STATUS_IS_RECVCF(chn))
/** \brief If the status of a channel is NOT recvFC return TRUE.*/
#define CANTP_STATUS_IS_NOT_RECVFC(chn) (!CANTP_STATUS_IS_RECVFC(chn))
/** \brief If the status of a channel is NOT tranSF return TRUE.*/
#define CANTP_STATUS_IS_NOT_TRANSF(chn) (!CANTP_STATUS_IS_TRANSF(chn))
/** \brief If the status of a channel is NOT tranFF return TRUE.*/
#define CANTP_STATUS_IS_NOT_TRANFF(chn) (!CANTP_STATUS_IS_TRANFF(chn))
/** \brief If the status of a channel is NOT tranCF return TRUE.*/
#define CANTP_STATUS_IS_NOT_TRANCF(chn) (!CANTP_STATUS_IS_TRANCF(chn))
/** \brief If the status of a channel is NOT tranFC return TRUE.*/
#define CANTP_STATUS_IS_NOT_TRANFC(chn) (!CANTP_STATUS_IS_TRANFC(chn))

/** \brief If the sub status of a channel is Idle return TRUE.*/
#define CANTP_SUB_STATUS_IS_IDLE(chn)   (CANTP_SUB_STATUS_IDLE \
                                            == (chn)->subStatus)
/** \brief If the sub status of a channel is Receiving return TRUE.*/
#define CANTP_SUB_STATUS_IS_RECV(chn)   (CANTP_SUB_STATUS_RECEIVING \
                                                == (chn)->subStatus)
/** \brief If the sub status of a channel is Transmitting return TRUE.*/
#define CANTP_SUB_STATUS_IS_TRAN(chn)   (CANTP_SUB_STATUS_TRANSMITTING \
                                                == (chn)->subStatus)

/** \brief If the sub status of a channel is NOT Idle return TRUE.*/
#define CANTP_SUB_STATUS_IS_NOT_IDLE(chn)   (!CANTP_SUB_STATUS_IS_IDLE(chn))
/** \brief If the sub status of a channel is NOT Receiving return TRUE.*/
#define CANTP_SUB_STATUS_IS_NOT_RECV(chn)   (!CANTP_SUB_STATUS_IS_RECV(chn))
/** \brief If the sub status of a channel is NOT Transmitting return TRUE.*/
#define CANTP_SUB_STATUS_IS_NOT_TRAN(chn)   (!CANTP_SUB_STATUS_IS_TRAN(chn))

/** \brief Set the status of a channel to Idle.*/
#define CANTP_STATUS_GOTO_IDLE(chn)     ((chn)->status = CANTP_STATUS_IDLE)
/** \brief Set the status of a channel to recvSF.*/
#define CANTP_STATUS_GOTO_RECVSF(chn)   ((chn)->status = \
                                            CANTP_STATUS_RECEIVING_SF)
/** \brief Set the status of a channel to recvFF.*/
#define CANTP_STATUS_GOTO_RECVFF(chn)   ((chn)->status = \
                                            CANTP_STATUS_RECEIVING_FF)
/** \brief Set the status of a channel to recvCF.*/
#define CANTP_STATUS_GOTO_RECVCF(chn)   ((chn)->status = \
                                            CANTP_STATUS_RECEIVING_CF)
/** \brief Set the status of a channel to recvFC.*/
#define CANTP_STATUS_GOTO_RECVFC(chn)   ((chn)->status = \
                                            CANTP_STATUS_RECEIVING_FC)

/** \brief Set the status of a channel to tranSF.*/
#define CANTP_STATUS_GOTO_TRANSF(chn)   ((chn)->status = \
                                            CANTP_STATUS_TRANSMITTING_SF)
/** \brief Set the status of a channel to tranFF.*/
#define CANTP_STATUS_GOTO_TRANFF(chn)   ((chn)->status = \
                                            CANTP_STATUS_TRANSMITTING_FF)
/** \brief Set the status of a channel to tranCF.*/
#define CANTP_STATUS_GOTO_TRANCF(chn)   ((chn)->status = \
                                            CANTP_STATUS_TRANSMITTING_CF)
/** \brief Set the status of a channel to tranFC.*/
#define CANTP_STATUS_GOTO_TRANFC(chn)   ((chn)->status = \
                                            CANTP_STATUS_TRANSMITTING_FC)

/** \brief Set the sub status of a channel to Idle.*/
#define CANTP_SUB_STATUS_GOTO_IDLE(chn) ((chn)->subStatus = \
                                            CANTP_SUB_STATUS_IDLE)

/** \brief Set the sub status of a channel to receiving.*/
#define CANTP_SUB_STATUS_GOTO_RECV(chn) ((chn)->subStatus = \
                                            CANTP_SUB_STATUS_RECEIVING)

/** \brief Set the sub status of a channel to transmitting.*/
#define CANTP_SUB_STATUS_GOTO_TRAN(chn) ((chn)->subStatus = \
                                            CANTP_SUB_STATUS_TRANSMITTING)

/** \brief Set the private data of a channel.*/
#define CANTP_SET_PRIVATE_DATA(chn,_pData) ((chn)->pData = (UINT8)(_pData))

/** \brief Set the BS of a channel.*/
#define CANTP_SET_BS(chn,_bs)       ((chn)->bs = (UINT8)(_bs))
/** \brief Set the STmin of a channel.*/
#define CANTP_SET_STMIN(chn,_st)    ((chn)->st = (UINT8)(_st))
/** \brief Set the maxWFT of a channel.*/
#define CANTP_SET_MAXWFT(chn,_wft)  ((chn)->wft = (UINT8)(_wft))
/** \brief Set the TA type of a channel.*/
#define CANTP_SET_TATYPE(chn,type)  ((chn)->taType = (UINT8)(type))

/** \brief Initialize the BS of a channel by channel configurations.*/
#define CANTP_INIT_BS_BY_CFG(chn,cfg)       CANTP_SET_BS(chn,(cfg)->bs)
/** \brief Initialize the STmin of a channel by channel configurations.*/
#define CANTP_INIT_STMIN_BY_CFG(chn,cfg)    CANTP_SET_STMIN(chn,(cfg)->st)
/** \brief Initialize the maxWFT of a channel by channel configurations.*/
#define CANTP_INIT_MAXWFT_BY_CFG(chn,cfg)   CANTP_SET_MAXWFT(chn,(cfg)->wft)
/** \brief Initialize the TA type of a channel by channel configurations.*/
#define CANTP_INIT_TATYPE_BY_CFG(chn,cfg)   CANTP_SET_TATYPE(chn,(cfg)->taType)

/** \brief Initialize the BS of a channel.*/
#define CANTP_INIT_BS(chn)      CANTP_INIT_BS_BY_CFG((chn),(chn)->chnCfg)
/** \brief Initialize the STmin of a channel.*/
#define CANTP_INIT_STMIN(chn)   CANTP_INIT_STMIN_BY_CFG((chn),(chn)->chnCfg)
/** \brief Initialize the maxWFT of a channel.*/
#define CANTP_INIT_MAXWFT(chn)  CANTP_INIT_MAXWFT_BY_CFG((chn),(chn)->chnCfg)
/** \brief Initialize the TA type of a channel.*/
#define CANTP_INIT_TATYPE(chn)  CANTP_INIT_TATYPE_BY_CFG((chn),(chn)->chnCfg)

/** \brief Set the timeout value of the timer in a channel.*/
#define CANTP_SET_TIMER(chn,timeout)    ((chn)->timer = (UINT16)(timeout))
/** \brief Get the current value of the timer in a channel.*/
#define CANTP_GET_TIMER(chn)            ((chn)->timer)

/** \brief Initialize the timer in a channel.*/
#define CANTP_INIT_TIMER(chn)           CANTP_SET_TIMER(chn,0u)
/** \brief Use the timerA to initialize the timer in a channel.*/
#define CANTP_INIT_TIMER_A(chn) CANTP_SET_TIMER(chn,(chn)->chnCfg->timerA)
/** \brief Use the timerB to initialize the timer in a channel.*/
#define CANTP_INIT_TIMER_B(chn) CANTP_SET_TIMER(chn,(chn)->chnCfg->timerB)
/** \brief Use the timerC to initialize the timer in a channel.*/
#define CANTP_INIT_TIMER_C(chn) CANTP_SET_TIMER(chn,(chn)->chnCfg->timerC)

/** \brief Check if the timer in a channel is timeout.*/
#define CANTP_IS_TIMEOUT(chn)           (0u == CANTP_GET_TIMER(chn))

/** \brief Set the SN of a channel.*/
#define CANTP_SET_SN(chn,_sn)       ((chn)->sn = (UINT8)(_sn))
/** \brief Set the CF counter of a channel.*/
#define CANTP_SET_CFCNT(chn,_cfCnt) ((chn)->cfCnt = (UINT16)(_cfCnt))
/** \brief Set the size of the last CF of a channel.*/
#define CANTP_SET_LAST_SIZE(chn,_lastSize)  ((chn)->lastSize =\
                                                (UINT8)(_lastSize))
/** \brief Set the total size of a channel.*/
#define CANTP_SET_TOTAL_SIZE(chn,_totalSize)    ((chn)->totalSize =\
                                                 (bl_BufferSize_t)(_totalSize))

/** \brief Set the tx delay of a channel.*/
#define CANTP_SET_TXDELAY(chn,delay)    ((chn)->txDelay = (UINT8)(delay))

/** \brief Initialize the SN of a channel.*/
#define CANTP_INIT_SN(chn)          CANTP_SET_SN(chn,0u)
/** \brief Initialize the CF counter of a channel.*/
#define CANTP_INIT_CFCNT(chn)       CANTP_SET_CFCNT(chn,0u)
/** \brief Initialize the size of the last CF of a channel.*/
#define CANTP_INIT_LAST_SIZE(chn)   CANTP_SET_LAST_SIZE(chn,0u)
/** \brief Initialize the total size of a channel.*/
#define CANTP_INIT_TOTAL_SIZE(chn)  CANTP_SET_TOTAL_SIZE(chn,0u)
/** \brief Initialize the tx delay of a channel.*/
#define CANTP_INIT_TXDELAY(chn)     CANTP_SET_TXDELAY(chn,(chn)->st)
/** \brief Add the SN of a channel.*/
#define CANTP_ADD_SN(chn)            ((chn)->sn += 1)
/** \brief Check if a channel is functional.*/
#define CANTP_IS_FUNCTIONAL_CHANNEL(chn)    (CANTP_TATYPE_FUNCTIONAL \
                                                == (chn)->chnCfg->taType)
/** \brief Check if a channel is physical.*/
#define CANTP_IS_PHYSICAL_CHANNEL(chn)      (CANTP_TATYPE_PHYSICAL \
                                                == (chn)->chnCfg->taType)

/*****************************************************************************
 *  Internal Type Definitions
 *****************************************************************************/
/** \brief A alias of the struct _tag_CanTpChannel.*/
typedef struct _tag_CanTpChannel bl_CanTpChannel_t;
/** \brief A alias of the struct _tag_CanTpPciInfo.*/
typedef struct _tag_CanTpPciInfo bl_CanTpPciInfo_t;
/** \brief A alias of the struct _tag_CanTpPeriodInterface.*/
typedef struct _tag_CanTpPeriodInterface bl_CanTpPeriodIF_t;
/** \brief A interface used to process received frame.*/
typedef UINT8 (*bl_CanTpRxProcess_t)(bl_CanTpChannel_t *channel,
                                                bl_BufferSize_t size,
                                                const bl_Buffer_t *buffer);
/** \brief A interface used to confirm that a frame is transmitted.*/
typedef void (*bl_CanTpTxConfirm_t)(bl_CanTpChannel_t *channel);
/*****************************************************************************
 *  Internal Structure Definitions
 *****************************************************************************/
/** \brief The PCI Informations for a can tp channel.*/
struct _tag_CanTpPciInfo
{
    UINT8 pciPos;     /**< The PCI position in a can frame.*/
    UINT8 dataPos;    /**< The valid data position in a SF or CF frame.*/
    UINT8 ffDataPos;  /**< The valid data position in a FF frame.*/
    UINT8 fcBsPos;    /**< The BS position in a FC frame.*/
    UINT8 fcStPos;    /**< The STmin position in a FC frame.*/
    UINT8 maxDataSize;/**< The max size of data in a SF or CF frame.*/
    UINT8 maxFFDataSize;  /**< The max size of data in a FF frame.*/
    UINT8 maxFCDataSize;  /**< The max size of data in a FC frame.*/
};

/** \brief The channel of the CAN TP.*/
struct _tag_CanTpChannel
{
    UINT8 status;     /**< The current status of a channel.*/
    UINT8 subStatus;  /**< The current sub status of a channel.*/
    UINT8 pData;      /**< The private data of a channel.*/
    UINT8 sn;         /**< The expected SN used to receive CF frames.*/
    UINT8 txDelay;    /**< The Delay timer for sending a CF frame.*/
    UINT8 st;         /**< The STmin copy from chnCfg->st or a FC frame.*/
    UINT8 bs;         /**< The BS copy from chnCfg->bs or a FC frame.*/
    UINT8 wft;        /**< The WFT copy from chnCfg->wft.*/
    UINT8 taType;      /**< The TA type copy from chnCfg->taType.*/
    UINT8 lastSize;   /**< The Size of the last CF or SF frame.*/
    bl_Buffer_t frame[CANTP_MAX_FRAME_SIZE];  /**< The local frame buffer.*/
    UINT16 timer;     /**< The timer*/
    UINT16 cfCnt;     /**< The counter of CF frames.*/
    bl_BufferSize_t totalSize; /**< The total size of Tx or Rx.*/
    const struct _tag_CanTpChannelCfg *chnCfg; /**< Channel configurations*/
    const struct _tag_CanTpPciInfo *pciInfo;   /**< PCI information*/
};

/** \brief The period process interface of the CAN TP channel.*/
struct _tag_CanTpPeriodInterface
{
    void (*Period)(bl_CanTpChannel_t *channel); /**< period of status.*/
    UINT8 (*Timeout)(bl_CanTpChannel_t *channel);/**< timeout of status*/
};

/*****************************************************************************
 *  Internal Function Declarations
 *****************************************************************************/
static void FblCanTpMsgHandle(UINT8 *pucData);
static void FblCanTpEventHandle(UINT16 uwEventId);
static void CanTp_Init(void);
/** \brief Use the configuration to initialize a cantp channel.*/
static void _Cantp_InitChannel(bl_CanTpChannel_t *channel,
                                const bl_CanTpChannelCfg_t *channelCfg);
/** \brief Use the Rx handle of the canif module to get a channel.*/
static bl_CanTpChannel_t * _Cantp_GetChannelByRxId(UINT16 id,
                                                   UINT16 chnNum,
                                                   bl_CanTpChannel_t *chnList);
/** \brief Use the size to set the CF counter and last size of a channel.*/
static void _Cantp_SetMultipleFrameSize(bl_CanTpChannel_t *channel,
                                        bl_BufferSize_t size);

/** \brief Get the STmin to used to transmit CF.*/
static UINT8 _Cantp_GetSTMinFromFC(UINT8 st);

/** \brief The period function.*/
static void _Cantp_PeriodFunction(UINT16 num,
                                bl_CanTpChannel_t *channelList,
                                const bl_CanTpPeriodIF_t *periodList);
/** \brief The Rx indication function used by Rx channel.*/
static UINT8 _Cantp_RxIndToRxChannel(bl_CanTpChannel_t *channel,
                                            bl_BufferSize_t size,
                                            const bl_Buffer_t *buffer);

/** \brief The Rx indication function used by Tx channel.*/
static UINT8 _Cantp_RxIndToTxChannel(bl_CanTpChannel_t *channel,
                                            bl_BufferSize_t size,
                                            const bl_Buffer_t *buffer);

/** \brief Transmit a CF.*/
static void _Cantp_TransmitCF(bl_CanTpChannel_t *channel);
/** \brief Make the PCI of SF into the local frame.*/
static void _Cantp_MakePciOfSF(bl_CanTpChannel_t *channel);
/** \brief Make the PCI of FF into the local frame.*/
static void _Cantp_MakePciOfFF(bl_CanTpChannel_t *channel);
/** \brief Make the PCI of CF into the local frame.*/
static void _Cantp_MakePciOfCF(bl_CanTpChannel_t *channel);
/** \brief Make the PCI of FC into the local frame.*/
static void _Cantp_MakePciOfFC(bl_CanTpChannel_t *channel);
/** \brief The Tx confirmation function used for the Idle status of a channel.*/
static void _Cantp_TxConfirmIdle(bl_CanTpChannel_t *channel);
/** \brief The Tx confirmation function used for the tranSF status of a channel.*/
static void _Cantp_TxConfirmSF(bl_CanTpChannel_t *channel);
/** \brief The Tx confirmation function used for the tranFF status of a channel.*/
static void _Cantp_TxConfirmFF(bl_CanTpChannel_t *channel);
/** \brief The Tx confirmation function used for the tranCF status of a channel.*/
static void _Cantp_TxConfirmCF(bl_CanTpChannel_t *channel);
/** \brief The Tx confirmation function used for the tranFC status of a channel.*/
static void _Cantp_TxConfirmFC(bl_CanTpChannel_t *channel);
/** \brief The Change the status of a channel to Idle.*/
static void _Cantp_GotoIdle(bl_CanTpChannel_t *channel);
/** \brief The Change the status of a channel to TranSF.*/
static void _Cantp_GotoTranSF(bl_CanTpChannel_t *channel);
/** \brief The Change the status of a channel to TranFF.*/
static void _Cantp_GotoTranFF(bl_CanTpChannel_t *channel);
/** \brief The Change the status of a channel to TranCF.*/
static void _Cantp_GotoTranCF(bl_CanTpChannel_t *channel);
/** \brief The Change the status of a channel to TranFC.*/
static void _Cantp_GotoTranFC(bl_CanTpChannel_t *channel, UINT8 fs);
/** \brief Receive a Single Frame.*/
static UINT8 _Cantp_ReceiveSF(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer);
/** \brief Receive a First Frame.*/
static UINT8 _Cantp_ReceiveFF(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer);
/** \brief Receive a Consecutive Frame.*/
static UINT8 _Cantp_ReceiveCF(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer);
/** \brief Receive a Flow Control Frame.*/
static UINT8 _Cantp_ReceiveFC(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer);

/** \brief The period function of the Idle status of a channel.*/
static void _Cantp_PeriodIdle(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the Idle status of a channel.*/
static UINT8 _Cantp_TimeoutIdle(bl_CanTpChannel_t *channel);
/** \brief The period function of the RecvSF status of a channel.*/
static void _Cantp_PeriodRecvSF(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the RecvSF status of a channel.*/
static UINT8 _Cantp_TimeoutRecvSF(bl_CanTpChannel_t *channel);
/** \brief The period function of the RecvFF status of a channel.*/
static void _Cantp_PeriodRecvFF(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the RecvFF status of a channel.*/
static UINT8 _Cantp_TimeoutRecvFF(bl_CanTpChannel_t *channel);
/** \brief The period function of the RecvCF status of a channel.*/
static void _Cantp_PeriodRecvCF(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the RecvCF status of a channel.*/
static UINT8 _Cantp_TimeoutRecvCF(bl_CanTpChannel_t *channel);
/** \brief The period function of the TranFC status of a channel.*/
static void _Cantp_PeriodTranFC(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the TranFC status of a channel.*/
static UINT8 _Cantp_TimeoutTranFC(bl_CanTpChannel_t *channel);
/** \brief The period function of the TranSF status of a channel.*/
static void _Cantp_PeriodTranSF(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the TranSF status of a channel.*/
static UINT8 _Cantp_TimeoutTranSF(bl_CanTpChannel_t *channel);
/** \brief The period function of the TranFF status of a channel.*/
static void _Cantp_PeriodTranFF(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the TranFF status of a channel.*/
static UINT8 _Cantp_TimeoutTranFF(bl_CanTpChannel_t *channel);
/** \brief The period function of the TranCF status of a channel.*/
static void _Cantp_PeriodTranCF(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the TranCF status of a channel.*/
static UINT8 _Cantp_TimeoutTranCF(bl_CanTpChannel_t *channel);
/** \brief The period function of the RecvFC status of a channel.*/
static void _Cantp_PeriodRecvFC(bl_CanTpChannel_t *channel);
/** \brief The timeout function of the RecvFC status of a channel.*/
static UINT8 _Cantp_TimeoutRecvFC(bl_CanTpChannel_t *channel);

/*****************************************************************************
 *  Internal Variable Definitions
 *****************************************************************************/
/** \brief The PCI used to standard, extanded or mixed channel.*/
static const bl_CanTpPciInfo_t gs_CanTpPciInfo[CANTP_NUMBER_OF_PCI_INFO] =
{
    /*The channel type is CANTP_TYPE_STANDARD.*/
    {
        0,1,2,1,2,
#if (ENABLE_CANFD == ON)
        63,/***最大的SF和CF的数据大小***/
        62,/***最大的FF的数据大小***/
        3 /***最大的FC的数据大小***/
#else
        7,
        6,
        3
#endif
    },
    /*The channel type is CANTP_TYPE_EXTENDED or CANTP_TYPE_MIXED.*/
    {
        1,2,3,2,3,6,5,4
    },
};

/** \brief The period schedule interface of the Rx channel.*/
static const bl_CanTpPeriodIF_t gs_RxPeriodList[CANTP_NUMBER_OF_RX_STATUS] =
{
    {
        &_Cantp_PeriodIdle,&_Cantp_TimeoutIdle
    },
    {
        &_Cantp_PeriodRecvSF,&_Cantp_TimeoutRecvSF
    },
    {
        &_Cantp_PeriodRecvFF,&_Cantp_TimeoutRecvFF
    },
    {
        &_Cantp_PeriodRecvCF,&_Cantp_TimeoutRecvCF
    },
    {
        &_Cantp_PeriodTranFC,&_Cantp_TimeoutTranFC
    },
};

/** \brief The period schedule interface of the Tx channel.*/
static const bl_CanTpPeriodIF_t gs_TxPeriodList[CANTP_NUMBER_OF_TX_STATUS] =
{
    {
        &_Cantp_PeriodIdle,&_Cantp_TimeoutIdle
    },
    {
        &_Cantp_PeriodTranSF,&_Cantp_TimeoutTranSF
    },
    {
        &_Cantp_PeriodTranFF,&_Cantp_TimeoutTranFF
    },
    {
        &_Cantp_PeriodTranCF,&_Cantp_TimeoutTranCF
    },
    {
        &_Cantp_PeriodRecvFC,&_Cantp_TimeoutRecvFC
    },
};
/** \brief The Tx confirmation interface of a channel.*/
static const bl_CanTpTxConfirm_t gs_TxConfirmList[CANTP_NUMBER_OF_TX_STATUS] =
{
    &_Cantp_TxConfirmIdle,
    &_Cantp_TxConfirmSF,
    &_Cantp_TxConfirmFF,
    &_Cantp_TxConfirmCF,
    &_Cantp_TxConfirmFC
};

/** \brief The Rx process interface of a channel.*/
static const bl_CanTpRxProcess_t gs_RxProcessList[CANTP_NUMBER_OF_FRAME_TYPE] =
{
    &_Cantp_ReceiveSF,
    &_Cantp_ReceiveFF,
    &_Cantp_ReceiveCF,
    &_Cantp_ReceiveFC
};



/** \brief The Rx channel list.*/
static bl_CanTpChannel_t gs_CanTpRxChannel[CANTP_NUMBER_OF_RX_CHANNEL];
/** \brief The Tx channel list.*/
static bl_CanTpChannel_t gs_CanTpTxChannel[CANTP_NUMBER_OF_TX_CHANNEL];

/** \brief The transmitting channel list.*/
static bl_CanTpChannel_t *gs_TransmittingChannel[CANTP_NUMBER_OF_TX_CHANNEL];

/*************************************************************************************************************
                                          Function Definitions
 ************************************************************************************************************/
/*************************************************************************************************************
* Function Name  : FblCanTpColdInit
* Description    : 
* Input          : None
* Output         : None
* Return         : TRUE/FALSE
*************************************************************************************************************/
UINT8 FblCanTpColdInit(void)
{       
    //FblTpInit();
    CanTp_Init();
    return INIT_SUCCESS;
}

/*************************************************************************************************************
* Function Name  : FblCanTpWarmInit
* Description    : 
* Input          : None
* Output         : None
* Return         : TRUE/FALSE
*************************************************************************************************************/
UINT8 FblCanTpWarmInit(void)
{
    //FblTpInit();
    CanTp_Init();
    return INIT_SUCCESS;
}

/*************************************************************************************************************
* Function Name  : FblCanTpTask
* Description    : 
* Input          : uwEventId
                 : pucData
* Output         : None
* Return         : None
*************************************************************************************************************/
void FblCanTpTask(UINT16 uwEventId,UINT8 *pucData)
{
    /*Parameter validity check*/
    RETURN_IF_FAIL(NULL != pucData);
    
    if(uwEventId & EVENT_MSG_READY){
        FblCanTpMsgHandle(pucData);
    } 

    if(uwEventId & EVENT_READY){
        FblCanTpEventHandle(uwEventId); 
    } 
}

/*************************************************************************************************************
* Function Name  : FblCanTpMsgHandle
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*************************************************************************************************************/
static void FblCanTpMsgHandle(UINT8 *pucData)
{
    
}

/*************************************************************************************************************
* Function Name  : FblCanTpEventHandle
* Description    : 
* Input          : uwEventId
* Output         : None
* Return         : None
*************************************************************************************************************/
static void FblCanTpEventHandle(UINT16 uwEventId)
{
    if(uwEventId & EVENT_SCAN_TIMER)
    {
        Cantp_PeriodFunction();
    }
}

/**************************************************************************//**
 *
 *  \details    Initialize the can TP module.
 *
 *  \return None.
 *
 *  \since  V1.0.0
 *
 *****************************************************************************/
static void CanTp_Init(void)
{
    UINT8 handle;
    bl_CanTpChannel_t *channel;
    const bl_CanTpChannelCfg_t *channelCfg;

    /*Initialize the RX channels*/
    for (handle = 0; handle < CANTP_NUMBER_OF_RX_CHANNEL; handle++)
    {
        channel = &gs_CanTpRxChannel[handle];
        channelCfg = &g_CanTpRxChnsCfg[handle];
        _Cantp_InitChannel(channel, channelCfg);
    }

    /*Initialize the TX channels*/
    for (handle = 0; handle < CANTP_NUMBER_OF_TX_CHANNEL; handle++)
    {
        channel = &gs_CanTpTxChannel[handle];
        channelCfg = &g_CanTpTxChnsCfg[handle];
        _Cantp_InitChannel(channel, channelCfg);
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details Transmit the data.
 *
 *  \param[in]  handle - Tx handle.
 *  \param[in]  size - the size of the data.
 *
 *  \return None.
 *
 *  \since  V2.0.0
 *
 *****************************************************************************/
UINT8 Cantp_Transmit(bl_CanTpHandle_t handle, bl_BufferSize_t size)
{
    UINT8  ret = ERR_ERROR;
    bl_CanTpChannel_t *channel;
    const bl_CanTpPciInfo_t *pci;

    BL_DEBUG_ASSERT_PARAM(handle < CANTP_NUMBER_OF_TX_CHANNEL);
    BL_DEBUG_ASSERT_PARAM(size != 0);

    channel = &gs_CanTpTxChannel[handle];
    
    if(CANTP_STATUS_IS_IDLE(channel))
    {
        pci = channel->pciInfo;

        if (size > (pci->maxDataSize))
        {
            if (CANTP_IS_PHYSICAL_CHANNEL(channel))
            {
                _Cantp_SetMultipleFrameSize(channel,size);
                _Cantp_GotoTranFF(channel);

                ret = ERR_OK;
            }
        }
        else
        {
            channel->lastSize = (UINT8)size;
            _Cantp_GotoTranSF(channel);

            ret = ERR_OK;
        }
    }

    return ret;
}

/**************************************************************************//**
 *
 *  \details    If a tx channel wait for receiving a FC frame, The tx channel
 *              process this frame. otherwise the rx channel process it.
 *
 *  \param[in]  handle - rx handle of the canif.
 *  \param[in]  size - the size of the data.
 *  \param[in]  buffer - the content of the data.
 *
 *  \return None.
 *
 *  \since  V2.0.0
 *
 *****************************************************************************/
void Cantp_RxIndication(UINT16 id,
                            bl_BufferSize_t size,
                            const bl_Buffer_t *buffer)
{
    bl_CanTpChannel_t *channel;
    UINT8 ret;

    /*Check whether the parameters are valid.*/
#if (CANTP_FUN_RX_FRAME_PADDING == ON)
    if ((CANTP_MAX_FRAME_SIZE >= size) && (buffer != NULL_PTR))
#else
    if ((size > 0) && (buffer != NULL_PTR))
#endif
    {
        channel = _Cantp_GetChannelByRxId(id,
                                            CANTP_NUMBER_OF_TX_CHANNEL,
                                            gs_CanTpTxChannel);
        ret = _Cantp_RxIndToTxChannel(channel,size,buffer);/***仅接收流控帧***/
        if (ret != ERR_OK)   /*the Tx channel is not process this frame.*/
        {
            channel = _Cantp_GetChannelByRxId(id,
                                                CANTP_NUMBER_OF_RX_CHANNEL,
                                                gs_CanTpRxChannel);
            (void)_Cantp_RxIndToRxChannel(channel,size,buffer);/***接收除流控帧外的其他帧***/

        }
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    If a channel wait for a confirmation, call the confirmation
 *              function of this channel.
 *
 *  \param[in]  handle - tx handle of the canif.
 *
 *  \return None.
 *
 *  \since  V2.0.0
 *
 *****************************************************************************/
void Cantp_TxConfirmation(UINT16 id)
{
    bl_CanTpChannel_t *channel;
    bl_CanTpTxConfirm_t confirm;
    UINT16 i;

    for (i = 0; i < CANTP_NUMBER_OF_TX_CHANNEL; i++)
    {
        channel = gs_TransmittingChannel[i];
        if ((channel != NULL_PTR) && (channel->chnCfg->txId == id))
        {
            BL_DEBUG_ASSERT_NO_RET(channel->status<CANTP_NUMBER_OF_TX_STATUS);

            confirm = gs_TxConfirmList[channel->status];

            confirm(channel);

            gs_TransmittingChannel[i] = NULL_PTR;

            break;
        }
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    When the timer of a channel is not timeout, call the period
 *              function. If the timer is timeout, call the timeout function.
 *
 *  \return None.
 *
 *  \since  V2.0.0
 *
 *****************************************************************************/
void Cantp_PeriodFunction(void)
{
    _Cantp_PeriodFunction(CANTP_NUMBER_OF_RX_CHANNEL,
                            gs_CanTpRxChannel,
                            gs_RxPeriodList);

    _Cantp_PeriodFunction(CANTP_NUMBER_OF_TX_CHANNEL,
                            gs_CanTpTxChannel,
                            gs_TxPeriodList);
}

/**************************************************************************//**
 *
 *  \details Initialize the channel of the cantp module.
 *
 *  \param[in]  channel     - the pointer of a cantp channel.
 *  \param[in]  channelCfg  - the pointer of a cantp channel cfg in ROM.
 *
 *  \return None.
 *
 *  \since  V2.0.0
 *
 *****************************************************************************/
static void _Cantp_InitChannel(bl_CanTpChannel_t *channel,
                                const bl_CanTpChannelCfg_t *channelCfg)
{
    BL_DEBUG_ASSERT_NO_RET(channel != NULL_PTR);
    BL_DEBUG_ASSERT_NO_RET(channelCfg != NULL_PTR);
    
    CANTP_STATUS_GOTO_IDLE(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);

    CANTP_INIT_STMIN_BY_CFG(channel, channelCfg);
    CANTP_INIT_BS_BY_CFG(channel, channelCfg);
    CANTP_INIT_MAXWFT_BY_CFG(channel, channelCfg);
    CANTP_INIT_TATYPE_BY_CFG(channel, channelCfg);

    CANTP_INIT_TIMER(channel);

    if (CANTP_TYPE_STANDARD == channelCfg->type)
    {
        channel->pciInfo = &gs_CanTpPciInfo[CANTP_STANDARD_PCI_INFO];
    }
    else
    {
        channel->pciInfo = &gs_CanTpPciInfo[CANTP_EXTENDED_PCI_INFO];
    }

    channel->chnCfg = channelCfg;
    return ;
}

/**************************************************************************//**
 *
 *  \details Get the cantp channel by the receiving handle.
 *
 *  \param[in]  handle  - the receiving handle of the canif moudle.
 *  \param[in]  chnNum  - the number of channels.
 *  \param[in]  chnList - the channels
 *
 *  \return If the rxId of a channel equal to the handle then return a pointer
 *          of the channel, otherwise returns NULL_PTR.
 *
 *  \since  V2.0.0
 *
 *****************************************************************************/
static bl_CanTpChannel_t * _Cantp_GetChannelByRxId(UINT16 id,
                                                    UINT16 chnNum,
                                                    bl_CanTpChannel_t *chnList)
{
    bl_CanTpChannel_t *channel = NULL_PTR;
    UINT16 i;

    for (i = 0; i < chnNum; i++)
    {
        if (chnList[i].chnCfg->rxId == id)
        {
            channel = &chnList[i];
            break;
        }
    }

    return channel;
}

/**************************************************************************//**
 *
 *  \details calculate the CF count and the size of last CF by the total size
 *           of transmitting or receiving data.
 *
 *  \param[in]  channel - the pointer of a cantp channel.
 *  \param[in]  size    - the total size of transmitting or receiving data.
 *
 *  \return None.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_SetMultipleFrameSize(bl_CanTpChannel_t *channel,
                                        bl_BufferSize_t size)
{
    bl_BufferSize_t tmpSize;
    UINT16 tmpCnt;
    const bl_CanTpPciInfo_t *pci = channel->pciInfo;

    BL_DEBUG_ASSERT_NO_RET(size > pci->maxDataSize);
    BL_DEBUG_ASSERT_NO_RET(pci->maxDataSize != 0);

    tmpCnt = (size / ((bl_BufferSize_t)pci->maxDataSize));

    tmpSize = (size % ((bl_BufferSize_t)pci->maxDataSize)) + 1u;

    CANTP_SET_TOTAL_SIZE(channel,size);
    CANTP_SET_LAST_SIZE(channel,tmpSize);
    CANTP_SET_CFCNT(channel,tmpCnt);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Calculate the STmin according to ISO 15765-2
 *
 *  \param[in]  st - The STmin is received from the tester.
 *
 *  \return the STmin used by cantp channel.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_GetSTMinFromFC(UINT8 st)
{
    UINT8 retSTMin;

    if (st > CANTP_MAX_STMIN_VALUE)
    {
        if ((st > CANTP_MIN_STMIN_VALUE_US) && (st < CANTP_MAX_STMIN_VALUE_US))
        {
            retSTMin = CANTP_SCHEDULE_PERIOD;
        }
        else
        {
#if (CANTP_SCHEDULE_PERIOD == 1)
            retSTMin = CANTP_MAX_STMIN_VALUE;
#else
            retSTMin = (CANTP_MAX_STMIN_VALUE / CANTP_SCHEDULE_PERIOD) + 1;
#endif
        }

    }
    else
    {
#if (CANTP_SCHEDULE_PERIOD == 1)
        retSTMin = st + 1;
#else
        retSTMin = (st / CANTP_SCHEDULE_PERIOD) + 1;
#endif

    }

    return retSTMin;
}

/**************************************************************************//**
 *
 *  \details the period function of the cantp module.
 *
 *  \param[in]  num - The number of the channels in the list.
 *  \param[in]  channelList - the list of the channels.
 *  \param[in]  periodList - the list of the period and timeout interface of
 *                           the channel.
 *
 *  \return None.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_PeriodFunction(UINT16 num,
                                    bl_CanTpChannel_t *channelList,
                                    const bl_CanTpPeriodIF_t *periodList)
{
    bl_CanTpChannel_t *channel;
    UINT8 ret;
    UINT16 i;

    for (i = 0; i < num; i++)
    {
        channel = &channelList[i];
        if (CANTP_IS_TIMEOUT(channel))
        {
            ret = periodList[channel->status].Timeout(channel);
            if (ERR_OK == ret)
            {
                _Cantp_GotoIdle(channel);
            }
        }
        else
        {
            channel->timer -= 1;
            periodList[channel->status].Period(channel);
        }
    }
}

/**************************************************************************//**
 *
 *  \details Indicate a rx channel to receive a frame data.
 *
 *  \param[in]  channel - the pointer of a rx channel.
 *  \param[in]  size - the size of received data
 *  \param[in]  buffer - the contents of received data.
 *
 *  \return If the received data can be processed by the rx channel, returns
 *          the ERR_OK. Otherwise returns ERR_ERROR.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static UINT8 _Cantp_RxIndToRxChannel(bl_CanTpChannel_t *channel,
                                            bl_BufferSize_t size,
                                            const bl_Buffer_t *buffer)
{
    bl_CanTpRxProcess_t processor;
    UINT8 frameType;
    UINT8 ret = ERR_ERROR;

    if (channel != NULL_PTR)
    {
        if ((channel->chnCfg->type == CANTP_TYPE_STANDARD)
            || (channel->chnCfg->ta == buffer[CANTP_TA_OFFSET]))
        {
            frameType = CANTP_GET_FRAME_TYPE(channel->pciInfo,buffer);
            if (frameType < CANTP_FRAME_TYPE_FC)
            {
                processor = gs_RxProcessList[frameType];
                ret = processor(channel,size,buffer);
            }
        }
    }

    return ret;
}

/**************************************************************************//**
 *
 *  \details Indicate a tx channel to receive a frame data.
 *
 *  \param[in]  channel - the pointer of a tx channel.
 *  \param[in]  size - the size of received data
 *  \param[in]  buffer - the contents of received data.
 *
 *  \return If the received data can be processed by the tx channel, returns
 *          the ERR_OK. Otherwise returns ERR_ERROR.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static UINT8 _Cantp_RxIndToTxChannel(bl_CanTpChannel_t *channel,
                                            bl_BufferSize_t size,
                                            const bl_Buffer_t *buffer)
{
    bl_CanTpRxProcess_t processor;
    UINT8 frameType;
    UINT8 ret = ERR_ERROR;
    
    if (channel != NULL_PTR)
    {
        if ((channel->chnCfg->type == CANTP_TYPE_STANDARD)
            || (channel->chnCfg->ta == buffer[CANTP_TA_OFFSET]))
        {
            frameType = CANTP_GET_FRAME_TYPE(channel->pciInfo,buffer);
            if (frameType == CANTP_FRAME_TYPE_FC)
            {
                processor = gs_RxProcessList[frameType];
                ret = processor(channel,size,buffer);
            }
        }
    }

    return ret;
}

/**************************************************************************//**
 *
 *  \details    receive a frame according to the Single Frame in
 *              the ISO 15765-2.
 *
 *  \param[in]  channel - the pointer of a channel.
 *  \param[in]  size - the size of received data
 *  \param[in]  buffer - the contents of received data.
 *
 *  \return If the received data can be processed by the channel, returns
 *          the ERR_OK. Otherwise returns ERR_ERROR.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static UINT8 _Cantp_ReceiveSF(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer)
{
    const bl_CanTpPciInfo_t *pci;
    UINT8 ret = ERR_ERROR;
    UINT8 tmpSize;

    pci = channel->pciInfo;
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
    if((CANTP_STATUS_IS_IDLE(&gs_CanTpTxChannel[CANTP_TATYPE_PHYSICAL])) 
        || (CANTP_IS_FUNCTIONAL_CHANNEL(channel)))
    {
#endif
#if (ENABLE_CANFD == ON)
        /***获得SF_DL***/
        if(size <= 8)
        {
            tmpSize = CANTP_GET_SF_DATASIZE(buffer[pci->pciPos]);
        }
        else
        {
            tmpSize = CANTP_GET_SF_DATASIZE(buffer[(pci->pciPos) + 1]);
            if((0x00 != CANTP_GET_SF_DATASIZE(buffer[pci->pciPos]))
              ||(tmpSize < 8))
            {
                return ret;
            }
        }
#else
        tmpSize = CANTP_GET_SF_DATASIZE(buffer[pci->pciPos]);
#endif
        if ((tmpSize != 0) && (tmpSize <= pci->maxDataSize) && (tmpSize < size))
        {
            /*  When continuous SF is received in one channel during
                a timeout period,It maybe break other channel.
                So If a channel do not get the buffer from DCM module,
                do NOT indicate.*/
            if(CANTP_IS_GETTING_BUFFER(channel))

            {
                Diag_RxIndication(channel->taType, ERR_ERROR);
            }
#if (ENABLE_CANFD == ON)
            if(tmpSize >= 8 )
            {
                FblMemCpy(channel->frame, &buffer[(pci->dataPos) + 1], (UINT16)tmpSize);
            }
            else
#endif
            {
                FblMemCpy(channel->frame, &buffer[pci->dataPos], (UINT16)tmpSize);
            }
            channel->lastSize = tmpSize;

            CANTP_STATUS_GOTO_RECVSF(channel);
            CANTP_INIT_TIMER_B(channel);

            ret = ERR_OK;
        }
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
    }
#endif
    return ret;
}
/**************************************************************************//**
 *
 *  \details    receive a frame according to the First Frame in
 *              the ISO 15765-2.
 *
 *  \param[in]  channel - the pointer of a channel.
 *  \param[in]  size - the size of received data
 *  \param[in]  buffer - the contents of received data.
 *
 *  \return If the received data can be processed by the channel, returns
 *          the ERR_OK. Otherwise returns ERR_ERROR.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static UINT8 _Cantp_ReceiveFF(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer)
{
    const bl_CanTpPciInfo_t *pci;
    UINT8 ret = ERR_ERROR;
    bl_BufferSize_t totalSize;

    if (CANTP_IS_PHYSICAL_CHANNEL(channel))
    {
        pci = channel->pciInfo;
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
        if(CANTP_STATUS_IS_IDLE(&gs_CanTpTxChannel[CANTP_TATYPE_PHYSICAL]))
        {
#endif
#if (ENABLE_CANFD == ON)
            totalSize = CANTP_GET_FF_DATASIZE(buffer[pci->pciPos],buffer[(pci->pciPos) + 1]);

            if(totalSize == 0x0000)/***FF_DL>4095***/
            {
                /***暂时不支持大于4095的数据***/
                return ret;
                /*totalSize = CANTP_GET_FF_DATASIZE2(buffer[(pci->pciPos) + 2],buffer[(pci->pciPos) + 3], \
                                                   buffer[(pci->pciPos) + 3],buffer[(pci->pciPos) + 4]);*/
            }
#else
            totalSize = CANTP_GET_FF_DATASIZE(buffer[pci->pciPos],buffer[(pci->pciPos) + 1]);
#endif
#if (CANTP_FUN_RX_FRAME_PADDING == ON)
            if((totalSize > pci->maxFFDataSize) && (CANTP_MAX_FRAME_SIZE == size))
#else
            if (totalSize > pci->maxFFDataSize)
#endif
            {
                /*  When continuous FF is received in one channel during
                    a timeout period,It maybe break other physical channel.
                    So If a channel do not get the buffer from DCM module,
                    do NOT indicate.*/
                if(CANTP_IS_GETTING_BUFFER(channel))
                {
                    Diag_RxIndication(channel->taType, ERR_ERROR);
                }
                FblMemCpy(channel->frame,&buffer[pci->ffDataPos],(UINT16)(size - pci->pciPos));
                _Cantp_SetMultipleFrameSize(channel,totalSize);

                CANTP_INIT_SN(channel);
                CANTP_INIT_MAXWFT_BY_CFG(channel,channel->chnCfg);
                CANTP_STATUS_GOTO_RECVFF(channel);
                CANTP_INIT_TIMER_B(channel);

                ret = ERR_OK;
            }
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
        }
#endif
    }

    return ret;
}

/**************************************************************************//**
 *
 *  \details    receive a frame according to the Consecutive Frame in
 *              the ISO 15765-2.
 *
 *  \param[in]  channel - the pointer of a channel.
 *  \param[in]  size - the size of received data
 *  \param[in]  buffer - the contents of received data.
 *
 *  \return If the received data can be processed by the channel, returns
 *          the ERR_OK. Otherwise returns ERR_ERROR.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static UINT8 _Cantp_ReceiveCF(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer)
{
    const bl_CanTpPciInfo_t *pci;
    UINT8 ret = ERR_ERROR;
    UINT8 expectedSN;
    UINT8 recvSN;
    UINT8 tmpSize;

    pci = channel->pciInfo;
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
    if(CANTP_STATUS_IS_IDLE(&gs_CanTpTxChannel[CANTP_TATYPE_PHYSICAL]))
    {
#endif
        do
        {
            if (CANTP_STATUS_IS_NOT_RECVCF(channel)
                || (0 == channel->cfCnt))   /*avoid unwanted CF*/
            {
                break;
            }
#if (CANTP_FUN_RX_FRAME_PADDING == OFF)
            if (1 == channel->cfCnt)        /*the last CF*/
            {
                if (size < channel->lastSize)
                {
                    break;
                }
            }
            else
            {
                if (size < CANTP_MAX_FRAME_SIZE)
                {
                    break;
                }
            }
#else
            (void)size;
#endif

            expectedSN = GET_LOW_HALF(channel->sn + 1);
            recvSN = CANTP_GET_CF_SN(pci,buffer);
            if (recvSN == expectedSN)
            {
                channel->sn = expectedSN;
            }
            else
            {
                Diag_RxIndication(channel->taType, ERR_ERROR);
                _Cantp_GotoIdle(channel);
                break;
            }

            channel->cfCnt -= 1;
            if (0 == channel->cfCnt)
            {
                tmpSize = channel->lastSize;
            }
            else
            {
                tmpSize = pci->maxDataSize;
            }

            /*Immediately copy data to buffer avoid continuous CF during a period*/
            ret = Diag_CopyRxData(tmpSize,&buffer[pci->dataPos]);
            if (ERR_OK == ret)
            {
                /*reset the timer of this channel.*/
                CANTP_INIT_TIMER_C(channel);

                if ((channel->bs != 0) && (channel->cfCnt != 0))
                {
                    channel->bs -= 1;
                    if (0 == channel->bs)
                    {
                        _Cantp_GotoTranFC(channel, CANTP_FC_FRAME_CTS);
                    }
                }
            }
            else
            {
                BL_DEBUG_ASSERT_NO_RET(0);
                Diag_RxIndication(channel->taType, ERR_ERROR);
                _Cantp_GotoIdle(channel);
            }
        }while(0);/*lint !e717*/
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
    }
#endif
    return ret;
}

/**************************************************************************//**
 *
 *  \details    receive a frame according to the Flow Control Frame in
 *              the ISO 15765-2.
 *
 *  \param[in]  channel - the pointer of a channel.
 *  \param[in]  size - the size of received data
 *  \param[in]  buffer - the contents of received data.
 *
 *  \return If the received data can be processed by the channel, returns
 *          the ERR_OK. Otherwise returns ERR_ERROR.
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static UINT8 _Cantp_ReceiveFC(bl_CanTpChannel_t *channel,
                                    bl_BufferSize_t size,
                                    const bl_Buffer_t *buffer)
{
    const bl_CanTpPciInfo_t *pci;
    UINT8 ret = ERR_ERROR;
    UINT8 fs;
    UINT8 tmpSt;
    pci = channel->pciInfo;

#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
    if(CANTP_STATUS_IS_IDLE(&gs_CanTpRxChannel[CANTP_TATYPE_PHYSICAL]))
    {
#endif
        /*if the status of this channel is not waiting for FC*/
#if (CANTP_FUN_RX_FRAME_PADDING == OFF)
        if (CANTP_STATUS_IS_RECVFC(channel) && (size >= pci->maxFCDataSize))
#else
        (void)size;

        if (CANTP_STATUS_IS_RECVFC(channel))
#endif
        {
            fs = CANTP_GET_FC_FS(channel->pciInfo,buffer);

            switch(fs)
            {
                case CANTP_FC_FRAME_CTS:
                    channel->bs = CANTP_GET_FC_BS(channel->pciInfo,buffer);
                    tmpSt = CANTP_GET_FC_STMIN(pci,buffer);
                    channel->st = _Cantp_GetSTMinFromFC(tmpSt);

                    _Cantp_GotoTranCF(channel);
                    break;
                case CANTP_FC_FRAME_WAIT:
                    CANTP_INIT_TIMER_B(channel);
                    break;
                case CANTP_FC_FRAME_OVERFLOW:
                    Diag_TxConfirmation(ERR_OVERFLOW);
                    _Cantp_GotoIdle(channel);
                    break;
                default:
                    Diag_TxConfirmation(ERR_ERROR);
                    _Cantp_GotoIdle(channel);
                    break;
            }

            ret = ERR_OK;
        }
#if(CANTP_COMMUNICATION_DUPLEX == CANTP_HALF_DUPLEX)
    }
#endif
    return ret;

}


/**************************************************************************//**
 *
 *  \details    Make the PCI information according to Single Frame and save it
 *              to the local frame of the channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_MakePciOfSF(bl_CanTpChannel_t *channel)
{
    const bl_CanTpPciInfo_t *pci = channel->pciInfo;
    bl_Buffer_t *frame = channel->frame;

    if (channel->chnCfg->type != CANTP_TYPE_STANDARD)
    {
        frame[CANTP_TA_OFFSET] = channel->chnCfg->ta;
    }

    frame[pci->pciPos] = GET_LOW_HALF(channel->lastSize);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Make the PCI information according to First Frame and save it
 *              to the local frame of the channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_MakePciOfFF(bl_CanTpChannel_t *channel)
{
    const bl_CanTpPciInfo_t *pci = channel->pciInfo;
    bl_Buffer_t *frame = channel->frame;
    bl_BufferSize_t totalSize = channel->totalSize;

    if (channel->chnCfg->type != CANTP_TYPE_STANDARD)
    {
        frame[CANTP_TA_OFFSET] = channel->chnCfg->ta;
    }

    totalSize = (totalSize & CANTP_FRAME_FF_DATASIZE_MASK)
                + CANTP_FRAME_FF_VAULE_16BITS;
    frame[pci->pciPos] = (UINT8)(totalSize >> 8);
    frame[(pci->pciPos) + 1] = (UINT8)(totalSize);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Make the PCI information according to Consecutive Frame and
 *              save it to the local frame of the channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_MakePciOfCF(bl_CanTpChannel_t *channel)
{
    const bl_CanTpPciInfo_t *pci = channel->pciInfo;
    bl_Buffer_t *frame = channel->frame;

    if (channel->chnCfg->type != CANTP_TYPE_STANDARD)
    {
        frame[CANTP_TA_OFFSET] = channel->chnCfg->ta;
    }

    frame[pci->pciPos] = CANTP_FRAME_CF_VALUE + GET_LOW_HALF(channel->sn);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Make the PCI information according to Flow Control Frame and
 *              save it to the local frame of the channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_MakePciOfFC(bl_CanTpChannel_t *channel)
{
    const bl_CanTpPciInfo_t *pci = channel->pciInfo;
    bl_Buffer_t *frame = channel->frame;

    if (channel->chnCfg->type != CANTP_TYPE_STANDARD)
    {
        frame[CANTP_TA_OFFSET] = channel->chnCfg->ta;
    }

    frame[pci->pciPos] = CANTP_FRAME_FC_VALUE
                            + GET_LOW_HALF(channel->pData);
    frame[pci->fcBsPos] = channel->bs;
    frame[pci->fcStPos] = channel->st;

    return ;
}

/**************************************************************************//**
 *
 *  \details    The tx confirm function is used for the idle statue of channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_TxConfirmIdle(bl_CanTpChannel_t *channel)
{
    (void)channel;

    BL_DEBUG_ASSERT_NO_RET(CANTP_STATUS_IS_NOT_IDLE(channel));

    return ;
}

/**************************************************************************//**
 *
 *  \details    The tx confirm function is used for the tranSF statue of
 *              channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_TxConfirmSF(bl_CanTpChannel_t *channel)
{
    Diag_TxConfirmation(ERR_OK);

    _Cantp_GotoIdle(channel);

    return ;
}

/**************************************************************************//**
 *
 *  \details    The tx confirm function is used for the tranFF statue of
 *              channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_TxConfirmFF(bl_CanTpChannel_t *channel)
{
    CANTP_ADD_SN(channel);  /*the First Frame use first SN.*/
    CANTP_STATUS_GOTO_RECVFC(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_INIT_TIMER_B(channel);

    return ;
}

/**************************************************************************//**
 *
 *  \details    The tx confirm function is used for the tranCF statue of
 *              channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_TxConfirmCF(bl_CanTpChannel_t *channel)
{
    BL_DEBUG_ASSERT_NO_RET(CANTP_SUB_STATUS_IS_TRAN(channel));

    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_ADD_SN(channel);

    if (channel->cfCnt == 0)
    {
        Diag_TxConfirmation(ERR_OK);
        _Cantp_GotoIdle(channel);
    }
    else
    {
        if (channel->bs != 0)
        {
            channel->bs -= 1;
            if (0 == channel->bs)
            {
                CANTP_STATUS_GOTO_RECVFC(channel);
                CANTP_INIT_TIMER_B(channel);
            }
            else
            {
            	CANTP_SET_TIMER(channel,CANTP_CHNNEL_RX_CR);
            	CANTP_INIT_TXDELAY(channel);
            }
        }
        else
        {
            CANTP_SET_TIMER(channel,CANTP_CHNNEL_RX_CR);
            CANTP_INIT_TXDELAY(channel);
        }
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    The tx confirm function is used for the tranFC statue of
 *              channel.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_TxConfirmFC(bl_CanTpChannel_t *channel)
{
    switch(channel->pData)
    {
        case CANTP_FC_FRAME_CTS:
            CANTP_STATUS_GOTO_RECVCF(channel);
            CANTP_INIT_TIMER_C(channel);
            break;
        case CANTP_FC_FRAME_WAIT:
            CANTP_STATUS_GOTO_RECVFF(channel);
            CANTP_INIT_TIMER_B(channel);
            break;
        default:
            _Cantp_GotoIdle(channel);
            break;
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    Change the status of the channel to tranFC status.
 *
 *  \param[in/out]  channel - the pointer of a rx channel.
 *  \param[in] fs - the FS value of the FC frame.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_GotoTranFC(bl_CanTpChannel_t *channel, UINT8 fs)
{
    CANTP_STATUS_GOTO_TRANFC(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_INIT_TIMER_A(channel);
    CANTP_INIT_BS_BY_CFG(channel,channel->chnCfg);
    CANTP_SET_PRIVATE_DATA(channel,fs);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Change the status of the channel to Idle status.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_GotoIdle(bl_CanTpChannel_t *channel)
{
    CANTP_STATUS_GOTO_IDLE(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_INIT_TIMER(channel);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Change the status of the channel to tranSF status.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_GotoTranSF(bl_CanTpChannel_t *channel)
{
    CANTP_STATUS_GOTO_TRANSF(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_INIT_TIMER_A(channel);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Change the status of the channel to tranFF status.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_GotoTranFF(bl_CanTpChannel_t *channel)
{
    CANTP_STATUS_GOTO_TRANFF(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_INIT_SN(channel);
    CANTP_INIT_TIMER_A(channel);

    return ;
}

/**************************************************************************//**
 *
 *  \details    Change the status of the channel to tranCF status.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_GotoTranCF(bl_CanTpChannel_t *channel)
{
    CANTP_STATUS_GOTO_TRANCF(channel);
    CANTP_SUB_STATUS_GOTO_IDLE(channel);
    CANTP_SET_TIMER(channel,CANTP_CHNNEL_RX_CR);
    CANTP_INIT_TXDELAY(channel);

    return ;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the Idle status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \note   This function is not called except the timer is not set to zero
 *          during the status of this channel is changed to IDLE.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodIdle(bl_CanTpChannel_t *channel)
{
    (void)channel;
    /*  This function is not called except the timer is not set to zero
        during the status of this channel is changed to IDLE.*/
    BL_DEBUG_ASSERT_NO_RET(0);

    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the Idle status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \note   Avoid unnecessary status switching, this function always returns
 *          ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutIdle(bl_CanTpChannel_t *channel)
{
    (void)channel;

    return ERR_ERROR;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the recvSF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodRecvSF(bl_CanTpChannel_t *channel)
{
    UINT8 ret;
    /*alloc the buffer from the Dcm*/
    ret = Diag_StartOfReception(channel->lastSize);
    /*if success then copy frame to buffer and goto idle*/
    if (ERR_OK == ret)
    {
        ret = Diag_CopyRxData(channel->lastSize,channel->frame);
        if (ERR_OK == ret)
        {
            Diag_RxIndication(channel->taType, ERR_OK);
        }
        else
        {
            BL_DEBUG_ASSERT_NO_RET(0);
            Diag_RxIndication(channel->taType, ERR_ERROR);
        }

        _Cantp_GotoIdle(channel);
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the recvSF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutRecvSF(bl_CanTpChannel_t *channel)
{
    (void)channel;

    return ERR_OK;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the recvFF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodRecvFF(bl_CanTpChannel_t *channel)
{
    UINT8 ret;
    /*Apply for the buffer from the Dcm*/
    ret = Diag_StartOfReception(channel->totalSize);

    if (ERR_OK == ret)
    {
        ret = Diag_CopyRxData(channel->pciInfo->maxFFDataSize,channel->frame);
        if (ERR_OK == ret)
        {
            _Cantp_GotoTranFC(channel, CANTP_FC_FRAME_CTS);
        }
        else
        {
            BL_DEBUG_ASSERT_NO_RET(0);
            Diag_RxIndication(channel->taType, ERR_ERROR);
            _Cantp_GotoIdle(channel);
        }
    }
    else if (ERR_OVERFLOW == ret)
    {
        _Cantp_GotoTranFC(channel, CANTP_FC_FRAME_OVERFLOW);
    }
    else
    {
        /* The buffer busy, wait the next period to alloc again.*/
    }

    return ;

}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the recvFF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutRecvFF(bl_CanTpChannel_t *channel)
{
    UINT8 ret = ERR_OK;

    if (channel->wft != 0)
    {
        channel->wft -= 1;

        _Cantp_GotoTranFC(channel, CANTP_FC_FRAME_WAIT);

        ret = ERR_ERROR;
    }

    /* Br timeout, do not indicate the DCM.*/
    return ret;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the recvCF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodRecvCF(bl_CanTpChannel_t *channel)
{
    if (0 == channel->cfCnt)
    {
        /*All CF is successfully received.*/
        Diag_RxIndication(channel->taType, ERR_OK);

        _Cantp_GotoIdle(channel);
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the recvCF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutRecvCF(bl_CanTpChannel_t *channel)
{
    (void)channel;

    /*Cr timeout!*/
    Diag_RxIndication(channel->taType, ERR_ERROR);

    return ERR_OK;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the tranFC status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodTranFC(bl_CanTpChannel_t *channel)
{
    UINT8 ret;
    bl_BufferSize_t frameSize;
    bl_Buffer_t *frame;
    UINT16 id;


    BL_DEBUG_ASSERT_NO_RET(channel != NULL_PTR);
    BL_DEBUG_ASSERT_NO_RET(CANTP_STATUS_IS_TRANFC(channel));

    if (CANTP_SUB_STATUS_IS_IDLE(channel))
    {
        _Cantp_MakePciOfFC(channel);

        frameSize = channel->pciInfo->maxFCDataSize;
        frame = channel->frame;
        id = channel->chnCfg->txId;

        BL_DEBUG_ASSERT_NO_RET(frameSize != 0);

#if (CANTP_FUN_TX_FRAME_PADDING == ON)
        Bl_MemSet(&frame[frameSize],
                    CANTP_FRAME_PADDING_VALUE,
                    (UINT16)(CANTP_MAX_FRAME_SIZE - frameSize));

        ret = FblCanSendData(frame, id,CANTP_MAX_FRAME_SIZE);
#else
        ret = FblCanSendData(frame, id,frameSize);

#endif

        if (ERR_OK == ret)
        {
            CANTP_SUB_STATUS_GOTO_TRAN(channel);
            gs_TransmittingChannel[0] = channel;
            Cantp_TxConfirmation(id);
        }
    }

    return ;

}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the tranFC status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
/*lint -e{818}*/
static UINT8 _Cantp_TimeoutTranFC(bl_CanTpChannel_t *channel)
{
    if (channel->pData != CANTP_FC_FRAME_WAIT)
    {
        /*  When a FC frame with wait flag is transmitted,
            this channel do not get a buffer from DCM module.
            So do NOT indicate the DCM module in order to avoid
            break other channel.*/
        Diag_RxIndication(channel->taType, ERR_ERROR);
    }

    return ERR_OK;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the tranSF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodTranSF(bl_CanTpChannel_t *channel)
{
    UINT8 ret;
    UINT8 dataPos;
    bl_BufferSize_t dataSize;
    bl_BufferSize_t frameSize;
    bl_Buffer_t *frame;
    UINT16 id;
    BL_DEBUG_ASSERT_NO_RET(channel != NULL_PTR);
    BL_DEBUG_ASSERT_NO_RET(CANTP_STATUS_IS_TRANSF(channel));

    if (CANTP_SUB_STATUS_IS_IDLE(channel))
    {
        _Cantp_MakePciOfSF(channel);

        dataSize = channel->lastSize;
        frame = channel->frame;
        dataPos = channel->pciInfo->dataPos;
        id = channel->chnCfg->txId;
        frameSize = dataSize + dataPos;

        BL_DEBUG_ASSERT_NO_RET(dataSize != 0);
        BL_DEBUG_ASSERT_NO_RET(frameSize <= CANTP_MAX_FRAME_SIZE);

        ret = Diag_CopyTxData(dataSize, &frame[dataPos]);

        BL_DEBUG_ASSERT_NO_RET(ERR_OK == ret);

#if (CANTP_FUN_TX_FRAME_PADDING == ON)
        Bl_MemSet(&frame[frameSize],
                    CANTP_FRAME_PADDING_VALUE,
                    (UINT16)(CANTP_MAX_FRAME_SIZE - frameSize));

        ret = FblCanSendData(frame, id,CANTP_MAX_FRAME_SIZE);
#else
        ret = FblCanSendData(frame, id,frameSize);
#endif
        if (ERR_OK == ret)
        {
            CANTP_SUB_STATUS_GOTO_TRAN(channel);
            gs_TransmittingChannel[0] = channel;
            Cantp_TxConfirmation(id);
        }
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the tranSF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutTranSF(bl_CanTpChannel_t *channel)
{
    (void)channel;
    /*The As timeout.*/
    Diag_TxConfirmation(ERR_ERROR);
    return ERR_OK;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the tranFF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodTranFF(bl_CanTpChannel_t *channel)
{
    UINT8 ret;
    UINT8 dataPos;
    bl_BufferSize_t dataSize;
    bl_Buffer_t *frame;
    UINT16 id;

    BL_DEBUG_ASSERT_NO_RET(channel != NULL_PTR);
    BL_DEBUG_ASSERT_NO_RET(CANTP_STATUS_IS_TRANFF(channel));
    BL_DEBUG_ASSERT_NO_RET(channel->totalSize != 0);

    if (CANTP_SUB_STATUS_IS_IDLE(channel))
    {
        _Cantp_MakePciOfFF(channel);

        frame = channel->frame;
        dataPos = channel->pciInfo->ffDataPos;
        dataSize = channel->pciInfo->maxFFDataSize;
        id = channel->chnCfg->txId;

        ret = Diag_CopyTxData(dataSize, &frame[dataPos]);

        BL_DEBUG_ASSERT_NO_RET(ERR_OK == ret);

        ret = FblCanSendData(frame, id,CANTP_MAX_FRAME_SIZE);

        if (ERR_OK == ret)
        {
            CANTP_SUB_STATUS_GOTO_TRAN(channel);
            gs_TransmittingChannel[0] = channel;
            Cantp_TxConfirmation(id);
        }
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the tranFF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutTranFF(bl_CanTpChannel_t *channel)
{
    (void)channel;
    /*The As timeout.*/
    Diag_TxConfirmation(ERR_ERROR);
    return ERR_OK;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the tranCF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodTranCF(bl_CanTpChannel_t *channel)
{
    BL_DEBUG_ASSERT_NO_RET(channel != NULL_PTR);
    BL_DEBUG_ASSERT_NO_RET(CANTP_STATUS_IS_TRANCF(channel));
    BL_DEBUG_ASSERT_NO_RET(channel->cfCnt > 0);

    if (CANTP_SUB_STATUS_IS_IDLE(channel))
    {
        if (channel->txDelay != 0)
        {
            channel->txDelay -= 1;
        }

        if (channel->txDelay == 0)
        {
            _Cantp_TransmitCF(channel);
        }
    }

    return ;
}
/**************************************************************************//**
 *
 *  \details    transmit a frame according to the Flow Control Frame in
 *              the ISO 15765-2.
 *              If the frame is not transmitted, try again during the next
 *              period.
 *
 *  \param[in/out]  channel - the pointer of a tx channel.
 *
 *  \return None
 *
 *  \since  V5.0.0
 *
 *****************************************************************************/
static void _Cantp_TransmitCF(bl_CanTpChannel_t *channel)
{
    UINT8 ret;
    UINT8 dataPos;
    bl_BufferSize_t dataSize;
    bl_BufferSize_t frameSize;
    bl_Buffer_t *frame;
    UINT16 id;
    UINT16 cfCounter;

    _Cantp_MakePciOfCF(channel);

    cfCounter = channel->cfCnt - 1;
    if (0 == cfCounter)
    {
        dataSize = channel->lastSize;
    }
    else
    {
        dataSize = channel->pciInfo->maxDataSize;
    }

    frame = channel->frame;
    dataPos = channel->pciInfo->dataPos;
    id = channel->chnCfg->txId;
    frameSize = dataSize + dataPos;
    BL_DEBUG_ASSERT_NO_RET(dataSize != 0);
    BL_DEBUG_ASSERT_NO_RET(frameSize <= CANTP_MAX_FRAME_SIZE);

#if (CANTP_FUN_TX_FRAME_PADDING == ON)
    Bl_MemSet(&frame[frameSize],
                CANTP_FRAME_PADDING_VALUE,
                (UINT16)(CANTP_MAX_FRAME_SIZE - frameSize));

    ret = FblCanSendData(frame, id,CANTP_MAX_FRAME_SIZE );
#else
    ret = FblCanSendData(frame, id,CANTP_MAX_FRAME_SIZE );
#endif


    if (ERR_OK == ret)
    {
        channel->cfCnt = cfCounter;
        CANTP_SUB_STATUS_GOTO_TRAN(channel);
        gs_TransmittingChannel[0] = channel;
        Cantp_TxConfirmation(id);
    }

    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the tranCF status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutTranCF(bl_CanTpChannel_t *channel)
{
    (void)channel;
    /*The As timeout.*/
    Diag_TxConfirmation(ERR_ERROR);
    return ERR_OK;
}

/**************************************************************************//**
 *
 *  \details    The period function is used for the recvFC status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return None
 *
 *  \since V5.1.0
 *
 *****************************************************************************/
static void _Cantp_PeriodRecvFC(bl_CanTpChannel_t *channel)
{
    (void)channel;
    return ;
}

/**************************************************************************//**
 *
 *  \details    The timeout function is used for the recvFC status of a channel.
 *
 *  \param[in/out]  channel - the pointer of a channel.
 *
 *  \return If the timeout event is processed return ERR_OK, otherwise
 *          return ERR_ERROR.
 *
 *  \since  V5.1.0
 *
 *****************************************************************************/
static UINT8 _Cantp_TimeoutRecvFC(bl_CanTpChannel_t *channel)
{
    (void)channel;
    /*The Bs timeout.*/
    Diag_TxConfirmation(ERR_ERROR);
    return ERR_OK;
}



/*************************************************************************************************************
                                          End Of File
*************************************************************************************************************/
