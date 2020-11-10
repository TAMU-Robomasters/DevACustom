#include "main.h"
#include "information/rc_protocol.h"

/* 
	Code heavily taken from DJI
*/

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t dmaData[18];
uint8_t dmaRxBuffer[2][18];
RC_ctrl_t rcDataStruct;
RC_ctrl_t lastRcDataStruct;

RC_ctrl_t* getRCData() {
    return &rcDataStruct;
}

void processDMAData(){
    lastRcDataStruct = rcDataStruct;

    rcDataStruct.rc.ch[0] = (dmaData[0] | (dmaData[1] << 8)) & 0x07ff;        //!< Channel 0: Right X
    rcDataStruct.rc.ch[1] = ((dmaData[1] >> 3) | (dmaData[2] << 5)) & 0x07ff; //!< Channel 1: Right Y
    rcDataStruct.rc.ch[2] = ((dmaData[2] >> 6) | (dmaData[3] << 2) |          //!< Channel 2: Left X
                         (dmaData[4] << 10)) &
                        0x07ff;
    rcDataStruct.rc.ch[3] = ((dmaData[4] >> 1) | (dmaData[5] << 7)) & 0x07ff; //!< Channel 3: Left Y
    rcDataStruct.rc.s[0] = ((dmaData[5] >> 4) & 0x0003);                       //!< Switch left
    rcDataStruct.rc.s[1] = ((dmaData[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rcDataStruct.mouse.x = dmaData[6] | (dmaData[7] << 8);                    //!< Mouse X axis
    rcDataStruct.mouse.y = dmaData[8] | (dmaData[9] << 8);                    //!< Mouse Y axis
    rcDataStruct.mouse.z = dmaData[10] | (dmaData[11] << 8);                  //!< Mouse Z axis
    rcDataStruct.mouse.press_l = dmaData[12];                                  //!< Mouse Left Is Press ?
    rcDataStruct.mouse.press_r = dmaData[13];                                  //!< Mouse Right Is Press ?
    rcDataStruct.key.v = dmaData[14] | (dmaData[15] << 8);                    //!< KeyBoard value
    rcDataStruct.rc.ch[4] = dmaData[16] | (dmaData[17] << 8);                 //NULL

    rcDataStruct.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

//receive data, 18 bytes one frame, but set 36 bytes

void RCInit() {
    //enable the DMA transfer for the receiver request
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enable idle interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
	hdma_usart1_rx.Instance->M0AR = (uint32_t) dmaRxBuffer[0];
    //memory buffer 2
	hdma_usart1_rx.Instance->M1AR = (uint32_t) dmaRxBuffer[1];
    //data length
	hdma_usart1_rx.Instance->NDTR = 36;
    //enable double memory buffer
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

bool btnIsRising(btnType btn){
    int curr;
    int last;
    switch (btn){
        case btnMouseL:
            curr = rcDataStruct.mouse.press_l;
            last = lastRcDataStruct.mouse.press_l;
            break;
        
        case btnMouseR:
            curr = rcDataStruct.mouse.press_r;
            last = lastRcDataStruct.mouse.press_r;
            break;

        default:
            curr = GET_BIT(rcDataStruct.key.v, btn);
            last = GET_BIT(lastRcDataStruct.key.v, btn);
            break;
    }
    return curr-last == 1;
}

bool btnIsFalling(btnType btn){
    int curr;
    int last;
    switch (btn){
        case btnMouseL:
            curr = rcDataStruct.mouse.press_l;
            last = lastRcDataStruct.mouse.press_l;
            break;
        
        case btnMouseR:
            curr = rcDataStruct.mouse.press_r;
            last = lastRcDataStruct.mouse.press_r;
            break;

        default:
            curr = GET_BIT(rcDataStruct.key.v, btn);
            last = GET_BIT(lastRcDataStruct.key.v, btn);
            break;
    }
    return last-curr == 1;
}
