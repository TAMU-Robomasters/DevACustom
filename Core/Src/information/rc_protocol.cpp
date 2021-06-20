#include "information/rc_protocol.h"
#include "main.h"

/*
	Code heavily taken from DJI
*/

uint8_t dmaData[DBUS_BUFLEN];
RC_ctrl_t rcDataStruct;
RC_ctrl_t lastRcDataStruct;

RC_ctrl_t* getRCData() {
    return &rcDataStruct;
}

void processDMAData() {
    lastRcDataStruct = rcDataStruct;

    rcDataStruct.rc.ch[0] = (dmaData[0] | (dmaData[1] << 8)) & 0x07ff;        //!< Channel 0: Right X
    rcDataStruct.rc.ch[1] = ((dmaData[1] >> 3) | (dmaData[2] << 5)) & 0x07ff; //!< Channel 1: Right Y
    rcDataStruct.rc.ch[2] = ((dmaData[2] >> 6) | (dmaData[3] << 2) |          //!< Channel 2: Left X
                             (dmaData[4] << 10)) &
                            0x07ff;
    rcDataStruct.rc.ch[3] = ((dmaData[4] >> 1) | (dmaData[5] << 7)) & 0x07ff; //!< Channel 3: Left Y
    rcDataStruct.rc.s[0] = ((dmaData[5] >> 4) & 0x0003);                      //!< Switch left
    rcDataStruct.rc.s[1] = ((dmaData[5] >> 4) & 0x000C) >> 2;                 //!< Switch right
    rcDataStruct.mouse.x = dmaData[6] | (dmaData[7] << 8);                    //!< Mouse X axis
    rcDataStruct.mouse.y = dmaData[8] | (dmaData[9] << 8);                    //!< Mouse Y axis
    rcDataStruct.mouse.z = dmaData[10] | (dmaData[11] << 8);                  //!< Mouse Z axis
    rcDataStruct.mouse.press_l = dmaData[12];                                 //!< Mouse Left Is Press ?
    rcDataStruct.mouse.press_r = dmaData[13];                                 //!< Mouse Right Is Press ?
    rcDataStruct.key.v = dmaData[14] | (dmaData[15] << 8);                    //!< KeyBoard value
    rcDataStruct.rc.ch[4] = dmaData[16] | (dmaData[17] << 8);                 //NULL

    rcDataStruct.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rcDataStruct.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef* dma_stream) {
    /* Return the number of remaining data units for DMAy Streamx */
    return ((uint16_t)(dma_stream->NDTR));
}

/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size) {
    uint32_t tmp1 = 0;

    tmp1 = huart->RxState;

    if (tmp1 == HAL_UART_STATE_READY) {
        if ((pData == NULL) || (Size == 0)) {
            return HAL_ERROR;
        }

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart) {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* handle received data in idle interrupt */
    if (huart == &DBUS_HUART) {
        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* handle dbus data dbus_buf from DMA */
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN) {
            processDMAData();
        }

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

/**
  * @brief      callback this function when uart interrupt
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
void uart_receive_handler(UART_HandleTypeDef* huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
        uart_rx_idle_callback(huart);
    }
}

/**
  * @brief   initialize dbus uart device 
  * @param   
  * @retval  
  */
void RCInit() {
    /* open uart idle it */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

    uart_receive_dma_no_it(&DBUS_HUART, dmaData, DBUS_MAX_LEN);
}

bool getBtn(btnType btn) {
    bool val;
    switch (btn) {
    case btnMouseL:
        val = rcDataStruct.mouse.press_l;
        break;

    case btnMouseR:
        val = rcDataStruct.mouse.press_r;
        break;

    default:
        val = GET_BIT(rcDataStruct.key.v, btn);
        break;
    }
    return val;
}

bool btnIsRising(btnType btn) {
    int curr;
    int last;
    switch (btn) {
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
    return curr - last == 1;
}

bool btnIsFalling(btnType btn) {
    int curr;
    int last;
    switch (btn) {
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
    return last - curr == 1;
}

float getMouse(mouseAxis axis) {
    float val;
    switch (axis) {
    case x: {
        val = rcDataStruct.mouse.x;
        break;
    }

    case y: {
        val = rcDataStruct.mouse.y;
        break;
    }

    case z: {
        val = rcDataStruct.mouse.z;
        break;
    }
    }
    return val;
}

bool switchIsRising(switchType sw, switchPosition pos) {
    int curr = (rcDataStruct.rc.s[sw] == pos);
    int last = (lastRcDataStruct.rc.s[sw] == pos);
    return curr - last == 1;
}

bool switchIsFalling(switchType sw, switchPosition pos) {
    int curr = (rcDataStruct.rc.s[sw] == pos);
    int last = (lastRcDataStruct.rc.s[sw] == pos);
    return last - curr == 1;
}

uint8_t getSwitch(switchType sw) {
    return rcDataStruct.rc.s[sw];
}

float getJoystick(joystickAxis joy) {
    return static_cast<float>(rcDataStruct.rc.ch[joy]) / 660.0f;
}
