#include "information/sd_protocol.h"
#include "bsp_driver_sd.h"
#include "fatfs.h"
#include "information/device.hpp"
#include "main.h"
#include "sdio.h"
#include "stm32f4xx_hal.h"

static uint8_t buffer[_MAX_SS];   /* a work buffer for the f_mkfs() */
uint32_t byteswritten, bytesread; /* file write/read counts */
static char* f_name = "test.txt"; /* file name */
uint8_t wtext[] = " wagabagabobo "; /* file write buffer */
uint8_t rtext[100];               /* file read buffer */
uint8_t err;
FIL fil;                             /* File object */

void checkOperatingType() {
    if ((f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) == FR_OK) && (f_mkfs((TCHAR const*)SDPath, FM_EXFAT, 0, buffer, sizeof(buffer)) == FR_NOT_READY)) {
        HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
        /* Create and Open a new text file object with write access */
        if (f_open(&SDFile, f_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
            //HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
            /* Write data to the text file */
            f_write(&SDFile, wtext, sizeof(wtext), (UINT*)&byteswritten);
            /* Close the open text file */
            f_close(&SDFile);

            /* Open the text file object with read access */
            if (f_open(&SDFile, f_name, FA_READ) == FR_OK) {
                /* Read data from the text file */
                f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);

                /* Close the open text file */
                f_close(&SDFile);
            } else {
                err = ERR_OPEN;
            }
        } else {
            err = ERR_OPEN;
        }
    } else {
        err = ERR_MOUNT_MKFS;
    }
    /* Unlink the SD disk I/O driver */
    FATFS_UnLinkDriver(SDPath);
}

void sdTestFunc() {
    //mount the SD card
    f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
    //if (f_mkfs((TCHAR const*)SDPath, FM_EXFAT, 0, buffer, sizeof(buffer)) != FR_OK) {
    //    HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
    //}
    //test open file
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
    if (f_open(&SDFile, f_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        //HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
    }
    //test write to file
    // if (f_write(&SDFile, wtext, sizeof(wtext), (UINT*)&byteswritten) != FR_OK) {
    //     HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
    // }
    f_close(&SDFile);
}
