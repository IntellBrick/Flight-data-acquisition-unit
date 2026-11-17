#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "ff.h"
#include "diskio.h"

#define SECTOR_SIZE 512U
#define SECTORS_PER_IO 8U
#define SD_TIMEOUT_MS 300U

static SD_HandleTypeDef sd = {
    .Instance = SDMMC1,
    .Init = {
        .ClockEdge = SDMMC_CLOCK_EDGE_RISING,
        .ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE,
        .BusWide = SDMMC_BUS_WIDE_4B,
        .HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE,
        .ClockDiv = 4, // PLL1Q = 100 MHz, SD clock = 12.5 MHz.
    },
};
static DSTATUS status = STA_NOINIT;
static _Alignas(4) uint8_t buffer[SECTOR_SIZE * SECTORS_PER_IO];

static bool card_ready(void)
{
    uint32_t start = HAL_GetTick();
    do {
        HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&sd);
        if (state == HAL_SD_CARD_TRANSFER) {
            return true;
        }
        if (state == HAL_SD_CARD_ERROR || state == HAL_SD_CARD_DISCONNECTED) {
            break;
        }
    } while (HAL_GetTick() - start < SD_TIMEOUT_MS);
    status = STA_NOINIT;
    return false;
}

DSTATUS disk_initialize(BYTE drive)
{
    if (drive != 0) {
        return STA_NOINIT;
    }
    if (sd.State != HAL_SD_STATE_RESET) {
        (void)HAL_SD_DeInit(&sd);
    }
    status = STA_NOINIT;
    if (HAL_SD_Init(&sd) == HAL_OK && card_ready()) {
        status = 0;
    }
    return status;
}

DSTATUS disk_status(BYTE drive)
{
    return drive == 0 ? status : STA_NOINIT;
}

static DRESULT transfer(BYTE drive, BYTE *read_out, const BYTE *write_in, LBA_t sector, UINT count)
{
    if (drive != 0 || count == 0) {
        return RES_PARERR;
    }
    if (status != 0) {
        return RES_NOTRDY;
    }
    uint32_t sectors = sd.SdCard.LogBlockNbr;
    if (sector >= sectors || count > sectors - sector) {
        return RES_PARERR;
    }
    while (count != 0) {
        UINT chunk = count < SECTORS_PER_IO ? count : SECTORS_PER_IO;
        unsigned bytes = chunk * SECTOR_SIZE;
        HAL_StatusTypeDef result;
        if (write_in != NULL) {
            memcpy(buffer, write_in, bytes);
            result = HAL_SD_WriteBlocks(&sd, buffer, sector, chunk, SD_TIMEOUT_MS);
            write_in += bytes;
        } else {
            result = HAL_SD_ReadBlocks(&sd, buffer, sector, chunk, SD_TIMEOUT_MS);
        }
        if (result != HAL_OK || !card_ready()) {
            status = STA_NOINIT;
            return RES_ERROR;
        }
        if (read_out != NULL) {
            memcpy(read_out, buffer, bytes);
            read_out += bytes;
        }
        sector += chunk;
        count -= chunk;
    }
    return RES_OK;
}

DRESULT disk_read(BYTE drive, BYTE *out, LBA_t sector, UINT count)
{
    return transfer(drive, out, NULL, sector, count);
}

DRESULT disk_write(BYTE drive, const BYTE *in, LBA_t sector, UINT count)
{
    return transfer(drive, NULL, in, sector, count);
}

DRESULT disk_ioctl(BYTE drive, BYTE command, void *out)
{
    if (drive != 0) {
        return RES_PARERR;
    }
    if (status != 0) {
        return RES_NOTRDY;
    }
    switch (command) {
    case CTRL_SYNC:
        return card_ready() ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT:
        *(LBA_t *)out = sd.SdCard.LogBlockNbr;
        return RES_OK;
    case GET_SECTOR_SIZE:
        *(WORD *)out = SECTOR_SIZE;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *(DWORD *)out = 1;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}
