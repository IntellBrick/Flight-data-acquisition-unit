/*
 * FatFs 与 HAL SDMMC1 的适配层。
 * FatFs 通过 disk_* 接口读写 512 字节逻辑扇区，本文件使用有超时的 HAL 轮询操作。
 */
#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "ff.h"
#include "diskio.h"

#define SECTOR_SIZE 512U      /* 与 ffconf.h 中 FF_MIN_SS、FF_MAX_SS 一致。 */
#define SECTORS_PER_IO 8U     /* 单次 HAL 数据操作最多 4 KiB。 */
#define SD_TIMEOUT_MS 300U   /* 数据传输和卡就绪轮询各自的超时，单位毫秒。 */

static SD_HandleTypeDef sd = {
    .Instance = SDMMC1,
    .Init = {
        .ClockEdge = SDMMC_CLOCK_EDGE_RISING,
        .ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE,
        .BusWide = SDMMC_BUS_WIDE_4B,
        .HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE,
        .ClockDiv = 4, /* 正常卡时钟为 PLL1Q / (2 × 4)，即 12.5 MHz。 */
    },
};
static DSTATUS status = STA_NOINIT;
/* 四字节对齐的中转缓冲，适配 FatFs 可能传入的未对齐指针。 */
static _Alignas(4) uint8_t buffer[SECTOR_SIZE * SECTORS_PER_IO];

/* HAL 块写返回后仍需等卡退出忙状态，CTRL_SYNC 也复用这一步。 */
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

/* 首次挂载或故障重挂载时初始化卡，仅接受逻辑设备号 0。 */
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

/* 返回最近一次初始化/读写维护的状态，卡状态由实际 I/O 进一步确认。 */
DSTATUS disk_status(BYTE drive)
{
    return drive == 0 ? status : STA_NOINIT;
}

/* read_out 与 write_in 由读写入口二选一提供，大请求分为最多八扇区一组。 */
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

/* FatFs 的扇区读取入口，数据经过对齐缓冲后复制到调用者。 */
DRESULT disk_read(BYTE drive, BYTE *out, LBA_t sector, UINT count)
{
    return transfer(drive, out, NULL, sector, count);
}

/* FatFs 的扇区写入入口，每组数据写完后确认卡已可继续接收命令。 */
DRESULT disk_write(BYTE drive, const BYTE *in, LBA_t sector, UINT count)
{
    return transfer(drive, NULL, in, sector, count);
}

/* 提供同步、容量、扇区大小及块大小查询，CTRL_SYNC 与文件同步链路关联。 */
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
