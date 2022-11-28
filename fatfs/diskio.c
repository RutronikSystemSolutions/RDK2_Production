/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various existing       */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"         /* Obtains integer types */
#include "diskio.h"     /* Declarations of disk functions */
#include "cyhal.h"
#include "sd_card.h"
#include <stdio.h>

/* SD initialization flag */
uint8_t SD_initVar = 0U;


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv        /* Physical drive number to identify the drive */
)
{
    DSTATUS stat = RES_OK;

    switch (pdrv) {
    case DEV_SD :
        if(0U == SD_initVar) {
            return STA_NOINIT;
        }
        return stat;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv                /* Physical drive number to identify the drive */
)
{
    DSTATUS stat = RES_OK;
    cy_rslt_t result;

    switch (pdrv) {
    case DEV_SD :
        if (0U == SD_initVar) {
            /* Initialize the SD card */
            result = sd_card_init();
            if(result != CY_RSLT_SUCCESS) {
                return STA_NOINIT;
            }
            SD_initVar = 1U;
        }
        return stat;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,        /* Physical drive number to identify the drive */
    BYTE *buff,        /* Data buffer to store read data */
    LBA_t sector,    /* Start sector in LBA */
    UINT count        /* Number of sectors to read */
)
{
    DRESULT res = RES_OK;
    cy_rslt_t result;

    switch (pdrv) {
    case DEV_SD :
        if (0U == SD_initVar) {
            return RES_NOTRDY;
        }
        result = sd_card_read(sector, buff, (uint32_t *)&count);
        if (result != CY_RSLT_SUCCESS) {
            printf("sd_card_read error: sector=%d count=%d\r\n", (int)sector, (int)count);
            return RES_ERROR;
        }
        return res;
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
    BYTE pdrv,            /* Physical drive number to identify the drive */
    const BYTE *buff,    /* Data to be written */
    LBA_t sector,        /* Start sector in LBA */
    UINT count            /* Number of sectors to write */
)
{
    DRESULT res = RES_OK;
    cy_rslt_t result;

    switch (pdrv) {
    case DEV_SD :
        if (0U == SD_initVar) {
            return RES_NOTRDY;
        }
        result = sd_card_write(sector, buff, (uint32_t *)&count);
        if (result != CY_RSLT_SUCCESS) {
            printf("sd_card_write error: sector=%d count=%d\r\n", (int)sector, (int)count);
            return RES_ERROR;
        }
        return res;
    }

    return RES_PARERR;
}

#endif


DWORD get_fattime (void)
{
    // TODO: Add RTC?
    return (40 << 25) | (1 << 21) | (1 << 16) | (0 << 11) | (0 << 5) | (0 << 0);
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,        /* Physical drive number (0..) */
    BYTE cmd,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
    DRESULT res = RES_OK;

    switch (pdrv) {
    case DEV_SD :
        if (0U == SD_initVar) {
            return RES_NOTRDY;
        }
        switch(cmd) {
            case CTRL_SYNC:
                break;
            case GET_SECTOR_COUNT: /* Get media size */
                *(DWORD *) buff = sd_card_max_sector_num();
                break;
            case GET_SECTOR_SIZE: /* Get sector size */
                *(WORD *) buff = sd_card_sector_size();
                break;
            case GET_BLOCK_SIZE: /* Get erase block size */
                *(DWORD *) buff = 8;
                break;
            default:
                res = RES_PARERR;
                break;
        }
        return res;
    }

    return RES_PARERR;
}

