/*****************************************************************************
* File Name: sd_card.c
*
* Description:
*  This file provides the source code to operate a SD card.
*
* Note:
*
******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*****************************************************************************/
#include "sd_card.h"
#include "cy_utils.h"
#include "cyhal.h"
#include "cycfg.h"
#include <stdio.h>

/*******************************************************************************
* Constants
*******************************************************************************/
/* Pins connected to the SDHC block */
#define CMD                     uSD_CMD
#define CLK                     uSD_CLK
#define DAT0                    uSD_DAT0
#define DAT1                    uSD_DAT1
#define DAT2                    uSD_DAT2
#define DAT3                    uSD_DAT3

/* dat4 to dat7 are reserved for future use and should be NC */
#define DAT4                    NC
#define DAT5                    NC
#define DAT6                    NC
#define DAT7                    NC
#define CARD_DETECT             uSD_DETECT
#define EMMC_RESET              NC
#define IO_VOLT_SEL             NC
#define CARD_IF_PWREN           NC
#define CARD_MECH_WRITEPROT     NC
#define LED_CTL                 NC

#define ENABLE_LED_CONTROL      false
#define LOW_VOLTAGE_SIGNALLING  false
#define IS_EMMC                 false
#define BUS_WIDTH               4
#define INITIAL_VALUE           true

#define DEFAULT_BLOCKSIZE       512

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* SDHC HAL object */
cyhal_sdhc_t sdhc_obj;

/* Configuration options for the SDHC block */
const cyhal_sdhc_config_t sdhc_config = {ENABLE_LED_CONTROL, LOW_VOLTAGE_SIGNALLING, IS_EMMC, BUS_WIDTH};

/*******************************************************************************
* Function Name: sd_card_is_connected
****************************************************************************//**
*
*  Checks to see if a card is currently connected.
*
* \return bool
*     true - the card is connected, false - the card is removed (not connected).
*
*******************************************************************************/
bool sd_card_is_connected(void)
{
    /* Card detect pin reads 0 when card detected, 1 when card not detected */
    return cyhal_gpio_read(CARD_DETECT) ? false : true;
}

/*******************************************************************************
* Function Name: sd_card_init
********************************************************************************
* Summary:
*  Initialize the SDHC card
*
* Return:
*  CY_RSLT_SUCCESS if successful.
*
*******************************************************************************/
cy_rslt_t sd_card_init(void)
{
    cy_rslt_t result;

    /* Initialize the SD card */
    result = cyhal_sdhc_init(&sdhc_obj, &sdhc_config, CMD, CLK, DAT0, DAT1, DAT2, DAT3, DAT4, DAT5, DAT6, DAT7,
                        CARD_DETECT, IO_VOLT_SEL, CARD_IF_PWREN, CARD_MECH_WRITEPROT, LED_CTL, EMMC_RESET, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: sd_card_sector_size
********************************************************************************
* Summary:
*  Get the sector size of SD card.
*
*******************************************************************************/
uint32_t sd_card_sector_size(void)
{
    return CY_SD_HOST_BLOCK_SIZE;
}

/*******************************************************************************
* Function Name: sd_card_max_sector_num
********************************************************************************
* Summary:
*  Get the SD card maximum number of the sectors.
*
*******************************************************************************/
uint32_t sd_card_max_sector_num(void)
{
    return sdhc_obj.sdxx.context.maxSectorNum;
}

/*******************************************************************************
* Function Name: sd_card_total_mem_bytes
********************************************************************************
* Summary:
*  Get the SD card total memory bytes.
*
*******************************************************************************/
uint64_t sd_card_total_mem_bytes(void)
{
    uint64_t sector_num = sd_card_max_sector_num();
    return (sector_num * sd_card_sector_size());
}

/*******************************************************************************
* Function Name: sd_card_read
********************************************************************************
* Summary:
*  Read data from SD card.
*
* Parameters:
*  address The address to read data from
*  data    Pointer to the byte-array where data read from the device should be stored
*  length  Number of 512 byte blocks to read, updated with the number actually read
*
* Return:
*  CY_RSLT_SUCCESS if successful.
*******************************************************************************/
cy_rslt_t sd_card_read(uint32_t address, uint8_t *data, uint32_t *length)
{
    cy_rslt_t result;

    if(!sd_card_is_connected()) {
        return CY_RSLT_TYPE_ERROR;
    }
    
    result = cyhal_sdhc_read(&sdhc_obj, address, data, (size_t *)length);
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }
    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: sd_card_write
********************************************************************************
* Summary:
*  Write data to SD card.
*
* Parameters:
*  address The address to write data to
*  data    Pointer to the byte-array of data to write to the device
*  length  Number of 512 byte blocks to write, updated with the number actually written
*
* Return:
*  CY_RSLT_SUCCESS if successful.
*******************************************************************************/
cy_rslt_t sd_card_write(uint32_t address, const uint8_t *data, uint32_t *length)
{
    cy_rslt_t result;

    if(!sd_card_is_connected()) {
        return CY_RSLT_TYPE_ERROR;
    }

    result = cyhal_sdhc_write(&sdhc_obj, address, data, (size_t *)length);
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }
    return CY_RSLT_SUCCESS;
}

/* [] END OF FILE */
