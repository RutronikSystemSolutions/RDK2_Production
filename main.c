/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK2_Production
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-11-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_qspi_memslot.h"
#include "stdlib.h"
#include "ff.h"
#include "diskio.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"
#include "cycfg_capsense.h"
#include "led.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY           (1u)     		/* SMIF interrupt priority */
#define PACKET_SIZE             (102400u)  		/* The memory Read/Write packet */
#define TIMEOUT_1_MS            (1000ul) 		/* 1 ms timeout for all blocking functions */
#define QSPI_FREQ            	(48000000ul)	/* The QSPI peripheral clock frequency*/
#define USER_DATA               (0xA0)   		/* User data for memory write and read */
#define USER_DATA2              (0xB0)   		/* User data for memory write and read */
#define PSRAM_ADDR				(0x00)			/* PSRAM memory address*/
#define FLASH_ADDR				(0x40000)		/* Flash memory address*/
#define SECTOR_SIZE				(0x40000)		/* Flash memory sector size*/
#define USBUART_BUFFER_SIZE 	(64u)
#define USBUART_COM_PORT    	(0U)
#define CDC_ENDPOINT     		(0x01)
#define USB_CONNECT_TIMEOUT		(0U)
#define LED_IND_OFF          	(0x03)
#define LED_IND_RED          	(0x01)
#define LED_IND_GREEN        	(0x02)
#define CAPSENSE_INTR_PRIORITY  (7U)
#define EZI2C_INTR_PRIORITY     (6U)
#define RS485_BAUDRATE			(115200U)
#define PDO0 					0x0201905A /* PDO 0 : 5V, 900mA*/
#define PDO1 					0x0002D0C8 /*PDO 1 : 9V, 2000mA*/

/***************************************************************************
* Global variables and functions
***************************************************************************/
void handle_error(void);
void Isr_SMIF(void);
cy_rslt_t Init_SMIF(void);
void status_led (uint32_t status);
void CANFD_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_rx_buffer_t* pstcCanFDmsg);
void CanfdInterruptHandler(void);
static void usb_high_isr(void);
static void usb_medium_isr(void);
static void usb_low_isr(void);
static uint32_t initialize_capsense(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
static void capsense_callback();
static void process_touch(void);
cy_rslt_t adc_hw_init(void);
void dma_interrupt(void);

/*QSPI PSRAM object*/
cyhal_qspi_t qspi_psram_obj;

/* SMIF configuration parameters */
cy_stc_smif_config_t SMIFConfig =
{
    /* .mode           */ CY_SMIF_NORMAL,      /* Mode of operation */
    /* .deselectDelay  */ 2U,      			/* Minimum duration of SPI deselection */
    /* .rxClockSel     */ CY_SMIF_SEL_INVERTED_INTERNAL_CLK,     /* Clock source for the receiver clock */
    /* .blockEvent     */ CY_SMIF_BUS_ERROR    /* What happens when there is a read
                                                * to an empty RX FIFO or write to a full TX FIFO
                                                */
};

/*QSPI pins configuration structure*/
cyhal_qspi_slave_pin_config_t qspi_psram_pins =
{
		.io[0] = QSPI_IO0,
		.io[1] = QSPI_IO1,
		.io[2] = QSPI_IO2,
		.io[3] = QSPI_IO3,
		.io[4] = NC,
		.io[5] = NC,
		.io[6] = NC,
		.io[7] = NC,
		.ssel = PSRAM_SSEL
};
cyhal_qspi_slave_pin_config_t qspi_flash_pins =
{
		.io[0] = QSPI_IO0,
		.io[1] = QSPI_IO1,
		.io[2] = QSPI_IO2,
		.io[3] = QSPI_IO3,
		.io[4] = NC,
		.io[5] = NC,
		.io[6] = NC,
		.io[7] = NC,
		.ssel = FLASH_SSEL
};

FATFS  SDFatFS;
TCHAR SDPath = DEV_SD;

const cy_stc_sysint_t irq_cfg =
{
    canfd_0_interrupts0_0_IRQn,
    3UL
};
cy_stc_canfd_context_t canfd_context;
uint32_t bmp_data[16] =
		{
				0x00000000,
				0xDEADBEAF,
				0xCAFEBABE,
				0xFACEFEED,
				0x11111111,
				0x22222222,
				0x33333333,
				0x44444444,
				0x55555555,
				0x66666666,
				0x77777777,
				0x88888888,
				0x99999999,
				0xAAAAAAAA,
				0xBBBBBBBB,
				0xCCCCCCCC,
		};

/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 5U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 6U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 7U,
};

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_cdc_context_t    usb_cdcContext;

/*CapSense global variables*/
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;
volatile bool capsense_scan_complete = false;

/*RTC Peripheral Object*/
cyhal_rtc_t rtc_obj;

/*RS485 Peripheral Variables*/
cyhal_uart_t rs485_obj;
uint32_t actual_baud;
cyhal_uart_cfg_t mb_uart_config =
		{
				.data_bits = 8,
				.stop_bits = 1,
				.parity = CYHAL_UART_PARITY_NONE,
				.rx_buffer = NULL,
				.rx_buffer_size = 0,
		};

/*RS485 Tx-Enable pin configuraton*/
const cy_stc_gpio_pin_config_t MB_UART_CTS_config =
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = HSIOM_SEL_ACT_6,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_1_2,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cyhal_resource_inst_t MB_UART_CTS_obj =
{
	.type = CYHAL_RSC_GPIO,
	.block_num = 3U,
	.channel_num = 3U,
};

/* This flag set in the DMA interrupt handler */
volatile bool dmaIntrTriggered = false;
/* DMA Transfer complete/error flags sent in DMA interrupt Handler*/
volatile uint8_t adc_dma_error;   /* ADCDma error flag */
volatile uint8_t adc_dma_done;    /* ADCDma done flag */
/* DMA interrupt configuration structure */
/* Source is set to DW 0 and Priority as 7 */
const cy_stc_sysint_t intRxDma_cfg =
{
        .intrSrc      = cpuss_interrupts_dw0_28_IRQn,
        .intrPriority = 7
};
/* Buffer to store data from SAR using DMA */
int16_t sar_buffer[12] = {0};

int main(void)
{
    cy_rslt_t result;
    cy_en_smif_status_t result_smif;
    uint8_t mem_buffer[PACKET_SIZE];    /* Buffer to read memory in burst mode */
    uint8_t pttrn=USER_DATA;
    uint32_t index;
    FRESULT fs_result;
    FIL SDFile;       /* File object for SD */
    uint8_t *memory = NULL;
    cy_en_usb_dev_ep_state_t epState;
    uint32_t count;
    uint8_t widget_id;
    struct tm new_date_time = {0};
    struct tm old_date_time = {0};
    uint8_t rs485_byte;
    volatile int32_t arduino_analog[6] = {0};
    uint8_t  volatile *Xip_Buffer_8;     /* Xip  buffer for 8-bit data */

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    __enable_irq();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    printf("\x1b[2J\x1b[;H");
    printf("RDK2 Factory Test Application.\r\n\r\n");

    /*Initialize LEDs*/
    result = initialize_led();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    printf("PWM LED Initialized Successfully.\r\n");

    /*CapSense Initialization*/
    initialize_capsense_tuner();
    result = initialize_capsense();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    widget_id = CY_CAPSENSE_LINEARSLIDER_WDGT_ID;
    Cy_CapSense_SetupWidget(widget_id, &cy_capsense_context);
    Cy_CapSense_Scan(&cy_capsense_context);
    printf("CapSense Initialized Successfully.\r\n");

    /* Initialize RTC */
    result = cyhal_rtc_init(&rtc_obj);
    if (CY_RSLT_SUCCESS != result)
    {handle_error();}
    printf("RTC Initialized Successfully.\r\n");

	/* RS485 MODBUS UART Initialization*/
	result = cyhal_uart_init(&rs485_obj, RS485_TX, RS485_RX, NC, NC, NULL, &mb_uart_config);
	if(result != CY_RSLT_SUCCESS)
	{handle_error();}
	result = cyhal_uart_set_baud(&rs485_obj, RS485_BAUDRATE, &actual_baud);
	if(result != CY_RSLT_SUCCESS)
	{handle_error();}
	result = Cy_GPIO_Pin_Init(GPIO_PRT6, 3U, &MB_UART_CTS_config);
	if(result != CY_RSLT_SUCCESS)
	{handle_error();}
	printf("RS485 Initialized Successfully.\r\n");

	/*Indicate the beginning of the test*/
	status_led (LED_IND_RED);

    /* Initialize the USB device */
    Cy_USB_Dev_Init(USB_DEV_HW, &USB_DEV_config, &usb_drvContext, &usb_devices[0], &usb_devConfig, &usb_devContext);

    /* Initialize the CDC Class */
    Cy_USB_Dev_CDC_Init(&usb_cdcConfig, &usb_cdcContext, &usb_devContext);

    /* Initialize the USB interrupts */
    Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
    Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
    Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);

    /* Enable the USB interrupts */
    NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

    /*Enable USB Driver*/
    Cy_USB_Dev_Connect(false, USB_CONNECT_TIMEOUT, &usb_devContext);
    printf("USB CDC Initialized Successfully.\r\n");

    /*Initialize QSPI PSRAM/Flash Interface*/
    result = cyhal_qspi_init(&qspi_psram_obj, QSPI_CLK, &qspi_psram_pins, QSPI_FREQ, 0, NULL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_qspi_slave_configure(&qspi_psram_obj, &qspi_flash_pins);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initializes SMIF block*/
    result = Init_SMIF();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("SMIF initialization failure!\r\n");
    	handle_error();
    }

	/*Write the data to the PSRAM*/
    pttrn = USER_DATA;
	memset(mem_buffer, pttrn, PACKET_SIZE);
	result_smif = Cy_SMIF_MemWrite( qspi_psram_obj.base, &APS6404L_3SQR_ZR_SlaveSlot_0, PSRAM_ADDR, mem_buffer, PACKET_SIZE, &qspi_psram_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("PSRAM write error.");
    	{handle_error();}
    }

    /*Read the data from the PSRAM*/
    memset(mem_buffer, 0x00, PACKET_SIZE);
    result_smif = Cy_SMIF_MemRead( qspi_psram_obj.base, &APS6404L_3SQR_ZR_SlaveSlot_0, PSRAM_ADDR, mem_buffer, PACKET_SIZE, &qspi_psram_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("PSRAM read error.");
    	{handle_error();}
    }

    /*Check the data consistency*/
    for(index=0; index < PACKET_SIZE; index++)
    {
    	if( mem_buffer[index] != pttrn)
    	{
    		printf("PSRAM read error.");
    		handle_error();
    	}
    }
    printf("PSRAM check: PASS.\r\n");
#if 1
    /*Select the Flash as active device*/
    result = cyhal_qspi_select_active_ssel(&qspi_psram_obj, FLASH_SSEL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Erase the Flash memory sector*/
    result_smif =  Cy_SMIF_MemEraseSector(qspi_psram_obj.base, &S25HL512T_SlaveSlot_1, FLASH_ADDR, SECTOR_SIZE, &qspi_psram_obj.context);
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH erase error.");
    	{handle_error();}
    }

	/*Write the data to the Flash*/
    pttrn = USER_DATA2;
	memset(mem_buffer, pttrn, PACKET_SIZE);
	result_smif = Cy_SMIF_MemWrite( qspi_psram_obj.base, &S25HL512T_SlaveSlot_1, FLASH_ADDR, mem_buffer, PACKET_SIZE, &qspi_psram_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH write error.");
    	{handle_error();}
    }

    /*Read the data from the Flash*/
    memset(mem_buffer, 0x00, PACKET_SIZE);
    result_smif = Cy_SMIF_MemRead( qspi_psram_obj.base, &S25HL512T_SlaveSlot_1, FLASH_ADDR, mem_buffer, PACKET_SIZE, &qspi_psram_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("FLASH read error.");
    	{handle_error();}
    }

    /*Check the data consistency*/
    for(index=0; index < PACKET_SIZE; index++)
    {
    	if( mem_buffer[index] != pttrn)
    	{
    		printf("FLASH read error.");
    		handle_error();
    	}
    }
    printf("FLASH check: PASS.\r\n");

    /*Entering XIP mode*/
    Cy_SMIF_SetMode(qspi_psram_obj.base, CY_SMIF_MEMORY);
    /* Initilaize the Xip buffer for memory mapped accesses */
    Xip_Buffer_8 =(uint8_t *)APS6404L_3SQR_ZR_SlaveSlot_0.baseAddress; /* Assign the SMIF base address to Xip buffer*/
    if(*Xip_Buffer_8 == USER_DATA)
    {
    	printf("PSRAM XIP Mode check: PASS.\r\n");
    }
    else
    {
    	printf("PSRAM XIP Mode check: FAIL.\r\n");
    }
    Xip_Buffer_8 =(uint8_t *)(S25HL512T_SlaveSlot_1.baseAddress + FLASH_ADDR); /* Assign the SMIF base address to Xip buffer*/
    if(*Xip_Buffer_8 == USER_DATA2)
    {
    	printf("FLASH XIP Mode check: PASS.\r\n");
    }
    else
    {
    	printf("FLASH XIP Mode check: FAIL.\r\n");
    }


#endif

    /*Mount the uSD card and write with a new test file*/

    fs_result = f_mount(&SDFatFS, &SDPath, 1);
    if (fs_result == FR_OK)
    {
        fs_result = f_unlink("TEST.TXT");
        if ((fs_result == FR_NO_FILE) || (fs_result == FR_OK) )
        {
        	fs_result = f_open(&SDFile, "TEST.TXT", FA_WRITE | FA_CREATE_ALWAYS);
        	if (fs_result == FR_OK)
        	{
        		memset(mem_buffer, 'A', PACKET_SIZE);
        		UINT bw;
        		f_write(&SDFile, mem_buffer, PACKET_SIZE, &bw);
        		f_close(&SDFile);
        	}
        	else
        	{
        		printf("Could not create file.\r\n");
        		goto sd_test_skip;
        	}
         }
        else
        {
          printf("Error deleting file.\r\n");
          goto sd_test_skip;
        }
    }
    else
    {
    	printf("Could not mount the uSD card.\r\n");
    	goto sd_test_skip;
    }
	/*Open and read test file*/
    fs_result = f_open(&SDFile, "TEST.TXT", FA_READ);
    {
      if(fs_result == FR_OK)
      {
        UINT br = 0;
        memset(mem_buffer, 0x00, PACKET_SIZE);
        f_read(&SDFile, mem_buffer, PACKET_SIZE, &br);
        f_close(&SDFile);
        if((br==0) || (br<PACKET_SIZE))
        {
          printf("Could not read file.\r\n");
          goto sd_test_skip;
        }
      }
      else
      {
        printf("Could not open file.\r\n");
        goto sd_test_skip;
      }
    }
    /*Check the data in the memory*/
    memory = mem_buffer;
    for(uint32_t i = 0; i < PACKET_SIZE; i++)
    {
      if(*memory != 'A')
      {
        printf("uSD data does not match!.\r\n");
        handle_error();
      }
      memory++;
    }
    memset(mem_buffer, 0x00, PACKET_SIZE);
    printf("uSD Card Check: PASS.\r\n");

    sd_test_skip:
	/*CAN FD Test*/
    /*Initialize CANFD Driver Stand-By pin */
    result = cyhal_gpio_init( CANFD_STB, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    if(CY_CANFD_SUCCESS != Cy_CANFD_Init (CANFD0, 0, &CAN_FD_config, &canfd_context))
    { handle_error();}
    /* Enables the configuration changes to set Test mode */
    Cy_CANFD_ConfigChangesEnable(CANFD0, 0);
    /* Sets the Test mode configuration */
    Cy_CANFD_TestModeConfig(CANFD0, 0, CY_CANFD_TEST_MODE_EXTERNAL_LOOP_BACK);
    /* Disables the configuration changes */
    Cy_CANFD_ConfigChangesDisable(CANFD0, 0);
    /* Hook the interrupt service routine and enable the interrupt */
    (void) Cy_SysInt_Init(&irq_cfg, &CanfdInterruptHandler);
    NVIC_EnableIRQ(canfd_0_interrupts0_0_IRQn);
    /* Prepare data to send */
    memcpy(CAN_FD_dataBuffer_0, bmp_data, 64);
    /* Sends the prepared data using tx buffer 1  */
    Cy_CANFD_UpdateAndTransmitMsgBuffer(CANFD0, 0u, &CAN_FD_txBuffer_0, 0u, &canfd_context);
    CyDelay(1000);

    /*Configure the ADC hardware*/
    cyhal_gpio_write(CANFD_STB, true);
    result = adc_hw_init();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    /* Start the TCPWM Timer triggering the ADC*/
    Cy_TCPWM_TriggerStart_Single(ADC_TIMER_HW, ADC_TIMER_NUM);
    CyDelay(1000);
	arduino_analog[0] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[0])/1000;
	arduino_analog[1] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[1])/1000;
	arduino_analog[2] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[2])/1000;
	arduino_analog[3] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[3])/1000;
	arduino_analog[4] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[4])/1000;
	arduino_analog[5] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[5])/1000;
	printf("Arduino ADC Input Values:\r\n");
	printf("A0: %4ldmV, A1: %4ldmV, A2: %4ldmV, A3: %4ldmV, A4: %4ldmV, A5: %4ldmV\r\n\r\n"
												,(long int)arduino_analog[0]
												,(long int)arduino_analog[1]
												,(long int)arduino_analog[2]
												,(long int)arduino_analog[3]
												,(long int)arduino_analog[4]
												,(long int)arduino_analog[5]);

	/*Indicate the end of the test*/
	status_led (LED_IND_GREEN);

    for (;;)
    {
    	/*The USB echoing*/
		if(usb_devContext.state == CY_USB_DEV_CONFIGURED)
		{
			/*Check if device is ready*/
			epState = Cy_USBFS_Dev_Drv_GetEndpointState(USB_DEV_HW, CDC_ENDPOINT, &usb_drvContext);
			if ((CY_USB_DEV_EP_IDLE == epState) || (CY_USB_DEV_EP_COMPLETED == epState))
			{
				/* Check if host sent any data */
		        if (Cy_USB_Dev_CDC_IsDataReady(USBUART_COM_PORT, &usb_cdcContext))
		        {
		            /* Get number of bytes */
		            count = Cy_USB_Dev_CDC_GetAll(USBUART_COM_PORT, mem_buffer, PACKET_SIZE, &usb_cdcContext);

		            if (0u != count)
		            {
		                /* Wait until component is ready to send data to host */
		                while (0u == Cy_USB_Dev_CDC_IsReady(USBUART_COM_PORT, &usb_cdcContext))
		                {
		                }

		                /* Send data back to host */
		                Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, mem_buffer, count, &usb_cdcContext);

		                /* If the last sent packet is exactly the maximum packet
		                *  size, it is followed by a zero-length packet to assure
		                *  that the end of the segment is properly identified by
		                *  the terminal.
		                */
		                if (USBUART_BUFFER_SIZE == count)
		                {
		                    /* Wait until component is ready to send data to PC. */
		                    while (0u == Cy_USB_Dev_CDC_IsReady(USBUART_COM_PORT, &usb_cdcContext))
		                    {
		                    }

		                    /* Send zero-length packet to PC. */
		                    Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, NULL, 0u, &usb_cdcContext);
		                }
		            }
		        }
			}
		}

    	/*Check if the CSD slider scan is complete*/
        if (capsense_scan_complete)
        {
        	/*Reset scan status flag*/
        	capsense_scan_complete = false;

        	/*Process the CSD slider */
        	Cy_CapSense_ProcessWidget(widget_id, &cy_capsense_context);

        	/*Update the content for the Capsense Tuner*/
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /*Scan the CSD slider */
            Cy_CapSense_SetupWidget(widget_id, &cy_capsense_context);
            Cy_CapSense_Scan(&cy_capsense_context);
        }

        /*Update the LED intensity*/
        process_touch();

        /*Show the system time*/
        result = cyhal_rtc_read(&rtc_obj, &new_date_time);
        if (CY_RSLT_SUCCESS == result)
        {
        	if(new_date_time.tm_sec != old_date_time.tm_sec)
        	{
        		memset(mem_buffer, 0x00, PACKET_SIZE);
                strftime((char*)mem_buffer, PACKET_SIZE, "%c", &new_date_time);
                printf("\x1b[1F\x1b[2K");
                printf("%s\r\n", mem_buffer);
                old_date_time.tm_hour = new_date_time.tm_hour;
                old_date_time.tm_isdst = new_date_time.tm_isdst;
                old_date_time.tm_mday = new_date_time.tm_mday;
                old_date_time.tm_min = new_date_time.tm_min;
                old_date_time.tm_mon = new_date_time.tm_mon;
                old_date_time.tm_sec = new_date_time.tm_sec;
                old_date_time.tm_wday = new_date_time.tm_wday;
                old_date_time.tm_yday = new_date_time.tm_yday;
                old_date_time.tm_year = new_date_time.tm_year;
        	}
        }

        /*The RS485 echoing*/
        result = cyhal_uart_getc(&rs485_obj, &rs485_byte, 1U);
        if (CY_RSLT_SUCCESS == result)
        {
        	/*Send the received byte back*/
        	(void) cyhal_uart_putc(&rs485_obj, rs485_byte);
        }

    }
}


/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(qspi_psram_obj.base, &qspi_psram_obj.context);
}

/*******************************************************************************
* Function Name: Initialize_SMIF
********************************************************************************
*
* This function initializes the SMIF block
*
*******************************************************************************/
cy_rslt_t Init_SMIF(void)
{
	cy_rslt_t result = CY_RSLT_SUCCESS;
	cy_en_smif_status_t smifStatus;
	cy_stc_sysint_t smifIntConfig =
	{
			.intrSrc = 2u,
			.intrPriority = SMIF_PRIORITY
	};

	cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);
	if(0u != intrStatus)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	smifStatus = Cy_SMIF_Init(qspi_psram_obj.base, &SMIFConfig, TIMEOUT_1_MS, &qspi_psram_obj.context);
	if(0u != smifStatus)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	/* Configure slave select and data select. These settings depend on the pins
	 * selected in the Device and QSPI configurators.
	 */
	Cy_SMIF_SetDataSelect(qspi_psram_obj.base, qspi_psram_obj.slave_select, APS6404L_3SQR_ZR_SlaveSlot_0.dataSelect);
	Cy_SMIF_Enable(qspi_psram_obj.base, &qspi_psram_obj.context);
	smifStatus = Cy_SMIF_Memslot_Init(qspi_psram_obj.base, (cy_stc_smif_block_config_t *)&smifBlockConfig, &qspi_psram_obj.context);
	if(0u != intrStatus)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	Cy_SMIF_SetMode(qspi_psram_obj.base, CY_SMIF_NORMAL);
	NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Enable the SMIF interrupt */

	return result;
}

/*******************************************************************************
* Function Name: status_led
********************************************************************************
*
* This function drives the red/green status of RGB
* *
* \param status - The led status (RGB_GLOW_OFF, RGB_GLOW_RED, or RGB_GLOW_GREEN)
*
*******************************************************************************/
void status_led (uint32_t status)
{
	led_data_t led_data = {LED_ON, LED_MAX_BRIGHTNESS};

	if (status == LED_IND_RED)
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_ON);
		led_data.state = LED_OFF;
		update_led_state(&led_data);
	}
	else if (status == LED_IND_GREEN)
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
		led_data.state = LED_ON;
		update_led_state(&led_data);
	}
	else
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
		led_data.state = LED_OFF;
		update_led_state(&led_data);
	}
}

/* CANFD reception callback */
void CANFD_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_rx_buffer_t* pstcCanFDmsg)
{
    (void)bRxFifoMsg;
    (void)u8MsgBufOrRxFifoNum;

    /* Only for data frames */
    if(0 == pstcCanFDmsg->r0_f->rtr)
    {
    	cyhal_gpio_toggle(LED1);
    	memset(bmp_data, 0x00, sizeof(bmp_data));
    	memcpy(bmp_data, pstcCanFDmsg->data_area_f, 64);

    	/*Print the data received from the loop-back*/
    	printf("\r\nCAN FD Data received in loop back mode:\r\n");
        for(uint32_t index = 0; index < 16; index++)
        {
            printf("0x%08X ", (unsigned int)bmp_data[index]);

            if(0u == ((index + 1) % 4))
            {
                printf("\r\n");
            }
        }

        /*Increase the data header*/
        CAN_FD_dataBuffer_0[CAN_FD_DATA_0] += 1;
    }

    printf("\r\n");
}

/* CANFD interrupt handler */
void CanfdInterruptHandler(void)
{
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD0, 0, &canfd_context);
}

/***************************************************************************
* Function Name: usb_high_isr
********************************************************************************
* Summary:
*  This function processes the high priority USB interrupts.
*
***************************************************************************/
static void usb_high_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseHi(USB_DEV_HW),&usb_drvContext);
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function processes the medium priority USB interrupts.
*
***************************************************************************/
static void usb_medium_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseMed(USB_DEV_HW),&usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function processes the low priority USB interrupts.
*
**************************************************************************/
static void usb_low_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseLo(USB_DEV_HW),&usb_drvContext);
}

/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_rslt_t result;

    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = false;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;

    result = cyhal_ezi2c_init(&sEzI2C, ARDU_SDA, ARDU_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
        {
            .intrSrc = CAPSENSE_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
        };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E, capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CAPSENSE_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    cy_stc_capsense_touch_t *slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status;
    bool led_update_req = false;
    static uint16_t slider_pos_prev;
    static led_data_t led_data = {LED_ON, LED_MAX_BRIGHTNESS};


    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect the new touch on slider */
    if ((0 != slider_touch_status) &&
        (slider_pos != slider_pos_prev))
    {
        led_data.brightness = (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER_WDGT_ID].xResolution;
        led_update_req = true;
    }

    /* Update the LED state if requested */
    if (led_update_req)
    {
        update_led_state(&led_data);
    }

    /* Update previous touch status */
    slider_pos_prev = slider_pos;
}

cy_rslt_t adc_hw_init(void)
{
	cy_en_tcpwm_status_t tcpwm_res;
	cy_en_sysanalog_status_t aref_res;
	cy_en_sar_status_t sar_res;
	cy_en_dma_status_t dma_res;

	/*TCPWM*/
	tcpwm_res = Cy_TCPWM_Counter_Init(ADC_TIMER_HW, ADC_TIMER_NUM, &ADC_TIMER_config);
	if(tcpwm_res == CY_TCPWM_SUCCESS)
	{
		Cy_TCPWM_Counter_Enable(ADC_TIMER_HW, ADC_TIMER_NUM);
	}
	else {goto return_err;}

	/*VREF*/
	aref_res = Cy_SysAnalog_Init(&ADC_VREF_config);
	if(aref_res == CY_SYSANALOG_SUCCESS)
	{
		Cy_SysAnalog_Enable();
	}
	else {goto return_err;}

	/*SAR*/
	sar_res = Cy_SAR_Init(SAR_ADC_HW, &SAR_ADC_config);
	if(sar_res == CY_SAR_SUCCESS)
	{
		Cy_SAR_Enable(SAR_ADC_HW);
	}
	else {goto return_err;}

	/*DMA*/
	/* Initialize descriptor 0 */
	dma_res = Cy_DMA_Descriptor_Init(&cpuss_0_dw0_0_chan_28_Descriptor_0,&cpuss_0_dw0_0_chan_28_Descriptor_0_config);
	if(dma_res != CY_DMA_SUCCESS) {goto return_err;}

	/* Initialize the channel and associate the descriptor to it */
	dma_res = Cy_DMA_Channel_Init(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL,&cpuss_0_dw0_0_chan_28_channelConfig);
	if(dma_res != CY_DMA_SUCCESS) {goto return_err;}

	/*Get the pointer to the SAR measurement data*/
	uint32_t *sar_data_ptr = (uint32_t*)&SAR_CHAN_RESULT(SAR_ADC_HW, 0);

    /* Set DMA Source and Destination address */
    Cy_DMA_Descriptor_SetSrcAddress(&cpuss_0_dw0_0_chan_28_Descriptor_0, sar_data_ptr);
    Cy_DMA_Descriptor_SetDstAddress(&cpuss_0_dw0_0_chan_28_Descriptor_0, sar_buffer);

    /*Set DMA Descriptor */
    Cy_DMA_Channel_SetDescriptor(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL, &cpuss_0_dw0_0_chan_28_Descriptor_0);

    /* Initialize and enable the interrupt from SAR DMA */
    Cy_SysInt_Init(&intRxDma_cfg, &dma_interrupt);
    NVIC_EnableIRQ((IRQn_Type)intRxDma_cfg.intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable DMA Channel and DMA Block to start descriptor execution process */
    Cy_DMA_Channel_Enable(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
    Cy_DMA_Enable(cpuss_0_dw0_0_chan_28_HW);

	return CY_RSLT_SUCCESS;

	return_err:
	return CY_RSLT_TYPE_ERROR;
}

void dma_interrupt(void)
{
    dmaIntrTriggered = true;

    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL))
    {
        adc_dma_done = 1;
    }
    else if((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW,cpuss_0_dw0_0_chan_28_CHANNEL)) &&
                                                (CY_DMA_INTR_CAUSE_CURR_PTR_NULL !=
                                                Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL)))
    {
        /* DMA error occurred while ADC operations */
        adc_dma_error = Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
    }

    /* Clear SAR DMA interrupt */
    Cy_DMA_Channel_ClearInterrupt(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
 }

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
