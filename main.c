/******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Master using HAL APIs. The I2C master sends the
* command packets to the I2C slave to control an user LED.
*
* Update 6-Nov-2021 (DJJW):
*		- Created new project (ISSI_LED_Matrix)
*			- Used Quick Launch "New Application"
*		- Added I2C Master code from github (https://github.com/Infineon/mtb-example-psoc6-i2c-master.git)
*		- Adapting ISSI S31FL3741A-QFLS4 driver from https://git.alt-tek.com/nathan/qmk-fw
*		- Instantiating and initializing I2C Master (MDriver) with HAL
*			- Must disable the MDriver SCB peripheral in Device Configurator to use HAL
*
* Update 9-Nov-2021 (DJJW):
*		- Abandoning qmk S31FL3741A driver (but borrowing #defines)
*		- Replaced/added base driver functions
*		- Adding turn-on effects
*
* Update 14-Nov-2021 (DJJW):
*		- completed s31fl3741a init/writeLED/readLED/toggleLED functions
*			- added macros for set/clear LED
*			- uses hardware-independent calls to i2c_transmit and i2c_receive
*		- completed i2c_master .c and .h files
*			- this is where hardware-specific i2c transmit and receive functions are defined
*			- uses HAL I2C SCB object (MDriverI2C)
*		- increased I2C bus speed to 1MHz (tested ok)
*		- added low-level s31fl3741a unlock, writeRegister and readRegister functions
*			- modified writeLED/readLED/toggleLED functions to use the new low-level functions
*
* Update 15-Nov-2021 (DJJW):
*		- completed more of the s31fl3741a functions
*		- cleaned up s31fl3741a init using helper functions
*		- cleaned up runEffects using newly created functions
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "resource_map.h"
#include "s31fl3741a.h"

/***************************************
*            Constants
****************************************/
/* enable or disable features */
#define DISCOVER_I2C_SLAVES		(1u)

/* I2C bus frequency */
#define I2C_FREQ                (1000000UL)

/* define MDriver I2C Master pins on hardware */
#define MDriverI2C_SCL			(P2_0)
#define MDriverI2C_SDA			(P2_1)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_rslt_t runEffects(void);

/***************************************
*          Global Variables
****************************************/
/* HAL I2C SCB object */
cyhal_i2c_t MDriverI2C;
cyhal_i2c_cfg_t MDriverI2C_cfg;


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*   1. I2C Master sends command packet to the slave
*   2. I2C Master reads the response packet to generate the next command
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    uint8_t buffer[8];

    /* Set up the device based on configurator selections */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                                  CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************************************************\r\n");
    printf("PSoC 6 MCU I2C Master - discover slave addresses\r\n");
    printf("************************************************\r\n\n");


    /* Configure user LED */
    printf(">> Configuring user LED..... ");
    result = cyhal_gpio_init( CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("Done\r\n");

	/* djjw - initialize and configure I2C Master SCB (borrowed from github I2C Master code example) */
    MDriverI2C_cfg.is_slave = false;
    MDriverI2C_cfg.address = 0;
    MDriverI2C_cfg.frequencyhal_hz = I2C_FREQ;

	result = cyhal_i2c_init(&MDriverI2C, MDriverI2C_SDA, MDriverI2C_SCL, NULL);
	result = cyhal_i2c_configure(&MDriverI2C, &MDriverI2C_cfg);

	if (CYRET_SUCCESS != result)
	{
		/* Halt the CPU if CapSense initialization failed */
		CY_ASSERT(0);
	}

	/* djjw - initialize ISSI IS31FL3741A LED Driver */
	cyhal_gpio_init(P1_5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /* disable while initializing */

	result = S31FL3741_init();

	cyhal_gpio_write(P1_5, 1); /* enable LED matrix */

    /* Enable interrupts */
    __enable_irq();

#if DISCOVER_I2C_SLAVES

    printf("Starting slave discovery .....\r\n\n");

	for(uint8_t i = 1; i < 128; i++)
	{
		if(CY_RSLT_SUCCESS == cyhal_i2c_master_read(&MDriverI2C, i, buffer, 1, 100, true))
		{
			printf("Found device at address 0x%x\r\n", i);
		}
	}
	printf("\nDiscovery complete\r\n\n");

#endif

	result = runEffects();

    for (;;)
    {

    }
}


cy_rslt_t runEffects(void)
{
	cy_rslt_t result;

	/* turn off all LEDs via global current control */
	result = S31FL3741_setGlobalCurrent(0x00);

	/* set all LEDs to max PWM value */
	result = setAllMatrix();

	/* fade in LEDs using global current control */
	for(uint16_t i = 0; i < 256; i++)
	{
		result = S31FL3741_setGlobalCurrent(i);
		cyhal_system_delay_ms(6);
	}

	cyhal_system_delay_ms(1500);

	/* fade out LEDs using global current control */
	for(uint16_t i = 255; i > 0; i--)
	{
		S31FL3741_setGlobalCurrent(i);
		cyhal_system_delay_ms(6);
	}

	/* extinguish all LEDs */
	result = clearAllMatrix();

	/* reset Global Current Control Register to 0xFF */
	result = S31FL3741_setGlobalCurrent(0xFF);

	/* cycle through lower LEDs */
	for(uint16_t i = 0; i < PAGE_ZERO_BOUNDARY; i++)
	{
		result = writeLED(i, 0xFF);
		cyhal_system_delay_ms(20);
	}

	cyhal_system_delay_ms(2000);

	/* cycle through upper LEDs */
	for(uint16_t i = 0; i < MAX_LEDS - PAGE_ZERO_BOUNDARY; i++)
	{
		result = writeLED(i + PAGE_ZERO_BOUNDARY, 0xFF);
		cyhal_system_delay_ms(20);
	}

	cyhal_system_delay_ms(4000);

	/* extinguish all LEDs */
	result = clearAllMatrix();

	cyhal_system_delay_ms(500);

	return result;
}

/* [] END OF FILE */
