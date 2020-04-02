/******************************************************************************
* File Name: resource_map.h
*
* Description: This file gives the I2C SCL and SDA pin map for all the supported
* kits.
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
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
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#ifndef RESOURCE_MAP_H_
#define RESOURCE_MAP_H_

#define sI2C_SCL                    (CYBSP_I2C_SCL)
#define sI2C_SDA                    (CYBSP_I2C_SDA)

#if defined (TARGET_CY8CPROTO_062S3_4343W)      /* CY8CPROTO-063S2-4343W kit */
    #define mI2C_SCL                (P5_0)
    #define mI2C_SDA                (P5_1)
#elif defined (TARGET_CYW9P62S1_43012EVB_01)    /* CYW9P62S1_43012EVB_01 kit */
    #define mI2C_SCL                (P0_2)
    #define mI2C_SDA                (P0_3)
#else
    #define mI2C_SCL                (P9_0)
    #define mI2C_SDA                (P9_1)
#endif

#endif /* RESOURCE_MAP_H_ */
