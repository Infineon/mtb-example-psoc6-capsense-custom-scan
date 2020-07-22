/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 6 MCU: CapSense Custom Scan
 *              Example for ModusToolbox.
 *
 * Related Document: See Readme.md
 *
 *******************************************************************************
 * (c) (2020), Cypress Semiconductor Corporation. All rights reserved.
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
 *******************************************************************************/

#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "led.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define CAPSENSE_INTR_PRIORITY  (7u)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static cy_status initialize_capsense(void);
static void process_touch(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
void start_sample_callback_function(cy_stc_active_scan_sns_t * ptrActiveScan);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
cyhal_ezi2c_t EZI2C;
cyhal_ezi2c_slave_cfg_t EZI2C_sub_cfg;
cyhal_ezi2c_cfg_t EZI2C_cfg;


/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CapSense
 *  - initialize tuner communication
 *  - scan touch input continuously and update the LED accordingly.
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_status status;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    initialize_led();

    /*Register the Start_Sample callback for CapSense*/
    Cy_CapSense_RegisterCallback(CY_CAPSENSE_START_SAMPLE_E,
            start_sample_callback_function,
            &cy_capsense_context);
    initialize_capsense_tuner();
    status = initialize_capsense();

    if(CYRET_SUCCESS != status)
    {
        /* Halt the CPU if CapSense initialization failed */
        CY_ASSERT(0);
    }

    /* Start first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for(;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Start next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }
    }
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
    cy_stc_capsense_touch_t* slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status ;
    uint32_t brightness;
    bool led_update_req = false;

    static uint16_t slider_pos_prev;

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
            CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect the new touch on slider */
    if((0 != slider_touch_status) &&
            (slider_pos != slider_pos_prev))
    {
        brightness = (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        led_update_req = true;
    }

    /* Update the LED state if requested */
    if(led_update_req)
    {
        update_led(brightness);
    }

    /* Update previous touch status */
    slider_pos_prev = slider_pos;
}


/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense and configure the CapSense
 *  interrupt.
 *
 *******************************************************************************/
static cy_status initialize_capsense(void)
{
    cy_status status;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
    {
            .intrSrc = CYBSP_CSD_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if(CYRET_SUCCESS == status)
    {
        /* Initialize CapSense interrupt */
        Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
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
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
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
    /* EZI2C Sub Configuration */
    EZI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    EZI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    EZI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    EZI2C_sub_cfg.slave_address = 8u;

    /* EZI2C Configuration */
    EZI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    EZI2C_cfg.enable_wake_from_sleep = true;
    EZI2C_cfg.slave1_cfg = EZI2C_sub_cfg;
    EZI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    EZI2C_cfg.two_addresses = false;

    /* Initialize EZI2C */
    cyhal_ezi2c_init( &EZI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &EZI2C_cfg);
}


/*******************************************************************************
 * Function Name: start_sample_callback_function
 ********************************************************************************
 * Summary:
 *  Callback function for the CAPSENSE_START_SAMPLE event. The callback will be
 *  executed before each sensor scan triggering.
 *  The pin state of the slider sensors are changed in this function. The
 *  sensors adjacent to the sensor being scanned are connected to shield. Other
 *  sensors are connected to ground.
 *
 * Parameter:
 *  cy_stc_active_scan_sns_t: Pointer to the current active sensor structure
 *
 *******************************************************************************/
void start_sample_callback_function(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    /* Variable to loop through every sensor in the widget*/
    uint8 sensorIndex;

    /* Process widgets only if the current widget being scanned is the slider */
    if(ptrActiveScan->widgetIndex == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
    {
        for(sensorIndex = 0; sensorIndex < ptrActiveScan->ptrWdConfig->numSns; sensorIndex++)
        {
            if(sensorIndex != ptrActiveScan->sensorIndex)
            {
                if((sensorIndex == (ptrActiveScan->sensorIndex - 1)) ||
                        (sensorIndex == (ptrActiveScan->sensorIndex + 1)))
                {
                    /* If the sensor is adjacent to the sensor being scanned,
                     * configure it as shield.
                     */
                    Cy_CapSense_SetPinState(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
                            sensorIndex, CY_CAPSENSE_SHIELD,
                            &cy_capsense_context);
                }
                else
                {
                    /* If the sensor is not adjacent to the sensor being
                     * scanned, connect it to ground.
                     */
                    Cy_CapSense_SetPinState(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
                            sensorIndex, CY_CAPSENSE_GROUND,
                            &cy_capsense_context);
                }
            }
        }
    }
}


/* [] END OF FILE */
