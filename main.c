/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 6 MCU: CapSense Custom Scan
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
 * (c) 2020-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
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
