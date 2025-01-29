/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for PSOC4 HVMS Challenge-Response
*              watchdog timer example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* LED blink interval */
#define BLINK_INTERVAL            (250UL)

/* Each limit is defined in the Device Configurator. Please revise those macro
 * if the limits are redefined in the Device Configurator                      */
#define EARLY_LIMIT               (20000UL)
#define WARN_LIMIT                (40000UL)
#define LATE_LIMIT                (80000UL)

/* It defines which limit is violated in operation */
#define FAULT_MODE                (LATE_LIMIT)
#define SYSCLK_DIV_LFCLK          (CY_CFG_SYSCLK_IMO_FREQ_HZ / \
                                   CY_CFG_SYSCLK_LFCLK_FREQ_HZ)

/* when FAULT_MODE is EARLY_LIMIT,
 * the time to wait should be smaller than EARLY_LIMIT */
#if FAULT_MODE == EARLY_LIMIT
#define DIVIDER                   (2u)
#elif FAULT_MODE != EARLY_LIMIT
#define DIVIDER                   (1u)
#endif

/* Used to wait time until violate the limit */
#define DELAY_TO_VIOLATE_LIMIT     (FAULT_MODE * SYSCLK_DIV_LFCLK / DIVIDER)

/*******************************************************************************
* Global Variables
********************************************************************************/
const cy_stc_sysint_t intr_cfg =
{
    .intrSrc = srss_interrupt_srss_IRQn,
    .intrPriority = 3UL
};

/* Global variables to store  */
static uint8_t g_challenge_value;
static uint8_t g_response_value;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void crwdt_isr(void);
static void check_reset_cause(void);
static void initialize_crwdt_interrupt(void);
static void clear_crwdt(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  1. Initializes BSP and CRWDT is initialized in the function with the values
*     configured by Device Configurator.
*  2. Checks whether the reset is caused due to CRWDT and it blinks LEDs twice
*     if it is caused by CRWDT.
*  4. Initialize CRWDT interrupt.
*  5. In the for loop, CR counter is cleared if the response value corresponds
*     to challenge value.
*  6. Wait until the counter exceeds the limit and the interrupt/reset occurs.
*
* Parameters:
*  void
*
* Return:
*  int
*
********************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals.
     * CRWDT is initialized in cybsp_init() if CRWDT
     * is configured by Device Configurator.         */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Check if the reset is caused by CRWDT reset. */
    check_reset_cause();

    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize interrupt service routine and enable interrupt. */
    initialize_crwdt_interrupt();

    /* Wait until CR watchdog up counter reaches early limit */
    while(EARLY_LIMIT < Cy_CRWDT_GetUpCnt());

    /* Clear CR watchdog up counter before for loop */
    clear_crwdt();

    for (;;)
    {
        /* Wait until the counter exceeds the limit. */
        Cy_SysLib_DelayCycles(DELAY_TO_VIOLATE_LIMIT);

        /* Clear CR watchdog up counter.
         * This should not be done in interrupt service routine. */
        clear_crwdt();
    }
}

/*******************************************************************************
* Function Name: crwdt_isr
********************************************************************************
* Summary: The function is CRWDT interrupt service routine.
*          LED toggles if early limit or warn limit is violated.
*
* Parameters:
*  void
*
* Return
*  void
*
********************************************************************************/
static void crwdt_isr(void)
{
    /* Mask interrupt to prevent from further interrupt */
    Cy_CRWDT_SetInterruptMask(true);

    if(true == Cy_CRWDT_GetInterruptStatus())
    {
        /* Clear interrupt */
        Cy_CRWDT_ClearInterrupt();

#if (FAULT_MODE == EARLY_LIMIT)
        /* LED6 blinks if FAULT_MODE is EARLY_LIMIT */
        Cy_GPIO_Inv(CYBSP_USER_LED6_PORT, CYBSP_USER_LED6_PIN);

#elif (FAULT_MODE == WARN_LIMIT)
        /* LED7 blinks if FAULT_MODE is WARN_LIMIT */
        Cy_GPIO_Inv(CYBSP_USER_LED7_PORT, CYBSP_USER_LED7_PIN);

#endif
    }

    /* Unmask interrupt */
    Cy_CRWDT_SetInterruptMask(false);
}

/*******************************************************************************
* Function Name: check_reset_cause
********************************************************************************
* Summary: This function checks whether the reset is CRWDT reset or not.
*          LEDs blink twice if it is CRWDT reset.
*
* Parameters:
*  void
*
* Return
*  void
*
********************************************************************************/
static void check_reset_cause(void)
{
    /* LEDs blink twice if the reset is caused by CRWDT reset. */
    if(CY_SYSLIB_RESET_CRWDT ==\
      (CY_SYSLIB_RESET_CRWDT & Cy_SysLib_GetResetReason()))
    {
        for(int i = 0; i < 4; i ++)
        {
            Cy_GPIO_Inv(CYBSP_USER_LED6_PORT, CYBSP_USER_LED6_PIN);
            Cy_GPIO_Inv(CYBSP_USER_LED7_PORT, CYBSP_USER_LED7_PIN);
            Cy_SysLib_Delay(BLINK_INTERVAL);
        }
    }

    /* Clear reset cause observation register
     * for next time the function is called.  */
    Cy_SysLib_ClearResetReason();
}

/*******************************************************************************
* Function Name: initialize_crwdt_interrupt
********************************************************************************
* Summary: This function initializes CRWDT interrupt.
*
* Parameters:
*  void
*
* Return
*  void
*
********************************************************************************/
static void initialize_crwdt_interrupt(void)
{
    /* Initialize interrupt and interrupt service routine. */
    cy_rslt_t rslt = Cy_SysInt_Init(&intr_cfg, crwdt_isr);

    if (rslt != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable NVIC. */
    NVIC_ClearPendingIRQ(intr_cfg.intrSrc);
    NVIC_EnableIRQ(intr_cfg.intrSrc);

    /* Unmask interrupt to pass it to CPU. */
    Cy_CRWDT_SetInterruptMask(false);

    /* Clear potential interrupt before going back to main routine. */
    Cy_CRWDT_ClearInterrupt();
}

/*******************************************************************************
* Function Name: clear_crwdt
********************************************************************************
* Summary: This function clears CRWDT_UPCNT if the response value corresponds
*          to the expected next value in the LFSR sequence following the value
*          obtained from the challenge value.
*
* Parameters:
*  void
*
* Return
*  void
*
********************************************************************************/
static void clear_crwdt(void)
{
    /* Get the challenge value. */
    g_challenge_value = Cy_CRWDT_GetChallenge();

    /* Calculate the response value. */
    g_response_value = Cy_CRWDT_CalculateResponse(g_challenge_value);

    /* Set the response value to clear CR watchdog counter.
     * If the values mismatch, the action selected by
     * CRWDT_CONFIG.CHALLENGE_FAIL_ACTION occurs.               */
    Cy_CRWDT_SetResponse(g_response_value);
}

/* [] END OF FILE */
