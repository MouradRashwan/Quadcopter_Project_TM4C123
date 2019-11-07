/*
 * QuadPWM.c
 *
 *  Created on: Mar 10, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "QuadPWM_driver.h"

static SignalAlign_t g_tSignalAlign;
static uint32_t g_ui32PWMClkInHz, g_ui32PWMPeriodInTicks, g_ui32PWMFreqInHz;

static void _PWMGenConfigure(uint32_t ui32Base, uint32_t ui32Gen,
                             uint32_t ui32Config, SignalAlign_t tSignalAlign)
{
    /* Compute the generator's base address */
    ui32Gen = ui32Base + ui32Gen;

    /* Change the global configuration of the generator */
    HWREG(ui32Gen + PWM_O_X_CTL) = ((HWREG(ui32Gen + PWM_O_X_CTL)
            & ~(PWM_X_CTL_MODE | PWM_X_CTL_DEBUG | PWM_X_CTL_LATCH
                    | PWM_X_CTL_MINFLTPER | PWM_X_CTL_FLTSRC
                    | PWM_X_CTL_DBFALLUPD_M | PWM_X_CTL_DBRISEUPD_M
                    | PWM_X_CTL_DBCTLUPD_M | PWM_X_CTL_GENBUPD_M
                    | PWM_X_CTL_GENAUPD_M | PWM_X_CTL_LOADUPD
                    | PWM_X_CTL_CMPAUPD | PWM_X_CTL_CMPBUPD)) | ui32Config);

    /* Set the individual PWM generator controls based on Signal Align */
    if (tSignalAlign == CENTER_ALIGNED)
    {
        //
        // In up/down count mode, set the signal high on up count comparison
        // and low on down count comparison (that is, center align the
        // signals).
        //
        HWREG(ui32Gen + PWM_O_X_GENA) = (PWM_X_GENA_ACTCMPAU_ONE
                | PWM_X_GENA_ACTCMPAD_ZERO);
        HWREG(ui32Gen + PWM_O_X_GENB) = (PWM_X_GENB_ACTCMPBU_ONE
                | PWM_X_GENB_ACTCMPBD_ZERO);
    }
    else if (tSignalAlign == LEFT_ALIGNED)
    {
        //
        // In down count mode, set the signal high on load and low on count
        // comparison (that is, left align the signals).
        //
        HWREG(ui32Gen + PWM_O_X_GENA) = (PWM_X_GENA_ACTLOAD_ONE
                | PWM_X_GENA_ACTCMPAD_ZERO);
        HWREG(ui32Gen + PWM_O_X_GENB) = (PWM_X_GENB_ACTLOAD_ONE
                | PWM_X_GENB_ACTCMPBD_ZERO);
    }
    else if (tSignalAlign == RIGHT_ALIGNED)
    {
        //
        // In down count mode, set the signal high on count comparison and low on ZERO
        // (that is, right align the signals).
        //
        HWREG(ui32Gen + PWM_O_X_GENA) = (PWM_X_GENA_ACTCMPAD_ONE
                | PWM_X_GENA_ACTZERO_ZERO);
        HWREG(ui32Gen + PWM_O_X_GENB) = (PWM_X_GENB_ACTCMPBD_ONE
                | PWM_X_GENB_ACTZERO_ZERO);
    }
    else
    {
        /* ERROR : [tSignalAlign] is not correct */
    }
}

PWMStatus_t QuadPWM_init(uint32_t ui32PWMFreqInHz, SignalAlign_t tSignalAlign)
{
    uint32_t ui32Div, ui32DivParam, ui32PWMPeriodInTicks, ui32PWMClkInHz,
            ui32CountMode, ui32PWMPulseWidth;

    SysCtlPeripheralEnable(PIN1_PORT_CLK_BASE);
    while (!SysCtlPeripheralReady(PIN1_PORT_CLK_BASE))
    {
    }
    GPIOPinConfigure(PIN1_PORT_PCTL_FUN);
    GPIOPinTypePWM(PIN1_PORT_ADDR_BASE, PIN1_PORT_BIT_NUM);

    SysCtlPeripheralEnable(PIN2_PORT_CLK_BASE);
    while (!SysCtlPeripheralReady(PIN2_PORT_CLK_BASE))
    {
    }
    GPIOPinConfigure(PIN2_PORT_PCTL_FUN);
    GPIOPinTypePWM(PIN2_PORT_ADDR_BASE, PIN2_PORT_BIT_NUM);

    SysCtlPeripheralEnable(PIN3_PORT_CLK_BASE);
    while (!SysCtlPeripheralReady(PIN3_PORT_CLK_BASE))
    {
    }
    GPIOPinConfigure(PIN3_PORT_PCTL_FUN);
    GPIOPinTypePWM(PIN3_PORT_ADDR_BASE, PIN3_PORT_BIT_NUM);

    SysCtlPeripheralEnable(PIN4_PORT_CLK_BASE);
    while (!SysCtlPeripheralReady(PIN4_PORT_CLK_BASE))
    {
    }
    GPIOPinConfigure(PIN4_PORT_PCTL_FUN);
    GPIOPinTypePWM(PIN4_PORT_ADDR_BASE, PIN4_PORT_BIT_NUM);

    SysCtlPeripheralEnable(PWM_CLK_BASE);
    while (!SysCtlPeripheralReady(PWM_CLK_BASE))
    {
    }

    /* Trying all PWM clock divisions to set its period/frequency correctly */
    for (ui32Div = 1U; ui32Div < 128U; ui32Div *= 2U)
    {
        if (ui32Div == 1U)
        {
            ui32DivParam = SYSCTL_PWMDIV_1;
        }
        else if (ui32Div == 2U)
        {
            ui32DivParam = SYSCTL_PWMDIV_2;
        }
        else
        {
            ui32DivParam += 0x00020000U;
        }

        ui32PWMClkInHz = SysCtlClockGet() / ui32Div;
        ui32PWMPeriodInTicks = SysCtlClockGet() / ui32Div / ui32PWMFreqInHz;

        if (tSignalAlign == CENTER_ALIGNED)
        {
            ui32PWMPulseWidth = 0;
            if ((ui32PWMPeriodInTicks / 2U) <= 0x0000FFFFU) /* As PWM has a 16-bit counter */
            {
                ui32CountMode = PWM_GEN_MODE_UP_DOWN;
                break;
            }
        }
        else if (tSignalAlign == LEFT_ALIGNED)
        {
            ui32PWMPulseWidth = 1U;
            if ((ui32PWMPeriodInTicks - 1U) <= 0x0000FFFFU) /* As PWM has a 16-bit counter */
            {
                ui32CountMode = PWM_GEN_MODE_DOWN;
                break;
            }
        }
        else if (tSignalAlign == RIGHT_ALIGNED)
        {
            ui32PWMPulseWidth = 0;
            if ((ui32PWMPeriodInTicks - 1U) <= 0x0000FFFFU) /* As PWM has a 16-bit counter */
            {
                ui32CountMode = PWM_GEN_MODE_DOWN;
                break;
            }
        }
        else
        {
            /* ERROR : [tSignalAlign] is not correct  */
            g_ui32PWMClkInHz = 0;
            g_ui32PWMFreqInHz = 0;
            g_ui32PWMPeriodInTicks = 0;
            g_tSignalAlign = UKNOWN_ALIGNED;
            return PWM_ERROR;
        }
    }

    /* Error to set the PWM period, as it is larger than [0xFFFF] */
    if (ui32Div == 128U)
    {
        g_ui32PWMClkInHz = 0;
        g_ui32PWMFreqInHz = 0;
        g_ui32PWMPeriodInTicks = 0;
        g_tSignalAlign = UKNOWN_ALIGNED;
        return PWM_ERROR;
    }

    /* Set the PWM clock source division */
    SysCtlPWMClockSet(ui32DivParam);

    /* Set update mode for PWMn OUT enable/disable to NO_SYNC */
    PWMOutputUpdateMode(
            PWM_ADDR_BASE,
            PIN1_PWM_OUT_BIT | PIN2_PWM_OUT_BIT | PIN3_PWM_OUT_BIT
                    | PIN4_PWM_OUT_BIT,
            PWM_OUTPUT_MODE_SYNC_GLOBAL);

    /* Configure all PWMn generators */
    _PWMGenConfigure(
            PWM_ADDR_BASE,
            PIN1_PWM_GEN,
            ui32CountMode | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN
                    | PWM_GEN_MODE_GEN_NO_SYNC,
            tSignalAlign);
    _PWMGenConfigure(
            PWM_ADDR_BASE,
            PIN2_PWM_GEN,
            ui32CountMode | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN
                    | PWM_GEN_MODE_GEN_NO_SYNC,
            tSignalAlign);
    _PWMGenConfigure(
            PWM_ADDR_BASE,
            PIN3_PWM_GEN,
            ui32CountMode | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN
                    | PWM_GEN_MODE_GEN_NO_SYNC,
            tSignalAlign);
    _PWMGenConfigure(
            PWM_ADDR_BASE,
            PIN4_PWM_GEN,
            ui32CountMode | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN
                    | PWM_GEN_MODE_GEN_NO_SYNC,
            tSignalAlign);

    /* Set the PWMn frequency by setting the its period in ticks */
    PWMGenPeriodSet(PWM_ADDR_BASE, PIN1_PWM_GEN, ui32PWMPeriodInTicks); /* PWMn GEN0 */
    PWMGenPeriodSet(PWM_ADDR_BASE, PIN2_PWM_GEN, ui32PWMPeriodInTicks); /* PWMn GEN1 */
    PWMGenPeriodSet(PWM_ADDR_BASE, PIN3_PWM_GEN, ui32PWMPeriodInTicks); /* PWMn GEN2 */
    PWMGenPeriodSet(PWM_ADDR_BASE, PIN4_PWM_GEN, ui32PWMPeriodInTicks); /* PWMn GEN3 */

    /* Set all pulses width to 0 */
    PWMPulseWidthSet(PWM_ADDR_BASE, PIN1_PWM_OUT, ui32PWMPulseWidth); /* PIN1 PWMn OUTn */
    PWMPulseWidthSet(PWM_ADDR_BASE, PIN2_PWM_OUT, ui32PWMPulseWidth); /* PIN2 PWMn OUTn */
    PWMPulseWidthSet(PWM_ADDR_BASE, PIN3_PWM_OUT, ui32PWMPulseWidth); /* PIN3 PWMn OUTn */
    PWMPulseWidthSet(PWM_ADDR_BASE, PIN4_PWM_OUT, ui32PWMPulseWidth); /* PIN4 PWMn OUTn */

    /* Enable the PWMn outputs */
    PWMOutputState(PWM_ADDR_BASE, PIN1_PWM_OUT_BIT, false); /* PIN1 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN2_PWM_OUT_BIT, false); /* PIN2 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN3_PWM_OUT_BIT, false); /* PIN3 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN4_PWM_OUT_BIT, false); /* PIN4 PWMn OUTn bit */

    /* Enable the PWMn generators */
    PWMGenEnable(PWM_ADDR_BASE, PIN1_PWM_GEN); /* PWMn GEN0 */
    PWMGenEnable(PWM_ADDR_BASE, PIN2_PWM_GEN); /* PWMn GEN1 */
    PWMGenEnable(PWM_ADDR_BASE, PIN3_PWM_GEN); /* PWMn GEN2 */
    PWMGenEnable(PWM_ADDR_BASE, PIN4_PWM_GEN); /* PWMn GEN3 */

    /* Commit all global updates specially Period & PulseWidth */
    PWMSyncUpdate(
            PWM_ADDR_BASE,
            PIN1_PWM_GEN_BIT | PIN2_PWM_GEN_BIT | PIN3_PWM_GEN_BIT
                    | PIN4_PWM_GEN_BIT);

    /* Reset the generators counters at once to synchronize all of them */
    PWMSyncTimeBase(
            PWM_ADDR_BASE,
            PIN1_PWM_GEN_BIT | PIN2_PWM_GEN_BIT | PIN3_PWM_GEN_BIT
                    | PIN4_PWM_GEN_BIT);

    g_ui32PWMClkInHz = ui32PWMClkInHz;
    g_ui32PWMFreqInHz = ui32PWMFreqInHz;
    g_ui32PWMPeriodInTicks = ui32PWMPeriodInTicks;
    g_tSignalAlign = tSignalAlign;

    return PWM_SUCCESS;
}

void QuadPWM_setPulses(PulseWidth_t *ptPulseWidth)
{
    if (g_tSignalAlign == CENTER_ALIGNED)
    {
        if (ptPulseWidth->ui32PulseInTicks1 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks1 = g_ui32PWMPeriodInTicks - 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks2 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks2 = g_ui32PWMPeriodInTicks - 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks3 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks3 = g_ui32PWMPeriodInTicks - 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks4 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks4 = g_ui32PWMPeriodInTicks - 1U;
        }
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN1_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks1); /* PIN1 PWMn OUTn */
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN2_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks2); /* PIN2 PWMn OUTn */
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN3_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks3); /* PIN3 PWMn OUTn */
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN4_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks4); /* PIN4 PWMn OUTn */
    }
    else if (g_tSignalAlign == LEFT_ALIGNED)
    {
        if (ptPulseWidth->ui32PulseInTicks1 == 0)
        {
            ptPulseWidth->ui32PulseInTicks1 = 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks2 == 0)
        {
            ptPulseWidth->ui32PulseInTicks2 = 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks3 == 0)
        {
            ptPulseWidth->ui32PulseInTicks3 = 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks4 == 0)
        {
            ptPulseWidth->ui32PulseInTicks4 = 1U;
        }
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN1_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks1); /* PIN1 PWMn OUTn */
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN2_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks2); /* PIN2 PWMn OUTn */
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN3_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks3); /* PIN3 PWMn OUTn */
        PWMPulseWidthSet(PWM_ADDR_BASE, PIN4_PWM_OUT,
                         ptPulseWidth->ui32PulseInTicks4); /* PIN4 PWMn OUTn */
    }
    else if (g_tSignalAlign == RIGHT_ALIGNED)
    {
        if (ptPulseWidth->ui32PulseInTicks1 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks1 = g_ui32PWMPeriodInTicks - 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks2 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks2 = g_ui32PWMPeriodInTicks - 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks3 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks3 = g_ui32PWMPeriodInTicks - 1U;
        }
        if (ptPulseWidth->ui32PulseInTicks4 >= g_ui32PWMPeriodInTicks)
        {
            ptPulseWidth->ui32PulseInTicks4 = g_ui32PWMPeriodInTicks - 1U;
        }
        PWMPulseWidthSet(
                PWM_ADDR_BASE, PIN1_PWM_OUT,
                g_ui32PWMPeriodInTicks - ptPulseWidth->ui32PulseInTicks1); /* PIN1 PWMn OUTn */
        PWMPulseWidthSet(
                PWM_ADDR_BASE, PIN2_PWM_OUT,
                g_ui32PWMPeriodInTicks - ptPulseWidth->ui32PulseInTicks2); /* PIN2 PWMn OUTn */
        PWMPulseWidthSet(
                PWM_ADDR_BASE, PIN3_PWM_OUT,
                g_ui32PWMPeriodInTicks - ptPulseWidth->ui32PulseInTicks3); /* PIN3 PWMn OUTn */
        PWMPulseWidthSet(
                PWM_ADDR_BASE, PIN4_PWM_OUT,
                g_ui32PWMPeriodInTicks - ptPulseWidth->ui32PulseInTicks4); /* PIN4 PWMn OUTn */
    }
    else
    {
        /* ERROR : [g_tSignalAlign] is not correct */
    }
}

void QuadPWM_getPulses(PulseWidth_t *ptPulseWidth)
{
    if ((g_tSignalAlign == CENTER_ALIGNED) || (g_tSignalAlign == LEFT_ALIGNED))
    {
        ptPulseWidth->ui32PulseInTicks1 = PWMPulseWidthGet(PWM_ADDR_BASE,
        PIN1_PWM_OUT); /* PIN1 PWMn OUTn */
        ptPulseWidth->ui32PulseInTicks2 = PWMPulseWidthGet(PWM_ADDR_BASE,
        PIN2_PWM_OUT); /* PIN2 PWMn OUTn */
        ptPulseWidth->ui32PulseInTicks3 = PWMPulseWidthGet(PWM_ADDR_BASE,
        PIN3_PWM_OUT); /* PIN3 PWMn OUTn */
        ptPulseWidth->ui32PulseInTicks4 = PWMPulseWidthGet(PWM_ADDR_BASE,
        PIN4_PWM_OUT); /* PIN4 PWMn OUTn */
    }
    else if (g_tSignalAlign == RIGHT_ALIGNED)
    {
        ptPulseWidth->ui32PulseInTicks1 = g_ui32PWMPeriodInTicks
                - PWMPulseWidthGet(PWM_ADDR_BASE, PIN1_PWM_OUT); /* PIN1 PWMn OUTn */
        ptPulseWidth->ui32PulseInTicks2 = g_ui32PWMPeriodInTicks
                - PWMPulseWidthGet(PWM_ADDR_BASE, PIN2_PWM_OUT); /* PIN2 PWMn OUTn */
        ptPulseWidth->ui32PulseInTicks3 = g_ui32PWMPeriodInTicks
                - PWMPulseWidthGet(PWM_ADDR_BASE, PIN3_PWM_OUT); /* PIN3 PWMn OUTn */
        ptPulseWidth->ui32PulseInTicks4 = g_ui32PWMPeriodInTicks
                - PWMPulseWidthGet(PWM_ADDR_BASE, PIN4_PWM_OUT); /* PIN4 PWMn OUTn */
    }
    else
    {
        /* ERROR : [g_tSignalAlign] is not correct */
    }
}

void QuadPWM_enableOutputs(void)
{
    /* Enable the PWM outputs */
    PWMOutputState(PWM_ADDR_BASE, PIN1_PWM_OUT_BIT, true); /* PIN1 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN2_PWM_OUT_BIT, true); /* PIN2 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN3_PWM_OUT_BIT, true); /* PIN3 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN4_PWM_OUT_BIT, true); /* PIN4 PWMn OUTn bit */
}

void QuadPWM_disableOutputs(void)
{
    /* Disable the PWM outputs */
    PWMOutputState(PWM_ADDR_BASE, PIN1_PWM_OUT_BIT, false); /* PIN1 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN2_PWM_OUT_BIT, false); /* PIN2 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN3_PWM_OUT_BIT, false); /* PIN3 PWMn OUTn bit */
    PWMOutputState(PWM_ADDR_BASE, PIN4_PWM_OUT_BIT, false); /* PIN4 PWMn OUTn bit */
}

void QuadPWM_update(void)
{
    /* Commit all global updates */
    PWMSyncUpdate(
            PWM_ADDR_BASE,
            PIN1_PWM_GEN_BIT | PIN2_PWM_GEN_BIT | PIN3_PWM_GEN_BIT
                    | PIN4_PWM_GEN_BIT);
}

void QuadPWM_updateReset(void)
{
    /* Commit all global updates specially Period & PulseWidth */
    PWMSyncUpdate(
            PWM_ADDR_BASE,
            PIN1_PWM_GEN_BIT | PIN2_PWM_GEN_BIT | PIN3_PWM_GEN_BIT
                    | PIN4_PWM_GEN_BIT);

    /* Reset the generators counters at once to synchronize all of them */
    PWMSyncTimeBase(
            PWM_ADDR_BASE,
            PIN1_PWM_GEN_BIT | PIN2_PWM_GEN_BIT | PIN3_PWM_GEN_BIT
                    | PIN4_PWM_GEN_BIT);
}

UpdateStatus_t QuadPWM_getUpdateStatus(void)
{
    return (UpdateStatus_t) (HWREG(PWM_ADDR_BASE + PWM_O_CTL)
            & (PIN1_PWM_GEN_BIT | PIN2_PWM_GEN_BIT | PIN3_PWM_GEN_BIT
                    | PIN4_PWM_GEN_BIT));
}

uint32_t QuadPWM_getPeriod(void)
{
    return g_ui32PWMPeriodInTicks;
}

uint32_t QuadPWM_getFreq(void)
{
    return g_ui32PWMFreqInHz;
}

uint32_t QuadPWM_getClk(void)
{
    return g_ui32PWMClkInHz;
}

