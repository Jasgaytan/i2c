/*
 * gpio.c
 *
 *  Created on: 23 jul. 2020
 *      Author: Matías Lopez - Jesús López
 */
//*****************************************************************************
//
// gpio.c - Driver para manejar los pines de uC.
//
//*****************************************************************************

#include "gpio.h"
//*****************************************************************************
uint8_t flag;
volatile unsigned int holdCount = 0;
//*****************************************************************************
// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};
//*****************************************************************************
void GPIO_configPins(uint16_t* selectedPort, uint16_t* selectedPin)
{
    uint16_t port = *(selectedPort);
    uint8_t  pin  = *(selectedPin);

    switch (port)
        {
            case 1:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPort) = 0x0200;
                break;
            case 2:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPin) <<= 8;
                *(selectedPort) =  0x0200;
                break;
            case 3:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPort) = 0x0220;
                break;
            case 4:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPin) <<= 8;
                *(selectedPort) = 0x0220;
                break;
            case 5:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPort) = 0x0240;
                break;
            case 6:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPin) <<= 8;
                *(selectedPort) = 0x0240;
                break;
            case 7:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPort) = 0x0260;
                break;
            case 8:
                *(selectedPin) = (0x0001 << pin);
                *(selectedPin) <<= 8;
                *(selectedPort) = 0x0260;
                break;
        }
}
//*****************************************************************************
void GPIO_powerOnSensor(const uint8_t vccPort, const uint8_t vccPin)
{
    uint16_t selectedPortVcc = vccPort;
    uint16_t selectedPinVcc  = vccPin;

    GPIO_configPins(&selectedPortVcc, &selectedPinVcc);
    GPIO_setHighOnPin(selectedPortVcc, selectedPinVcc);
}
//*****************************************************************************
void GPIO_powerOffSensor(const uint8_t vccPort, const uint8_t vccPin)
{
    uint16_t selectedPortVcc = vccPort;
    uint16_t selectedPinVcc  = vccPin;

    GPIO_configPins(&selectedPortVcc, &selectedPinVcc);
    GPIO_setLowOnPin(selectedPortVcc, selectedPinVcc);
}
//*****************************************************************************
/*
 * PORT1 Interrupt Service Routine
 * Handles S1 button press interrupt
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 : break;
        case P1IV_P1IFG2 :
            flag = 1;
            // Start debounce timer
            Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            break;
        case P1IV_P1IFG3 : break;
        case P1IV_P1IFG4 : break;
        case P1IV_P1IFG5 : break;
        case P1IV_P1IFG6 : break;
        case P1IV_P1IFG7 : break;
    }
}
//*****************************************************************************
/*
 * PORT2 Interrupt Service Routine
 * Handles S2 button press interrupt
 */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    switch(__even_in_range(P2IV, P2IV_P2IFG7))
    {
        case P2IV_NONE : break;
        case P2IV_P2IFG0 : break;
        case P2IV_P2IFG1 : break;
        case P2IV_P2IFG2 : break;
        case P2IV_P2IFG3 : break;
        case P2IV_P2IFG4 : break;
        case P2IV_P2IFG5 : break;
        case P2IV_P2IFG6 :
            flag = 2;
            // Start debounce timer
            Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            break;
        case P2IV_P2IFG7 : break;
    }
}
//*****************************************************************************
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // Both button S1 & S2 held down
    if (!(P1IN & BIT2) && !(P2IN & BIT6))
    {
        holdCount++;
        if (holdCount == 40)
        {
            flag = 3;
            Timer_A_stop(TIMER_A0_BASE);
            holdCount = 0;
            TA0CTL &= ~TAIFG;
            //__bic_SR_register_on_exit(LPM3_bits);                // exit LPM3
        }
    }

    // Both button S1 & S2 released
    if ((P1IN & BIT2) && (P2IN & BIT6))
    {
        // Stop timer A0
        Timer_A_stop(TIMER_A0_BASE);
        TA0CTL &= ~TAIFG;
    }
}
