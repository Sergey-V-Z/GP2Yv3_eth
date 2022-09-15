/*
 * hcsr04_driver.cpp
 *
 *  Created on: 10 сент. 2022 г.
 *      Author: Ierixon-HP
 */

#include "hcsr04_driver.h"
#include "utils.h"

void HCSR04Driver::init(TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, float soundSpeed)
{
    // default driver state
    echoPulseStart = 0;
    echoPulseEnd = 0;
    echoPulseState = NOT_STARTED;
    echoDelay = -1;

    // save driver parameters
    this->soundSpeed = soundSpeed;

    // save timer parameters
    hTim.Instance = tim;
    this->triggerChannel = triggerChannel;
    this->echoChannel = echoChannel;

    // configure and initizialize timer
    hTim.State = HAL_TIM_STATE_RESET;

    // calculate timer prescaler to make timer frequncy equals COUNTER_FREQUNCY
    hTim.Init.Prescaler = (HAL_RCC_GetPCLK1Freq()*2) / COUNTER_FREQUNCY - 1;

    // calculate counter period
    hTim.Init.Period = COUNTER_FREQUNCY / PROBING_FREQUNCY - 1;
    hTim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hTim.Init.RepetitionCounter = 0;
    hTim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // configure timer
    enableTimerClock(hTim.Instance);
    HAL_TIM_Base_Init(&hTim);

    // configure and initizialize trigger channel
    TIM_OC_InitTypeDef sConfigTrigger;
    sConfigTrigger.OCMode = TIM_OCMODE_PWM1;
    sConfigTrigger.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigTrigger.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigTrigger.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigTrigger.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigTrigger.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigTrigger.Pulse = (TRIGGER_PULSE_LEN * COUNTER_FREQUNCY) / 1000000;

    // configure trigger channel
    HAL_TIM_PWM_ConfigChannel(&hTim, &sConfigTrigger, triggerChannel);

    // configure capture channel
    TIM_IC_InitTypeDef sConfigCapture;

    // detect start and end of the echo pulse
    sConfigCapture.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfigCapture.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigCapture.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigCapture.ICFilter = 0x00; // magic value between 0x00 and 0x0F
    HAL_TIM_IC_ConfigChannel(&hTim, &sConfigCapture, echoChannel);

    // configure timer interruptions in the NVIC

    IRQn_Type triggerIRQ = getIRQCode(hTim.Instance, TIM_IT_UPDATE);
    IRQn_Type captureIRQ = getIRQCode(hTim.Instance, getTimInterruptTypeCode(echoChannel));
    HAL_NVIC_SetPriority(triggerIRQ, INTERRUPT_PRIOIRY, INTERRUPT_SUBPRIOIRY);
    HAL_NVIC_EnableIRQ(triggerIRQ);
    HAL_NVIC_SetPriority(captureIRQ, INTERRUPT_PRIOIRY, INTERRUPT_SUBPRIOIRY);
    HAL_NVIC_EnableIRQ(captureIRQ);


    // start timer
    // enable update interrupt at the timer side
    __HAL_TIM_ENABLE_IT(&hTim, TIM_IT_UPDATE);

    // enable echo capture/compare interrupt at the timer side
    __HAL_TIM_ENABLE_IT(&hTim, getTimInterruptTypeCode(echoChannel));

    // enable trigger channel
    TIM_CCxChannelCmd(hTim.Instance, triggerChannel, TIM_CCx_ENABLE);
    if (IS_TIM_BREAK_INSTANCE(hTim.Instance) != RESET) {
        __HAL_TIM_MOE_ENABLE(&hTim);
    }

    // enable echo channel
    TIM_CCxChannelCmd(hTim.Instance, echoChannel, TIM_CCx_ENABLE);

    // start timer
    __HAL_TIM_ENABLE(&hTim);
}

void HCSR04Driver::_acknowledgeTimerUpdate()
{
    if (echoPulseState == ENDED) {
        echoDelay = echoPulseEnd - echoPulseStart;
    } else {
        // there no echo sigma or echo signal isn't ended
        echoDelay = -1;
    }
    echoPulseState = NOT_STARTED;
}

void HCSR04Driver::_acknowledgeChannelCapture()
{
    switch (echoPulseState) {
        case NOT_STARTED:
            echoPulseStart = HAL_TIM_ReadCapturedValue(&hTim, echoChannel);
            echoPulseState = STARTED;
            break;
        case STARTED:
             echoPulseEnd = HAL_TIM_ReadCapturedValue(&hTim, echoChannel);
            echoPulseState = ENDED;
            break;
        default:
            // do nothing
            break;
    }
}

float HCSR04Driver::getDistanceInSeconds()
{
    if (echoDelay < 0) {
        return -1.0f;
    } else {
        return (float)echoDelay / (COUNTER_FREQUNCY * 2.0f);
    }
}

float HCSR04Driver::getDistance()
{
    float delay = getDistanceInSeconds();
    if (delay < 0) {
        return delay;
    } else {
        return delay * soundSpeed;
    }
}


