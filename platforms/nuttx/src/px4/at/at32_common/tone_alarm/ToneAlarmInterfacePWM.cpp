/****************************************************************************
 *
 *   Copyright (C) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>
#include <px4_platform_common/defines.h>
#include <math.h>

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_TIMER) && defined(HRT_TIMER)
# if TONE_ALARM_TIMER == HRT_TIMER
#   error TONE_ALARM_TIMER and HRT_TIMER must use different timers.
# endif
#endif

/* Period of the free-running counter, in microseconds. */
#ifndef TONE_ALARM_COUNTER_PERIOD
#define TONE_ALARM_COUNTER_PERIOD 65536
#endif

/* The H7 has a 2 RCC_APB1ENR registers RCC_APB1LENR and RCC_APB1HENR
 * We simply map the RCC_APB1LENR back to AT32_CRM_APB1EN as well as
 * the bits
 */

#if !defined(AT32_CRM_APB1EN) && defined(AT32_CRM_APB1LPEN)
# define AT32_CRM_APB1EN AT32_CRM_APB1LPEN

# define RCC_APB1ENR_TIM2EN  CRM_APB1LPEN_TMR2LPEN
# define RCC_APB1ENR_TIM3EN  CRM_APB1LPEN_TMR3LPEN
# define RCC_APB1ENR_TIM4EN  CRM_APB1LPEN_TMR4LPEN
# define RCC_APB1ENR_TIM5EN  CRM_APB1LPEN_TMR5LPEN
# define RCC_APB1ENR_TIM6EN  CRM_APB1LPEN_TMR6LPEN
# define RCC_APB1ENR_TIM7EN  CRM_APB1LPEN_TMR7LPEN
# define RCC_APB1ENR_TIM12EN CRM_APB1LPEN_TMR12LPEN
# define RCC_APB1ENR_TIM13EN CRM_APB1LPEN_TMR13LPEN
# define RCC_APB1ENR_TIM14EN CRM_APB1LPEN_TMR14LPEN
#endif
/* Tone alarm configuration */
#if   TONE_ALARM_TIMER == 1
# define TONE_ALARM_BASE                AT32_TMR1_BASE
# define TONE_ALARM_CLOCK               AT32_APB2_TIM1_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB2EN_TMR1EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB2EN
# if defined(CONFIG_AT32_TIM1)
#  error Must not set CONFIG_AT32_TIM1 when TONE_ALARM_TIMER is 1
# endif
#elif TONE_ALARM_TIMER == 2
# define TONE_ALARM_BASE                AT32_TMR2_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM2_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR2EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM2)
#  error Must not set CONFIG_AT32_TIM2 when TONE_ALARM_TIMER is 2
# endif
#elif TONE_ALARM_TIMER == 3
# define TONE_ALARM_BASE                AT32_TMR3_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM3_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR3EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM3)
#  error Must not set CONFIG_AT32_TIM3 when TONE_ALARM_TIMER is 3
# endif
#elif TONE_ALARM_TIMER == 4
# define TONE_ALARM_BASE                AT32_TMR4_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM4_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR4EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM4)
#  error Must not set CONFIG_AT32_TIM4 when TONE_ALARM_TIMER is 4
# endif
#elif TONE_ALARM_TIMER == 5
# define TONE_ALARM_BASE                AT32_TMR5_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM5_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR5EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM5)
#  error Must not set CONFIG_AT32_TIM5 when TONE_ALARM_TIMER is 5
# endif
#elif TONE_ALARM_TIMER == 8
# define TONE_ALARM_BASE                AT32_TMR8_BASE
# define TONE_ALARM_CLOCK               AT32_APB2_TIM8_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB2EN_TMR8EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB2EN
# if defined(CONFIG_AT32_TIM8)
#  error Must not set CONFIG_AT32_TIM8 when TONE_ALARM_TIMER is 8
# endif
#elif TONE_ALARM_TIMER == 9
# define TONE_ALARM_BASE                AT32_TMR9_BASE
# define TONE_ALARM_CLOCK               AT32_APB2_TIM9_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB2EN_TMR9EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB2EN
# if defined(CONFIG_AT32_TIM9)
#  error Must not set CONFIG_AT32_TIM9 when TONE_ALARM_TIMER is 9
# endif
#elif TONE_ALARM_TIMER == 10
# define TONE_ALARM_BASE                AT32_TMR10_BASE
# define TONE_ALARM_CLOCK               AT32_APB2_TIM10_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB2EN_TMR10EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB2EN
# if defined(CONFIG_AT32_TIM10)
#  error Must not set CONFIG_AT32_TIM10 when TONE_ALARM_TIMER is 10
# endif
#elif TONE_ALARM_TIMER == 11
# define TONE_ALARM_BASE                AT32_TMR11_BASE
# define TONE_ALARM_CLOCK               AT32_APB2_TIM11_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB2EN_TMR11EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB2EN
# if defined(CONFIG_AT32_TIM11)
#  error Must not set CONFIG_AT32_TIM11 when TONE_ALARM_TIMER is 11
# endif
#elif TONE_ALARM_TIMER == 12
# define TONE_ALARM_BASE                AT32_TMR12_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM12_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR12EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM12)
#  error Must not set CONFIG_AT32_TIM12 when TONE_ALARM_TIMER is 12
# endif
#elif TONE_ALARM_TIMER == 13
# define TONE_ALARM_BASE                AT32_TMR13_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM13_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR13EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM13)
#  error Must not set CONFIG_AT32_TIM13 when TONE_ALARM_TIMER is 13
# endif
#elif TONE_ALARM_TIMER == 14
# define TONE_ALARM_BASE                AT32_TMR14_BASE
# define TONE_ALARM_CLOCK               AT32_APB1_TIM14_CLKIN
# define TONE_ALARM_CLOCK_ENABLE        CRM_APB1EN_TMR14EN
# define TONE_ALARM_CLOCK_POWER_REG     AT32_CRM_APB1EN
# if defined(CONFIG_AT32_TIM14)
#  error Must not set CONFIG_AT32_TIM14 when TONE_ALARM_TIMER is 14
# endif
#else
# error Must set TONE_ALARM_TIMER to one of the timers between 1 and 17 (inclusive) to use this driver.
#endif // TONE_ALARM_TIMER

#if TONE_ALARM_CHANNEL == 1
# define TONE_CCER      (1 << 0)
# define TONE_CCMR1     (3 << 4)
# define TONE_CCMR2     0
# define TONE_rCCR      rCCR1
#elif TONE_ALARM_CHANNEL == 2
# define TONE_CCER      (1 << 4)
# define TONE_CCMR1     (3 << 12)
# define TONE_CCMR2     0
# define TONE_rCCR      rCCR2
#elif TONE_ALARM_CHANNEL == 3
# define TONE_CCER      (1 << 8)
# define TONE_CCMR1     0
# define TONE_CCMR2     (3 << 4)
# define TONE_rCCR      rCCR3
#elif TONE_ALARM_CHANNEL == 4
# define TONE_CCER      (1 << 12)
# define TONE_CCMR1     0
# define TONE_CCMR2     (3 << 12)
# define TONE_rCCR      rCCR4
#else
# error Must set TONE_ALARM_CHANNEL to a value between 1 and 4 to use this driver.
#endif // TONE_ALARM_CHANNEL

/* Timer register accessors. */
#define REG(_reg)       (*(volatile uint32_t *)(TONE_ALARM_BASE + _reg))

#if TONE_ALARM_TIMER == 1 || TONE_ALARM_TIMER == 8 // Note: If using TIM1 or TIM8, then you are using the ADVANCED timers and NOT the GENERAL TIMERS, therefore different registers
# define rARR           REG(AT32_ATIM_ARR_OFFSET)
# define rBDTR          REG(AT32_ATIM_BDTR_OFFSET)
# define rCCER          REG(AT32_ATIM_CCER_OFFSET)
# define rCCMR1         REG(AT32_ATIM_CCMR1_OFFSET)
# define rCCMR2         REG(AT32_ATIM_CCMR2_OFFSET)
# define rCCR1          REG(AT32_ATIM_CCR1_OFFSET)
# define rCCR2          REG(AT32_ATIM_CCR2_OFFSET)
# define rCCR3          REG(AT32_ATIM_CCR3_OFFSET)
# define rCCR4          REG(AT32_ATIM_CCR4_OFFSET)
# define rCNT           REG(AT32_ATIM_CNT_OFFSET)
# define rCR1           REG(AT32_ATIM_CR1_OFFSET)
# define rCR2           REG(AT32_ATIM_CR2_OFFSET)
# define rDCR           REG(AT32_ATIM_DCR_OFFSET)
# define rDIER          REG(AT32_ATIM_DIER_OFFSET)
# define rDMAR          REG(AT32_ATIM_DMAR_OFFSET)
# define rEGR           REG(AT32_ATIM_EGR_OFFSET)
# define rPSC           REG(AT32_ATIM_PSC_OFFSET)
# define rRCR           REG(AT32_ATIM_RCR_OFFSET)
# define rSMCR          REG(AT32_ATIM_SMCR_OFFSET)
# define rSR            REG(AT32_ATIM_SR_OFFSET)
#else
# define rARR           REG(AT32_GTIM_ARR_OFFSET)
# define rCCER          REG(AT32_GTIM_CCER_OFFSET)
# define rCCMR1         REG(AT32_GTIM_CCMR1_OFFSET)
# define rCCMR2         REG(AT32_GTIM_CCMR2_OFFSET)
# define rCCR1          REG(AT32_GTIM_CCR1_OFFSET)
# define rCCR2          REG(AT32_GTIM_CCR2_OFFSET)
# define rCCR3          REG(AT32_GTIM_CCR3_OFFSET)
# define rCCR4          REG(AT32_GTIM_CCR4_OFFSET)
# define rCNT           REG(AT32_GTIM_CNT_OFFSET)
# define rCR1           REG(AT32_GTIM_CR1_OFFSET)
# define rCR2           REG(AT32_GTIM_CR2_OFFSET)
# define rDCR           REG(AT32_GTIM_DCR_OFFSET)
# define rDIER          REG(AT32_GTIM_DIER_OFFSET)
# define rDMAR          REG(AT32_GTIM_DMAR_OFFSET)
# define rEGR           REG(AT32_GTIM_EGR_OFFSET)
# define rPSC           REG(AT32_GTIM_PSC_OFFSET)
# define rRCR           REG(AT32_GTIM_RCR_OFFSET)
# define rSMCR          REG(AT32_GTIM_SMCR_OFFSET)
# define rSR            REG(AT32_GTIM_SR_OFFSET)
#endif

namespace ToneAlarmInterface
{

void init()
{
#ifdef GPIO_TONE_ALARM_NEG
	px4_arch_configgpio(GPIO_TONE_ALARM_NEG);
#else
	// Configure the GPIO to the idle state.
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
#endif

	// Clock/power on our timer.
	modifyreg32(TONE_ALARM_CLOCK_POWER_REG, 0, TONE_ALARM_CLOCK_ENABLE);

	// Initialize the timer.
	rCCER &= TONE_CCER;	// Unlock CCMR* registers.
	rCCER  = TONE_CCER;
	rCCMR1 = TONE_CCMR1;
	rCCMR2 = TONE_CCMR2;
	rCR1   = 0;
	rCR2   = 0;
	rDCR   = 0;
	rDIER  = 0;
	rSMCR  = 0;

#ifdef rBDTR // If using an advanced timer, you need to activate the output.
	rBDTR = ATIM_BDTR_MOE; // Enable the main output of the advanced timer.
#endif

	TONE_rCCR = 1;		// Toggle the CC output each time the count passes 1.
	rPSC = 0;		// Default the timer to a prescale value of 1; playing notes will change this.
	rCR1 = GTIM_CR1_CEN;	// Ensure the timer is running.
}

hrt_abstime start_note(unsigned frequency)
{
	// Calculate the signal switching period.
	// (Signal switching period is one half of the frequency period).
	float signal_period = (1.0f / frequency) * 0.5f;

	// Calculate the hardware clock divisor rounded to the nearest integer.
	unsigned int divisor = roundf(signal_period * TONE_ALARM_CLOCK);

	// Pick the lowest prescaler value that we can use.
	// (Note that the effective prescale value is 1 greater.)
	unsigned int prescale = divisor / TONE_ALARM_COUNTER_PERIOD;

	// Calculate the period for the selected prescaler value.
	unsigned int period = (divisor / (prescale + 1)) - 1;

	rPSC = prescale;	// Load new prescaler.
	rARR = period;		// Load new toggle period.
	rEGR = GTIM_EGR_UG;	// Force a reload of the period.
	rCCER |= TONE_CCER; 	// Enable the output.

	// Configure the GPIO to enable timer output.
	hrt_abstime time_started = hrt_absolute_time();
	irqstate_t flags = enter_critical_section();
	px4_arch_configgpio(GPIO_TONE_ALARM);
	leave_critical_section(flags);

	return time_started;
}

void stop_note()
{
	// Stop the current note.
	rCCER &= ~TONE_CCER;

	// Ensure the GPIO is not driving the speaker.
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}

} /* namespace ToneAlarmInterface */
