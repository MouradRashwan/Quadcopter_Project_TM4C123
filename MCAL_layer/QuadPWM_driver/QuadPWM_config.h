/*
 * QuadPWM_config.h
 *
 *  Created on: Mar 12, 2019
 *      Author: Administrator
 */

#ifndef QUADPWM_CONFIG_H_
#define QUADPWM_CONFIG_H_

#define PWM_ADDR_BASE           PWM0_BASE
#define PWM_CLK_BASE            SYSCTL_PERIPH_PWM0

#define PIN1_PWM_GEN            PWM_GEN_0
#define PIN1_PWM_GEN_BIT        PWM_GEN_0_BIT
#define PIN1_PWM_OUT            PWM_OUT_0
#define PIN1_PWM_OUT_BIT        PWM_OUT_0_BIT
#define PIN1_PORT_BIT_NUM       GPIO_PIN_6
#define PIN1_PORT_PCTL_FUN      GPIO_PB6_M0PWM0
#define PIN1_PORT_ADDR_BASE     GPIO_PORTB_BASE
#define PIN1_PORT_CLK_BASE      SYSCTL_PERIPH_GPIOB

#define PIN2_PWM_GEN            PWM_GEN_1
#define PIN2_PWM_GEN_BIT        PWM_GEN_1_BIT
#define PIN2_PWM_OUT            PWM_OUT_2
#define PIN2_PWM_OUT_BIT        PWM_OUT_2_BIT
#define PIN2_PORT_BIT_NUM       GPIO_PIN_4
#define PIN2_PORT_PCTL_FUN      GPIO_PB4_M0PWM2
#define PIN2_PORT_ADDR_BASE     GPIO_PORTB_BASE
#define PIN2_PORT_CLK_BASE      SYSCTL_PERIPH_GPIOB

#define PIN3_PWM_GEN            PWM_GEN_2
#define PIN3_PWM_GEN_BIT        PWM_GEN_2_BIT
#define PIN3_PWM_OUT            PWM_OUT_4
#define PIN3_PWM_OUT_BIT        PWM_OUT_4_BIT
#define PIN3_PORT_BIT_NUM       GPIO_PIN_4
#define PIN3_PORT_PCTL_FUN      GPIO_PE4_M0PWM4
#define PIN3_PORT_ADDR_BASE     GPIO_PORTE_BASE
#define PIN3_PORT_CLK_BASE      SYSCTL_PERIPH_GPIOE

#define PIN4_PWM_GEN            PWM_GEN_3
#define PIN4_PWM_GEN_BIT        PWM_GEN_3_BIT
#define PIN4_PWM_OUT            PWM_OUT_6
#define PIN4_PWM_OUT_BIT        PWM_OUT_6_BIT
#define PIN4_PORT_BIT_NUM       GPIO_PIN_4
#define PIN4_PORT_PCTL_FUN      GPIO_PC4_M0PWM6
#define PIN4_PORT_ADDR_BASE     GPIO_PORTC_BASE
#define PIN4_PORT_CLK_BASE      SYSCTL_PERIPH_GPIOC

#endif /* QUADPWM_CONFIG_H_ */
