/**
 *
 * @file tty.h
 * @author CM Geldenhuys
 * @date 31 Jul. 2020
 *
 * @brief TTY prompt for basic interactions with the MCU using a REPL
 *
 *
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx_hal.h"

#define LED_HIGH GPIO_PIN_SET
#define LED_LOW GPIO_PIN_RESET

#define LED_SET_LOW(__port__, __pin__) LED_WRITE((__port__), (__pin__), LED_LOW)
#define LED_SET_HIGH(__port__, __pin__) LED_WRITE((__port__), (__pin__), LED_HIGH)
#define LED_WRITE(__port__, __pin__, __state__) HAL_GPIO_WritePin((__port__), (__pin__), (__state__))
#define LED_TOGGLE(__port__, __pin__) HAL_GPIO_TogglePin((__port__), (__pin__))
#define LED_READ(__port__, __pin__) HAL_GPIO_ReadPin((__port__), (__pin__))

// TODO Write in a more generic way to self generate these functions after registering the LED's pin and port
#if defined(LED_ORANGE_GPIO_Port) && !defined(LED_ORANGE_PORT)
#define LED_ORANGE_PORT LED_ORANGE_GPIO_Port
#endif
#if defined(LED_ORANGE_Pin) && !defined(LED_ORANGE_PIN)
#define LED_ORANGE_PIN LED_ORANGE_Pin
#endif

#if !defined(LED_ORANGE_PORT) || !defined(LED_ORANGE_PIN)
#warn "LED_ORANGE not defined"
#else
#define LED_ORANGE_LOW LED_LOW
#define LED_ORANGE_HIGH LED_HIGH
#define LED_ORANGE_SET_LOW() LED_ORANGE_WRITE(LED_ORANGE_LOW)
#define LED_ORANGE_SET_HIGH() LED_ORANGE_WRITE(LED_ORANGE_HIGH)
#define LED_ORANGE_WRITE(__state__) LED_WRITE(LED_ORANGE_PORT, LED_ORANGE_PIN, (__state__))
#define LED_ORANGE_TOGGLE() LED_TOGGLE(LED_ORANGE_PORT, LED_ORANGE_PIN)
#define LED_ORANGE_READ() LED_READ(LED_ORANGE_PORT, LED_ORANGE_PIN)
#endif

#if defined(LED_BLUE_GPIO_Port) && !defined(LED_BLUE_PORT)
#define LED_BLUE_PORT LED_BLUE_GPIO_Port
#endif
#if defined(LED_BLUE_Pin) && !defined(LED_BLUE_PIN)
#define LED_BLUE_PIN LED_BLUE_Pin
#endif

#if !defined(LED_BLUE_PORT) || !defined(LED_BLUE_PIN)
#warn "LED_BLUE not defined"
#else
#define LED_BLUE_LOW LED_LOW
#define LED_BLUE_HIGH LED_HIGH
#define LED_BLUE_SET_LOW() LED_BLUE_WRITE(LED_BLUE_LOW)
#define LED_BLUE_SET_HIGH() LED_BLUE_WRITE(LED_BLUE_HIGH)
#define LED_BLUE_WRITE(__state__) LED_WRITE(LED_BLUE_PORT, LED_BLUE_PIN, (__state__))
#define LED_BLUE_TOGGLE() LED_TOGGLE(LED_BLUE_PORT, LED_BLUE_PIN)
#define LED_BLUE_READ() LED_READ(LED_BLUE_PORT, LED_BLUE_PIN)
#endif

#endif //LED_H_
