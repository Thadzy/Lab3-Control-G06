/**
  ******************************************************************************
  * @file    hcsr04.h
  * @brief   HC-SR04 Ultrasonic Distance Sensor Driver — Public Interface
  *
  * How to use this library:
  *   1. Copy hcsr04.h and hcsr04.c into your project's Inc/ and Src/ folders.
  *   2. In STM32CubeIDE, right-click the Src folder -> Add Files -> hcsr04.c
  *   3. Add #include "hcsr04.h" at the top of main.c
  *   4. Call HCSR04_Init() once in main() after HAL_TIM_Base_Start()
  *   5. Call HCSR04_Read() inside your main loop to get distance in cm
  ******************************************************************************
  */

#ifndef HCSR04_H
#define HCSR04_H

#include "main.h"   /* Pulls in stm32g4xx_hal.h and all GPIO/TIM definitions */
#include <stdint.h>

/* ============================================================================
 * User Configuration
 * Edit these four lines to match your wiring.
 * ============================================================================ */

#define HCSR04_TRIG_PIN     GPIO_PIN_9   /* Arduino D8 on NUCLEO-G474RE */
#define HCSR04_TRIG_PORT    GPIOA

#define HCSR04_ECHO_PIN     GPIO_PIN_8   /* Arduino D7 on NUCLEO-G474RE */
#define HCSR04_ECHO_PORT    GPIOA

/* ============================================================================
 * Timeout Configuration
 * ============================================================================ */

/*
 * Maximum time (ms) to wait for the ECHO pin to go HIGH after the trigger.
 * The HC-SR04 normally responds within 25-30 ms.  30 ms is a safe margin.
 */
#define HCSR04_ECHO_HIGH_TIMEOUT_MS    30U

/*
 * Maximum time (ms) to wait for the ECHO pin to go LOW.
 * The longest echo pulse is ~38 ms (no obstacle detected).
 * 50 ms gives a comfortable margin above that.
 */
#define HCSR04_ECHO_LOW_TIMEOUT_MS     50U

/*
 * Sentinel value returned by HCSR04_Read() when a timeout occurs.
 * This value (65535) is outside the HC-SR04's physical range (2-400 cm),
 * so the caller can detect it and discard the reading.
 */
#define HCSR04_TIMEOUT_VALUE           0xFFFFU

/*
 * Minimum safe delay (ms) between consecutive measurements.
 * The HC-SR04 datasheet recommends at least 60 ms between triggers
 * to prevent echo interference from the previous measurement.
 */
#define HCSR04_MIN_INTERVAL_MS         60U

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
  * @brief  Initialise the HC-SR04 driver.
  *
  * Must be called once after HAL_TIM_Base_Start() and before HCSR04_Read().
  * Drives TRIG LOW so the sensor does not trigger spuriously on power-up.
  *
  * @param  htim  Pointer to the TIM handle configured at 1 MHz (1 tick = 1 us).
  *               Pass &htim1 from main.c.
  */
void HCSR04_Init(TIM_HandleTypeDef *htim);

/**
  * @brief  Trigger one measurement and return the distance.
  *
  * Blocking function.  Sends the trigger pulse, waits for the echo,
  * then calculates and returns the distance.
  *
  * @retval Distance in centimetres (2 to 400 cm), or
  *         HCSR04_TIMEOUT_VALUE (0xFFFF) if no echo was received in time.
  */
uint16_t HCSR04_Read(void);

#endif /* HCSR04_H */
