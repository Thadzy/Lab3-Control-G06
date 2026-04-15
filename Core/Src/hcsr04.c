/**
  ******************************************************************************
  * @file    hcsr04.c
  * @brief   HC-SR04 Ultrasonic Distance Sensor Driver — Implementation
  *
  * Measurement sequence:
  *   1. Pull TRIG HIGH for 10 us  -> sensor fires ultrasonic burst
  *   2. Pull TRIG LOW
  *   3. Wait for ECHO to go HIGH  -> burst has left the sensor
  *   4. Record timer count        -> Value1 (rising edge timestamp)
  *   5. Wait for ECHO to go LOW   -> burst has returned
  *   6. Record timer count        -> Value2 (falling edge timestamp)
  *   7. Distance = (Value2 - Value1) * 17 / 1000  (cm)
  *
  * Timer requirement:
  *   The timer pointed to by the handle passed to HCSR04_Init() must be
  *   configured so that each tick = 1 microsecond.
  *   For a 170 MHz APB2 clock: Prescaler = 169 gives 1 MHz timer clock.
  ******************************************************************************
  */

#include "hcsr04.h"

/* ============================================================================
 * Private Variables
 * ============================================================================ */

/* Pointer to the timer handle supplied by the caller in HCSR04_Init() */
static TIM_HandleTypeDef *hcsr04_htim = NULL;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void delay_us(uint32_t us);

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
  * @brief  Busy-wait for a precise number of microseconds using the timer.
  *
  * Resets the timer counter to zero, then polls it until the requested
  * number of ticks has elapsed.  Since each tick = 1 us, the count equals
  * the elapsed time in microseconds.
  *
  * @note   This is a blocking delay. Do not call it from an ISR.
  * @param  us  Number of microseconds to wait (max 65535).
  */
static void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(hcsr04_htim, 0);
    while (__HAL_TIM_GET_COUNTER(hcsr04_htim) < us);
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

/**
  * @brief  Initialise the HC-SR04 driver.
  * @param  htim  Pointer to a TIM handle running at 1 MHz (1 tick = 1 us).
  */
void HCSR04_Init(TIM_HandleTypeDef *htim)
{
    /* Store the timer handle so all functions in this file can use it */
    hcsr04_htim = htim;

    /* Drive TRIG LOW so the sensor does not fire on power-up */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Trigger one measurement and return the distance in centimetres.
  *
  * Distance formula derivation:
  *   echo_time (us) = Value2 - Value1
  *
  *   distance (cm)  = echo_time * speed_of_sound / 2
  *                  = echo_time * 0.034 cm/us / 2
  *                  = echo_time * 0.017
  *                  = echo_time * 17 / 1000
  *
  *   Integer math is used (multiply by 17, then divide by 1000) to avoid
  *   float promotion which would truncate small distances to zero.
  *
  * @retval Distance in cm, or HCSR04_TIMEOUT_VALUE on timeout.
  */
uint16_t HCSR04_Read(void)
{
    /*
     * volatile prevents the compiler from caching the timer register value
     * in a CPU register across loop iterations.  Without it, an optimised
     * build (-O2) may read the register only once and loop forever because
     * the cached value never changes.
     */
    volatile uint32_t value1 = 0;
    volatile uint32_t value2 = 0;
    uint32_t          timeout_start;

    /* ------------------------------------------------------------------
     * Step 1: Send a 10 us trigger pulse.
     *
     * The HC-SR04 requires TRIG to be HIGH for at least 10 us to start
     * one measurement cycle.  After the pulse, the sensor autonomously
     * sends 8 bursts of 40 kHz ultrasound and listens for the echo.
     * ------------------------------------------------------------------ */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* ------------------------------------------------------------------
     * Step 2: Reset the timer counter before measuring the echo.
     *
     * The reset happens HERE rather than inside delay_us(), so that
     * value1 and value2 are both measured from the same zero baseline.
     * If the counter were left running from inside delay_us(), value1
     * would be non-zero and the subtraction would undercount.
     * ------------------------------------------------------------------ */
    __HAL_TIM_SET_COUNTER(hcsr04_htim, 0);

    /* ------------------------------------------------------------------
     * Step 3: Wait for ECHO to go HIGH (rising edge = sound has left).
     *
     * Timeout: HCSR04_ECHO_HIGH_TIMEOUT_MS (30 ms).
     * Subtraction-based comparison is used instead of addition to avoid
     * overflow when HAL_GetTick() wraps around after ~49 days.
     * ------------------------------------------------------------------ */
    timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - timeout_start) >= HCSR04_ECHO_HIGH_TIMEOUT_MS)
        {
            return HCSR04_TIMEOUT_VALUE;
        }
    }

    /* Latch the timer count at the moment ECHO went HIGH */
    value1 = __HAL_TIM_GET_COUNTER(hcsr04_htim);

    /* ------------------------------------------------------------------
     * Step 4: Wait for ECHO to go LOW (falling edge = echo received).
     *
     * Timeout: HCSR04_ECHO_LOW_TIMEOUT_MS (50 ms).
     * The longest valid echo pulse is ~38 ms (nothing within 4 m).
     * ------------------------------------------------------------------ */
    timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - timeout_start) >= HCSR04_ECHO_LOW_TIMEOUT_MS)
        {
            return HCSR04_TIMEOUT_VALUE;
        }
    }

    /* Latch the timer count at the moment ECHO went LOW */
    value2 = __HAL_TIM_GET_COUNTER(hcsr04_htim);

    /* ------------------------------------------------------------------
     * Step 5: Convert echo pulse width to centimetres.
     * ------------------------------------------------------------------ */
    return (uint16_t)((value2 - value1) * 17U / 1000U);
}
