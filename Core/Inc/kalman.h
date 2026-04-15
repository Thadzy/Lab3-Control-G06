/**
  ******************************************************************************
  * @file    kalman.h
  * @brief   2-State Discrete Kalman Filter with Scalar Measurement
  *
  * State vector (2 states):
  *   x[0] = distance   (cm)
  *   x[1] = velocity   (cm/s)
  *
  * System model:
  *   x[k+1] = A * x[k] + w[k]       w ~ N(0, Q)   process noise (2x2)
  *   z[k]   = H * x[k] + v[k]       v ~ N(0, R)   measurement noise (scalar)
  *
  * State transition matrix A (2x2):
  *   A = | 1   dt |
  *       | 0    1 |
  *
  *   dt = sample period in seconds  (e.g. 0.06 for 60 ms loop)
  *   Row 0: distance += velocity * dt
  *   Row 1: velocity is assumed constant between samples
  *
  * Observation matrix H (1x2):
  *   H = | 1   0 |
  *   The sensor measures distance only, not velocity.
  *
  * Process noise covariance Q (2x2):
  *   Q = | q0   0  |
  *       |  0  q1  |
  *   q0 = variance of distance noise
  *   q1 = variance of velocity noise
  *
  * Measurement noise covariance R (scalar):
  *   R = variance of raw sensor readings.
  *   Measure this: collect 100 readings of a stationary target, compute variance.
  *
  * Key insight: innovation is still scalar
  *   S = H * P_pred * H^T + R   is a 1x1 scalar because H is 1x2.
  *   The Kalman gain inversion is a simple division, not a matrix inverse.
  *   No math library needed.
  *
  * Tuning guide:
  *   Increase q0/q1 : trust measurements more  -> faster response, noisier
  *   Decrease q0/q1 : trust the model more     -> smoother, slower to react
  *   Increase R     : sensor is very noisy     -> more smoothing applied
  *   Decrease R     : sensor is accurate       -> less smoothing applied
  ******************************************************************************
  */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

/* ============================================================================
 * 2-State Kalman Filter Structure
 *
 * All 2x2 matrices are stored as individual named floats.
 * This avoids any dependency on an external matrix library and keeps
 * the code readable on a microcontroller.
 *
 * Naming convention for 2x2 matrix elements: m_ij (row i, column j, 0-based)
 * ============================================================================ */
typedef struct
{
    /* Sample period in seconds.  Must match the HAL_Delay in the main loop. */
    float dt;

    /* Process noise covariance Q (2x2 diagonal matrix) */
    float q0;       /* Variance on the distance state */
    float q1;       /* Variance on the velocity state */

    /* Measurement noise covariance R (scalar, one sensor only) */
    float R;

    /* State estimate vector x (2x1) */
    float x0;       /* Estimated distance  (cm)   */
    float x1;       /* Estimated velocity  (cm/s) */

    /* Error covariance matrix P (2x2 symmetric)
     * Only three values stored because P[0][1] == P[1][0] always. */
    float p00;      /* P[0][0] */
    float p01;      /* P[0][1] = P[1][0] */
    float p11;      /* P[1][1] */

} Kalman2State_t;

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
  * @brief  Initialise a 2-state Kalman filter instance.
  *
  * Call once before the first Kalman2_Update().
  *
  * @param  kf        Pointer to a Kalman2State_t instance.
  * @param  dt        Sample period in seconds (e.g. 0.06 for 60 ms).
  * @param  q0        Process noise variance for the distance state.
  *                   Good starting point: 0.1
  * @param  q1        Process noise variance for the velocity state.
  *                   Good starting point: 1.0
  * @param  R         Measurement noise variance (compute from 100 readings).
  * @param  initial_d Initial distance estimate in cm (use first sensor reading).
  * @param  initial_v Initial velocity estimate in cm/s (use 0.0 at startup).
  */
void Kalman2_Init(Kalman2State_t *kf,
                  float           dt,
                  float           q0,
                  float           q1,
                  float           R,
                  float           initial_d,
                  float           initial_v);

/**
  * @brief  Run one predict-update cycle and return the filtered distance.
  *
  * Call every time a new sensor reading arrives.
  * Internally updates both x0 (distance) and x1 (velocity).
  *
  * @param  kf           Pointer to a Kalman2State_t instance.
  * @param  measurement  Raw distance in cm from HCSR04_Read().
  * @retval              Filtered distance estimate in cm.
  */
float Kalman2_Update(Kalman2State_t *kf, float measurement);

/**
  * @brief  Return the current velocity estimate from the filter.
  *
  * Updated automatically on every Kalman2_Update() call.
  * Positive value = object moving away from sensor.
  * Negative value = object moving toward sensor.
  *
  * @param  kf  Pointer to a Kalman2State_t instance.
  * @retval     Estimated velocity in cm/s.
  */
float Kalman2_GetVelocity(const Kalman2State_t *kf);

#endif /* KALMAN_H */
