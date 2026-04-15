/**
  ******************************************************************************
  * @file    kalman.c
  * @brief   2-State Discrete Kalman Filter — Implementation
  *
  * All matrix operations are expanded manually into scalar arithmetic.
  * No external math library is required.
  *
  * Notation used in comments:
  *   ()^-  means "predicted" (before incorporating the measurement)
  *   ()    means "updated"   (after incorporating the measurement)
  *   T     means matrix transpose
  *
  * Full equations (vector/matrix form):
  *
  *   PREDICT:
  *     x^-  = A * x
  *     P^-  = A * P * A^T + Q
  *
  *   UPDATE:
  *     S    = H * P^- * H^T + R          (scalar: innovation covariance)
  *     K    = P^- * H^T / S              (2x1 Kalman gain vector)
  *     x    = x^- + K * (z - H * x^-)   (corrected state estimate)
  *     P    = (I - K * H) * P^-          (updated error covariance)
  *
  * Expansion for A = |1 dt|, H = |1 0|:
  *                   |0  1|
  *
  *   With these specific matrices, every product reduces to simple
  *   additions and multiplications as shown step by step below.
  ******************************************************************************
  */

#include "kalman.h"

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

/**
  * @brief  Initialise a 2-state Kalman filter instance.
  */
void Kalman2_Init(Kalman2State_t *kf,
                  float           dt,
                  float           q0,
                  float           q1,
                  float           R,
                  float           initial_d,
                  float           initial_v)
{
    kf->dt = dt;
    kf->q0 = q0;
    kf->q1 = q1;
    kf->R  = R;

    /* Seed the state estimate with the first real measurement */
    kf->x0 = initial_d;
    kf->x1 = initial_v;

    /*
     * Seed the error covariance matrix P with the identity matrix.
     * This expresses maximum initial uncertainty.
     * The filter will converge to a good P within a few iterations.
     *
     * P = | 1  0 |
     *     | 0  1 |
     */
    kf->p00 = 1.0f;
    kf->p01 = 0.0f;
    kf->p11 = 1.0f;
}

/**
  * @brief  Run one predict-update cycle and return the filtered distance.
  *
  * =========================================================================
  * PREDICT STEP (time update)
  * =========================================================================
  *
  * Goal: propagate the state and covariance forward one time step using
  * the system model, before seeing the new measurement.
  *
  * --- State prediction: x^- = A * x ---
  *
  *   A = |1  dt|,   x = |x0|
  *       |0   1|        |x1|
  *
  *   x0^- = 1*x0 + dt*x1 = x0 + dt*x1    (distance moves by velocity*dt)
  *   x1^- = 0*x0 +  1*x1 = x1            (velocity stays the same)
  *
  * --- Covariance prediction: P^- = A * P * A^T + Q ---
  *
  *   P = |p00  p01|,   A^T = |1   0|
  *       |p01  p11|           |dt  1|
  *
  *   Step 1 — compute A * P:
  *     (A*P)[0][0] = 1*p00 + dt*p01 = p00 + dt*p01
  *     (A*P)[0][1] = 1*p01 + dt*p11 = p01 + dt*p11
  *     (A*P)[1][0] = 0*p00 +  1*p01 = p01
  *     (A*P)[1][1] = 0*p01 +  1*p11 = p11
  *
  *   Step 2 — compute (A*P) * A^T, then add Q:
  *     P^-[0][0] = (p00 + dt*p01)*1 + (p01 + dt*p11)*dt  + q0
  *               = p00 + 2*dt*p01 + dt^2*p11              + q0
  *     P^-[0][1] = (p00 + dt*p01)*0 + (p01 + dt*p11)*1
  *               = p01 + dt*p11
  *     P^-[1][1] = p01*0 + p11*1                          + q1
  *               = p11                                     + q1
  *
  *   P is symmetric so P^-[1][0] = P^-[0][1].
  *
  * =========================================================================
  * UPDATE STEP (measurement update)
  * =========================================================================
  *
  * Goal: correct the prediction using the actual sensor reading z.
  *
  * --- Innovation (residual): y = z - H * x^- ---
  *
  *   H = |1  0|
  *   H * x^- = 1*x0^- + 0*x1^- = x0^-
  *   y = z - x0^-                    (how far off was our distance prediction)
  *
  * --- Innovation covariance: S = H * P^- * H^T + R ---
  *
  *   H * P^- = |p00^-  p01^-|   (first row of P^-)
  *   H * P^- * H^T = p00^-      (dot with H^T = [1,0]^T picks first element)
  *   S = p00^- + R              (scalar — this is why no matrix inverse needed)
  *
  * --- Kalman gain: K = P^- * H^T / S ---
  *
  *   P^- * H^T = |p00^-| * [1]  = |p00^-|   (H^T picks the first column)
  *               |p01^-|   [0]    |p01^-|
  *
  *   K = |p00^- / S|   (2x1 vector, one gain per state)
  *       |p01^- / S|
  *
  * --- State update: x = x^- + K * y ---
  *
  *   x0 = x0^- + K0 * y
  *   x1 = x1^- + K1 * y
  *
  * --- Covariance update: P = (I - K * H) * P^- ---
  *
  *   K * H = |K0| * |1  0| = |K0   0 |
  *           |K1|             |K1   0 |
  *
  *   I - K*H = |1-K0   0 |
  *             |-K1    1 |
  *
  *   P = (I - K*H) * P^-:
  *     P[0][0] = (1-K0)*p00^- + 0*p01^-    = (1-K0)*p00^-
  *     P[0][1] = (1-K0)*p01^- + 0*p11^-    = (1-K0)*p01^-
  *     P[1][0] = (-K1) *p00^- + 1*p01^-    = p01^- - K1*p00^-
  *     P[1][1] = (-K1) *p01^- + 1*p11^-    = p11^- - K1*p01^-
  *
  *   Note: P remains symmetric after the update.
  */
float Kalman2_Update(Kalman2State_t *kf, float measurement)
{
    const float dt = kf->dt;

    /* ---- PREDICT: state -------------------------------------------------- */
    float x0_pred = kf->x0 + dt * kf->x1;   /* distance + velocity * dt */
    float x1_pred = kf->x1;                  /* velocity unchanged       */

    /* ---- PREDICT: covariance --------------------------------------------- */
    float p00_pred = kf->p00 + dt * (kf->p01 + kf->p01) + (dt * dt) * kf->p11 + kf->q0;
    float p01_pred = kf->p01 + dt * kf->p11;
    float p11_pred = kf->p11 + kf->q1;

    /* ---- UPDATE: innovation (residual) ----------------------------------- */
    float innovation = measurement - x0_pred;   /* z - H * x^- */

    /* ---- UPDATE: innovation covariance (scalar) -------------------------- */
    float S = p00_pred + kf->R;                 /* H*P^-*H^T + R */

    /* ---- UPDATE: Kalman gain (2x1 vector) -------------------------------- */
    float K0 = p00_pred / S;   /* gain for distance state */
    float K1 = p01_pred / S;   /* gain for velocity state */

    /* ---- UPDATE: corrected state estimate -------------------------------- */
    kf->x0 = x0_pred + K0 * innovation;
    kf->x1 = x1_pred + K1 * innovation;

    /* ---- UPDATE: updated error covariance -------------------------------- */
    kf->p00 = (1.0f - K0) * p00_pred;
    kf->p01 = (1.0f - K0) * p01_pred;
    kf->p11 = p11_pred - K1 * p01_pred;

    /* Return the filtered distance estimate */
    return kf->x0;
}

/**
  * @brief  Return the current velocity estimate.
  */
float Kalman2_GetVelocity(const Kalman2State_t *kf)
{
    return kf->x1;
}
