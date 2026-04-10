#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise all hardware quadrature encoders and internal RPM state.
 *
 * - now_ms should be HAL_GetTick() at the time of initialisation.
 * - Must be called once before any other encoder_driver_* API.
 */
void encoder_driver_init(uint32_t now_ms);

/**
 * Update internal wheel speed estimates and clear pulse counts.
 *
 * - Call periodically from the main loop with the current HAL_GetTick().
 * - The sample interval is inferred from successive now_ms values.
 */
void encoder_driver_update(uint32_t now_ms);

/**
 * Get the most recently computed wheel speed for encoder_id in RPM.
 * Returns 0.0f if encoder_id is out of range or not yet initialised.
 */
float encoder_driver_get_rpm(uint8_t encoder_id);

/**
 * Get the most recently computed wheel speed for encoder_id in rad/s.
 * Returns 0.0f if encoder_id is out of range or not yet initialised.
 */
float encoder_driver_get_rad_per_sec(uint8_t encoder_id);

/**
 * Get raw quadrature counts since the last call for encoder_id.
 * This is a thin wrapper around encoder_backend_get_and_reset_counts.
 */
int32_t encoder_driver_get_and_reset_counts(uint8_t encoder_id);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_DRIVER_H */

