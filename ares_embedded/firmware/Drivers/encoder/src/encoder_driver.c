#include "encoder_driver.h"

#include <math.h>

#include "encoder_backend.h"
#include "encoder_config.h"

static uint8_t  g_initialised = 0U;
static uint32_t g_prev_ms     = 0U;
static float    g_rpm[ENCODER_COUNT];

void encoder_driver_init(uint32_t now_ms)
{
    encoder_backend_init_all();

    g_prev_ms     = now_ms;
    g_initialised = 1U;

    for (uint8_t i = 0U; i < ENCODER_COUNT; ++i)
    {
        (void)encoder_backend_get_and_reset_counts(i);
        g_rpm[i] = 0.0f;
    }
}

void encoder_driver_update(uint32_t now_ms)
{
    if (!g_initialised)
    {
        return;
    }

    uint32_t elapsed = now_ms - g_prev_ms;
    if (elapsed == 0U)
    {
        return;
    }

    for (uint8_t i = 0U; i < ENCODER_COUNT; ++i)
    {
        int32_t counts = encoder_backend_get_and_reset_counts(i);
        g_rpm[i] = ((float)counts * 60000.0f) /
                   ((float)ENCODER_COUNTS_PER_REV * (float)elapsed);
    }

    g_prev_ms = now_ms;
}

float encoder_driver_get_rpm(uint8_t encoder_id)
{
    if (!g_initialised || encoder_id >= ENCODER_COUNT)
    {
        return 0.0f;
    }
    return g_rpm[encoder_id];
}

float encoder_driver_get_rad_per_sec(uint8_t encoder_id)
{
    if (!g_initialised || encoder_id >= ENCODER_COUNT)
    {
        return 0.0f;
    }

    const float rpm = g_rpm[encoder_id];
    const float two_pi_over_60 = 0.104719755f; /* 2*pi/60 */
    return rpm * two_pi_over_60;
}

int32_t encoder_driver_get_and_reset_counts(uint8_t encoder_id)
{
    return encoder_backend_get_and_reset_counts(encoder_id);
}

