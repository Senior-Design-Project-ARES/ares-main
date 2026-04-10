#ifndef ENCODER_BACKEND_H
#define ENCODER_BACKEND_H

#include <stdint.h>

void    encoder_backend_init_all(void);
void    encoder_backend_poll(uint8_t encoder_id);
int32_t encoder_backend_get_and_reset_counts(uint8_t encoder_id);

#endif /* ENCODER_BACKEND_H */
