#ifndef STATE_H
#define STATE_H

#include <stdint.h>

#define STATE_IS_IMU 1
#define STATE_TYPE STATE_IS_IMU

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
} State;

extern State m_state_cur;
extern State m_state_prev;

#endif /* STATE_H */
