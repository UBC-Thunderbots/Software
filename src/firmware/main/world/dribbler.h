#pragma once

typedef struct {
    // TODO: change RPM to a float here?
    void (*set_speed)(uint32_t rpm);
} Dribbler;
