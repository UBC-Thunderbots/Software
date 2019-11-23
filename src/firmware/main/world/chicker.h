#pragma once

// TODO: jdoc for struct and each function
typedef struct
{
    void (*kick)(float speed_m_per_s);
    void (*chip)(float distance_m);
    void (*enable_autokick)(void);
    void (*enable_autochip)(void);
    void (*disable_autokick)(void);
    void (*disable_autochip)(void);
} Chicker;