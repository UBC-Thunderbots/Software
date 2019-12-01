#pragma once

struct Chicker;
typedef struct Chicker Chicker;

Chicker* Chicker_create(void (*kick)(float speed_m_per_s),
                            void (*chip)(float distance_m), void (*enable_autokick)(void),
                            void (*enable_autochip)(void), void (*disable_autokick)(void),
                            void (*disable_autochip)(void));
void Chicker_kick(Chicker* chicker, float speed_m_per_s);
void Chicker_chip(Chicker* chicker, float distance_m);
void Chicker_enableAutokick(Chicker* chicker);
void Chicker_enableAutochip(Chicker* chicker);
void Chicker_disableAutokick(Chicker* chicker);
void Chicker_disableAutochip(Chicker* chicker);
void Chicker_destroy(Chicker* chicker);
