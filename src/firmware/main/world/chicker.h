#pragma once

struct Chicker;
typedef struct Chicker Chicker;

Chicker* Chicker__construct(void (*kick)(float speed_m_per_s),
                            void (*chip)(float distance_m), void (*enable_autokick)(void),
                            void (*enable_autochip)(void), void (*disable_autokick)(void),
                            void (*disable_autochip)(void));
void Chicker__kick(Chicker* this, float speed_m_per_s);
void Chicker__chip(Chicker* this, float distance_m);
void Chicker__enableAutokick(Chicker* this);
void Chicker__enableAutochip(Chicker* this);
void Chicker__disableAutokick(Chicker* this);
void Chicker__disableAutochip(Chicker* this);
