#ifndef LPS_H
#define LPS_H

#define LPS_ARRAY_SIZE 4

typedef float lps_values[LPS_ARRAY_SIZE];


void lps_init(void);
void lps_incr(void);
void lps_tick(void);
void lps_get_raw(lps_values lps_val);
void lps_get_pos(lps_values lps_val);

#endif
