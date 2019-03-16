#ifndef FEEDBACK_H
#define FEEDBACK_H

void feedback_init(void);
void feedback_shutdown(void);
void feedback_pend_normal(void);
void feedback_pend_has_ball(void);
void feedback_pend_autokick(void);
void feedback_pend_build_ids(void);
void feedback_tick(void);

#endif
