#ifndef PROMISCUOUS_H
#define PROMISCUOUS_H

#include <stdbool.h>
#include <usb.h>

void promiscuous_init(void);
void promiscuous_on_enter(void);
void promiscuous_on_exit(void);
bool promiscuous_control_handler(const usb_setup_packet_t *pkt);

#endif
