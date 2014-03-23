#ifndef __CIN_API_H__
#define __CIN_API_H__

#include "cin.h"
#include "fifo.h"

void *cin_ctl_listen_thread(void* args);
uint32_t cin_ctl_get_packet(struct cin_port *cp, uint32_t *val);
//double cin_ctl_current_calc(uint16_t val);

#endif /* ifndef __CIN_API_H__ */
