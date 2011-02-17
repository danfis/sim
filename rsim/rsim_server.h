/***
 * sim
 * ---------------------------------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_RSIM_SERVER_H_
#define _SIM_RSIM_SERVER_H_

#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
#include "rsim.h"
#include "list.h"

typedef void (*rsim_session_cb)(const rsim_msg_t *msg, int sock, void *data);

struct _rsim_server_t {
    struct sockaddr_in addr; /*!< Address of server */
    int sock;                /*!< Socket for reading tcp connections */

    pthread_t th;

    pthread_mutex_t lock;
    sim_list_t sessions;
    sim_list_t sessions_del;
    sim_list_t callbacks;
};
typedef struct _rsim_server_t rsim_server_t;

rsim_server_t *rsimServerNew(void);
void rsimServerDel(rsim_server_t *);
int rsimServerStart(rsim_server_t *, int port);
int rsimServerStop(rsim_server_t *s);

int rsimServerRegister(rsim_server_t *s, uint16_t id, rsim_session_cb cb,
                       void *data);

#endif /* _SIM_RSIM_SERVER_H_ */
