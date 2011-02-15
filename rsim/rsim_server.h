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
#include "msg.h"
#include "list.h"

struct _rsim_server_t {
    struct sockaddr_in addr; /*!< Address of server */
    int sock;                /*!< Socket for reading tcp connections */

    sim_list_t sessions;
};
typedef struct _rsim_server_t rsim_server_t;

rsim_server_t *rsimServerNew(void);
void rsimServerDel(rsim_server_t *);
int rsimServerStart(rsim_server_t *, int port);

#endif /* _SIM_RSIM_SERVER_H_ */
