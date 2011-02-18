/***
 * Remote sim
 * -----------
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

#ifndef _SIM_RSIM_H_
#define _SIM_RSIM_H_

#include <stdint.h>
#include <netinet/in.h>

#define RSIM_BUFSIZE 4096
//#define RSIM_BUFSIZE 1

#define RSIM_MSG_PING 1
#define RSIM_MSG_PONG 2

struct _rsim_msg_t {
    uint16_t id;
    char type;
};
typedef struct _rsim_msg_t rsim_msg_t;



struct _rsim_t {
    int sock;

    char buf[RSIM_BUFSIZE];
    char *bufstart, *bufend;
    rsim_msg_t *msg;
};
typedef struct _rsim_t rsim_t;

/**
 * Connects client to specified server:port and id of robot.
 * Returns 0 on success, -1 if server is not reachable or other client is
 * already registered to specified robot.
 */
int rsimConnect(rsim_t *, const char *addr, uint16_t port);

/**
 * Close previously established connection to server.
 */
void rsimClose(rsim_t *);

/**
 * Returns true if there are some pending data.
 */
int rsimHaveMsg(rsim_t *);

/**
 * Returns next message sent from server.
 * This is blocking call.
 * Returns NULL if connection failed or was closed or some invalid data
 * were read.
 */
const rsim_msg_t *rsimNextMsg(rsim_t *);


/**
 * Sends simple message.
 */
int rsimSendSimple(rsim_t *, uint16_t id, char type);

#endif /* _SIM_RSIM_H_ */
