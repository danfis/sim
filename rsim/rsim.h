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

#ifndef _SIM_RSIM_H_
#define _SIM_RSIM_H_

#include <stdint.h>
#include <netinet/in.h>
#include "compiler.h"

//#define RSIM_BUFSIZE 4096
#define RSIM_BUFSIZE 1

#define RSIM_MSG_INIT 1
#define RSIM_MSG_PING 10
#define RSIM_MSG_PONG 11

struct _rsim_msg_t {
    char type;
} sim_packed;
typedef struct _rsim_msg_t rsim_msg_t;

struct _rsim_msg_init_t {
    char type;
    uint16_t id;
} sim_packed;
typedef struct _rsim_msg_init_t rsim_msg_init_t;


int rsimMsgSendInit(int sock, uint16_t id);
int rsimMsgSendPing(int sock);
int rsimMsgSendPong(int sock);


struct _rsim_msg_reader_t {
    int sock; /*!< Socket for reading and writing data into stream */
    char buf[RSIM_BUFSIZE];
    char *bufstart, *bufend;
};
typedef struct _rsim_msg_reader_t rsim_msg_reader_t;

void rsimMsgReaderInit(rsim_msg_reader_t *c, int sock);
rsim_msg_t *rsimMsgReaderNext(rsim_msg_reader_t *r);
int rsimMsgSend(rsim_msg_t *msg, int sock);


struct _rsim_client_t {
    uint16_t id;
    struct sockaddr_in addr; /*!< Address of server */
    int sock;

    rsim_msg_reader_t reader;
};
typedef struct _rsim_client_t rsim_client_t;

int rsimClientConnect(rsim_client_t *c, const char *addr, uint16_t port, uint16_t id);
void rsimClientClose(rsim_client_t *c);
rsim_msg_t *rsimClientNextMsg(rsim_client_t *c);
int rsimClientSendMsg(rsim_client_t *c, rsim_msg_t *m);

#endif /* _SIM_RSIM_H_ */
