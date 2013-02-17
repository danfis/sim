/***
 * Remote sim
 * -----------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_RSIM_COMMON_H_
#define _SIM_RSIM_COMMON_H_

#include <stdint.h>
#include <netinet/in.h>
#include "rsim.h"

struct _rsim_msg_init_t {
    char type;
    uint16_t id;
} sim_packed;
typedef struct _rsim_msg_init_t rsim_msg_init_t;

/**
 * Initializes reader struct using established socket.
 */
void rsimMsgReaderInit(rsim_msg_reader_t *c, int sock);

/**
 * Destroys previously initalized reader struct.
 */
void rsimMsgReaderDestroy(rsim_msg_reader_t *c);

/**
 * Returns next message read from socket.
 * Note that pointer points inside reader struct and must not be free'd!
 */
const rsim_msg_t *rsimMsgReaderNext(rsim_msg_reader_t *r);

/**
 * Sends init message over given socket.
 */
int rsimMsgSendInit(int sock, uint16_t id);

/**
 * Sends ping/pong message.
 */
int rsimMsgSendPing(int sock);
int rsimMsgSendPong(int sock);

#endif /* _SIM_RSIM_COMMON_H_ */

