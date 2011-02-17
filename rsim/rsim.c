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

#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include "rsim.h"
#include "common.h"
#include "alloc.h"
#include "dbg.h"

int rsimConnect(rsim_t *c, const char *addr, uint16_t port, uint16_t id)
{
    int res;
    const rsim_msg_t *response;

    c->id = id;

    c->sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (c->sock < 0){
        perror("cannot create socket");
        return -1;
    }

    memset(&c->addr, 0, sizeof(c->addr));

    c->addr.sin_family = AF_INET;
    c->addr.sin_port = htons(port);
    res = inet_pton(AF_INET, addr, &c->addr.sin_addr);
    if (res < 0){
        perror("error: first parameter is not a valid address family");
        close(c->sock);
        return -1;
    }else if (res == 0){
        perror("char string (second parameter does not contain valid ipaddress");
        close(c->sock);
        return -1;
    }

    if (connect(c->sock, (struct sockaddr *)&c->addr, sizeof(c->addr)) < 0){
        perror("connect failed");
        close(c->sock);
        return -1;
    }

    rsimMsgReaderInit(&c->reader, c->sock);

    // Login to serverj
    if (rsimMsgSendInit(c->sock, c->id) != 0){
        fprintf(stderr, "Unable to send initial message.\n");
        close(c->sock);
        return -1;
    }

    response = rsimMsgReaderNext(&c->reader);
    if (!response){
        fprintf(stderr, "Server is not responding.\n");
        close(c->sock);
        return -1;
    }
    if (response->type != RSIM_MSG_INIT){
        fprintf(stderr, "Invalid response from server.\n");
        close(c->sock);
        return -1;
    }
    if (((rsim_msg_init_t *)response)->id != c->id){
        fprintf(stderr, "Robot with id `%d' is already in use.\n", c->id);
        close(c->sock);
        return -1;
    }

    DBG("response id: %d, type: %d", (int)((rsim_msg_init_t *)response)->id, (int)response->type);

    return 0;
}

void rsimClose(rsim_t *c)
{
    rsimMsgReaderDestroy(&c->reader);
    close(c->sock);
}

const rsim_msg_t *rsimNextMsg(rsim_t *c)
{
    return rsimMsgReaderNext(&c->reader);
}

int rsimSendPing(rsim_t *r)
{
    return rsimMsgSendPing(r->sock);
}
