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
#include "alloc.h"
#include "dbg.h"

void rsimMsgReaderInit(rsim_msg_reader_t *c, int sock)
{
    c->sock = sock;
    c->bufstart = c->bufend = 0;
}

rsim_msg_t *rsimMsgReaderNext(rsim_msg_reader_t *r)
{
    ssize_t readsize;
    uint16_t id;
    char type;
    rsim_msg_t *msg = NULL;
    int state; // 0 - id1, 1 - id2, 2 - type, 100 - end

    DBG("%d %d", (int)r->bufstart, (int)r->bufend);

    state = 0;
    while (r->bufstart == r->bufend && state != 100){
        readsize = read(r->sock, r->buf, RSIM_BUFSIZE);
        DBG("readsize: %d", (int)readsize);
        if (readsize < 0){
            perror("");
        }

        if (readsize <= 0)
            return NULL;

        r->bufstart = r->buf;
        r->bufend = r->buf + readsize;

        while (r->bufstart != r->bufend && state != 100){
            DBG("state: %d", state);
            if (state == 0){
                ((char *)&id)[0] = *r->bufstart;
                state = 1;
            }else if (state == 1){
                ((char *)&id)[1] = *r->bufstart;
                id = ntohs(id);
                state = 2;
            }else if (state == 2){
                type = *r->bufstart;
                state = 100;
            }

            r->bufstart++;
        }
    }
    DBG2("");

    msg = SIM_ALLOC(rsim_msg_t);
    msg->id = id;
    msg->type = type;

    return msg;
}

int rsimMsgSend(rsim_msg_t *msg, int sock)
{
    ssize_t writesize;
    uint16_t id;

    // write id
    id = htons(msg->id);
    writesize = write(sock, &id, sizeof(id));
    if (writesize != sizeof(id))
        return -1;

    // write type
    writesize = write(sock, &msg->type, 1);
    if (writesize != 1)
        return -1;

    return 0;
}




int rsimClientConnect(rsim_client_t *c, const char *addr, uint16_t port, uint16_t id)
{
    int res;
    rsim_msg_t msg, *response;

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
    msg.id = c->id;
    msg.type = RSIM_MSG_INIT;
    res = rsimClientSendMsg(c, &msg);
    if (res != 0){
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
    if (response->id != c->id){
        fprintf(stderr, "Robot with id `%d' is already in use.\n", c->id);
        close(c->sock);
        return -1;
    }

    DBG("response id: %d, type: %d", (int)response->id, (int)response->type);
    free(response);

    return 0;
}

void rsimClientClose(rsim_client_t *c)
{
    close(c->sock);
}

rsim_msg_t *rsimClientNextMsg(rsim_client_t *c)
{
    return rsimMsgReaderNext(&c->reader);
}

int rsimClientSendMsg(rsim_client_t *c, rsim_msg_t *m)
{
    return rsimMsgSend(m, c->sock);
}
