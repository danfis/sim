/***
 * sim
 * ---------------------------------
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

#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include "common.h"
#include "alloc.h"
#include "dbg.h"


void rsimMsgReaderInit(rsim_msg_reader_t *c, int sock)
{
    c->sock = sock;
    c->bufstart = c->bufend = 0;
    c->msg = NULL;
}

void rsimMsgReaderDestroy(rsim_msg_reader_t *c)
{
    if (c->msg)
        free(c->msg);
}


#define STATE_TYPE 0
#define STATE_ID1  1
#define STATE_ID2  2
#define STATE_END  100

const rsim_msg_t *rsimMsgReaderNext(rsim_msg_reader_t *r)
{
    ssize_t readsize;
    uint16_t id;
    char type;
    rsim_msg_init_t *msg_init;
    int state;

    DBG("%d %d", (int)r->bufstart, (int)r->bufend);

    state = STATE_TYPE;
    while (r->bufstart == r->bufend && state != STATE_END){
        readsize = read(r->sock, r->buf, RSIM_BUFSIZE);
        DBG("readsize: %d", (int)readsize);
        if (readsize < 0){
            perror("");
        }

        if (readsize <= 0)
            return NULL;

        r->bufstart = r->buf;
        r->bufend = r->buf + readsize;

        while (r->bufstart != r->bufend && state != STATE_END){
            DBG("state: %d", state);
            if (state == STATE_TYPE){
                type = *r->bufstart;

                if (type == RSIM_MSG_INIT){
                    state = STATE_ID1;
                }else{
                    state = STATE_END;
                }
            }else if (state == STATE_ID1){
                state = 100;
                ((char *)&id)[0] = *r->bufstart;
                state = STATE_ID2;
            }else if (state == STATE_ID2){
                ((char *)&id)[1] = *r->bufstart;
                id = ntohs(id);
                state = STATE_END;
            }

            r->bufstart++;
        }
    }
    DBG2("");

    if (r->msg)
        free(r->msg);

    if (type == RSIM_MSG_INIT){
        msg_init = SIM_ALLOC(rsim_msg_init_t);
        msg_init->type = type;
        msg_init->id = id;
        r->msg = (rsim_msg_t *)msg_init;
    }else{
        r->msg = SIM_ALLOC(rsim_msg_t);
        r->msg->type = type;
    }

    return r->msg;
}


int rsimMsgSendInit(int sock, uint16_t id)
{
    rsim_msg_init_t msg;
    ssize_t size;

    msg.type = RSIM_MSG_INIT;
    msg.id = htons(id);

    size = write(sock, &msg, sizeof(msg));
    fprintf(stderr, "size: %d, %d, %d\n", (int)size, (int)sizeof(msg),
            (int)sizeof(rsim_msg_init_t));
    if (size == sizeof(msg))
        return 0;
    return -1;
}

int rsimMsgSendPing(int sock)
{
    rsim_msg_t msg;
    ssize_t size;

    msg.type = RSIM_MSG_PING;
    size = write(sock, &msg, sizeof(msg));
    if (size == sizeof(msg))
        return 0;
    return -1;
}

int rsimMsgSendPong(int sock)
{
    rsim_msg_t msg;
    ssize_t size;

    msg.type = RSIM_MSG_PONG;
    size = write(sock, &msg, sizeof(msg));
    if (size == sizeof(msg))
        return 0;
    return -1;
}


