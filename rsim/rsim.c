/***
 * sim
 * ----
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
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include "rsim.h"

static int rsimReadByte(rsim_t *c, char *b);
static int rsimReadID(rsim_t *c, uint16_t *id);
static int rsimReadType(rsim_t *c, char *type);
static int rsimReadFloat(rsim_t *c, float *f);

static int rsimWriteFloat(rsim_t *c, float f);

int rsimConnect(rsim_t *c, const char *ipaddr, uint16_t port)
{
    struct sockaddr_in addr;
    int res;

    c->sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (c->sock < 0){
        perror("cannot create socket");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    res = inet_pton(AF_INET, ipaddr, &addr.sin_addr);
    if (res < 0){
        perror("error: first parameter is not a valid address family");
        close(c->sock);
        return -1;
    }else if (res == 0){
        perror("char string (second parameter does not contain valid ipaddress");
        close(c->sock);
        return -1;
    }

    if (connect(c->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        perror("connect failed");
        close(c->sock);
        return -1;
    }

    c->bufstart = c->bufend = 0;
    c->msg = NULL;

    return 0;
}

void rsimClose(rsim_t *c)
{
    if (c->msg)
        free(c->msg);
    close(c->sock);
}

int rsimHaveMsg(rsim_t *c)
{
    fd_set fds;
    int ready, maxfd;
    struct timeval timeout;

    FD_ZERO(&fds);
    FD_SET(c->sock, &fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 1;


    maxfd = c->sock + 1;
    ready = select(maxfd, &fds, NULL, NULL, &timeout);
    return ready > 0;
}

const rsim_msg_t *rsimNextMsg(rsim_t *c)
{
    uint16_t id;
    char type;
    rsim_msg_float3_t *msgf3;
    rsim_msg_float4_t *msgf4;

    if (c->msg){
        free(c->msg);
        c->msg = NULL;
    }

    // first read ID
    if (rsimReadID(c, &id) != 0)
        return NULL;

    // then type
    if (rsimReadType(c, &type) != 0)
        return NULL;

    if (type == RSIM_MSG_POS){
        msgf3 = (rsim_msg_float3_t *)malloc(sizeof(rsim_msg_float3_t));

        if (rsimReadFloat(c, msgf3->f + 0) != 0)
            return NULL;
        if (rsimReadFloat(c, msgf3->f + 1) != 0)
            return NULL;
        if (rsimReadFloat(c, msgf3->f + 2) != 0)
            return NULL;

        c->msg = (rsim_msg_t *)msgf3;

    }else if (type == RSIM_MSG_ROT){
        msgf4 = (rsim_msg_float4_t *)malloc(sizeof(rsim_msg_float4_t));

        if (rsimReadFloat(c, msgf4->f + 0) != 0)
            return NULL;
        if (rsimReadFloat(c, msgf4->f + 1) != 0)
            return NULL;
        if (rsimReadFloat(c, msgf4->f + 2) != 0)
            return NULL;
        if (rsimReadFloat(c, msgf4->f + 3) != 0)
            return NULL;

        c->msg = (rsim_msg_t *)msgf4;

    }else{
        c->msg = (rsim_msg_t *)malloc(sizeof(rsim_msg_t));
    }

    c->msg->id   = id;
    c->msg->type = type;

    return c->msg;
}

int rsimSendSimple(rsim_t *c, uint16_t id, char type)
{
    ssize_t size;

    id = htons(id);
    size = write(c->sock, (void *)&id, sizeof(uint16_t));
    if (size != sizeof(uint16_t))
        return -1;

    size = write(c->sock, (void *)&type, sizeof(char));
    if (size != sizeof(char))
        return -1;

    return 0;
}

int rsimSendFloat(rsim_t *c, uint16_t id, char type, float f)
{
    if (rsimSendSimple(c, id, type) != 0)
        return -1;
    return rsimWriteFloat(c, f);
}


static int rsimReadByte(rsim_t *c, char *b)
{
    ssize_t readsize;

    if (c->bufstart == c->bufend){
        readsize = read(c->sock, c->buf, RSIM_BUFSIZE);
        if (readsize <= 0)
            return -1;

        c->bufstart = c->buf;
        c->bufend = c->buf + readsize;
    }

    *b = *c->bufstart;
    c->bufstart++;

    return 0;
}

static int rsimReadID(rsim_t *c, uint16_t *id)
{
    char *cid = (char *)id;

    if (rsimReadByte(c, cid + 0) != 0)
        return -1;
    if (rsimReadByte(c, cid + 1) != 0)
        return -1;

    *id = ntohs(*id);
    return 0;
}

static int rsimReadType(rsim_t *c, char *type)
{
    return rsimReadByte(c, type);
}

static int rsimReadFloat(rsim_t *r, float *f)
{
    uint32_t i;
    char *c, *cf;

    c = (char *)&i;
    if (rsimReadByte(r, c + 0) != 0)
        return -1;
    if (rsimReadByte(r, c + 1) != 0)
        return -1;
    if (rsimReadByte(r, c + 2) != 0)
        return -1;
    if (rsimReadByte(r, c + 3) != 0)
        return -1;

    i = ntohl(i);
    cf = (char *)f;
    cf[0] = c[0];
    cf[1] = c[1];
    cf[2] = c[2];
    cf[3] = c[3];

    return 0;
}

static int rsimWriteFloat(rsim_t *r, float f)
{
    ssize_t size;
    uint32_t i;
    char *c, *cf;

    cf = (char *)&f;
    c  = (char *)&i;
    c[0] = cf[0];
    c[1] = cf[1];
    c[2] = cf[2];
    c[3] = cf[3];
    i = htonl(i);

    size = write(r->sock, (void *)&i, 4);
    if (size != 4)
        return -1;
    return 0;
}
