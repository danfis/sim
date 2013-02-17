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

#include <stdio.h>
#include <string.h>
#include "rsim_server.h"
#include "common.h"
#include "alloc.h"
#include "dbg.h"


struct _rsim_session_cb_t {
    uint16_t id;
    rsim_session_cb cb;
    void *data;
    int occupied;

    sim_list_t list;
};
typedef struct _rsim_session_cb_t rsim_session_cb_t;

struct _rsim_session_t {
    rsim_server_t *server;
    rsim_session_cb_t *cb;
    void *cbdata;

    uint16_t id;

    int sock;
    rsim_msg_reader_t reader;

    pthread_t th;
    sim_list_t list;
};
typedef struct _rsim_session_t rsim_session_t;

static void *rsimServerTh(void *_s);
static void rsimServerDelDeadSessions(rsim_server_t *s);
static int rsimServerCBIsOccupied(rsim_server_t *s, uint16_t id);
static int rsimServerCBIsRegistered(rsim_server_t *s, uint16_t id);
static rsim_session_cb_t *rsimServerCB(rsim_server_t *s, uint16_t id);

static rsim_session_t *rsimSessionNew(rsim_server_t *s, int sock);
static void rsimSessionDel(rsim_server_t *s, rsim_session_t *sess);
static void rsimSessionStart(rsim_session_t *sess);
static void *rsimSessionTh(void *_sess);

rsim_server_t *rsimServerNew(void)
{
    rsim_server_t *s;

    s = SIM_ALLOC(rsim_server_t);
    s->sock = -1;

    simListInit(&s->sessions);
    simListInit(&s->sessions_del);
    simListInit(&s->callbacks);
    pthread_mutex_init(&s->lock, NULL);

    return s;
}

void rsimServerDel(rsim_server_t *s)
{
    sim_list_t *item;
    rsim_session_t *sess;
    rsim_session_cb_t *cb;

    rsimServerStop(s);

    pthread_mutex_lock(&s->lock);
    while (!simListEmpty(&s->sessions)){
        item = simListNext(&s->sessions);
        sess = simListEntry(item, rsim_session_t, list);
        rsimSessionDel(s, sess);
    }

    while (!simListEmpty(&s->callbacks)){
        item = simListNext(&s->callbacks);
        cb = simListEntry(item, rsim_session_cb_t, list);
        simListDel(item);
        free(cb);
    }
    pthread_mutex_unlock(&s->lock);

    rsimServerDelDeadSessions(s);

    if (s->sock > 0)
        close(s->sock);

    free(s);
}

int rsimServerStart(rsim_server_t *s, int port)
{
    // create tcp socket
    s->sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

    if(s->sock < 0){
        perror("can not create socket");
        return -1;
    }

    memset(&s->addr, 0, sizeof(s->addr));

    s->addr.sin_family = AF_INET;
    s->addr.sin_port = htons(port);
    s->addr.sin_addr.s_addr = INADDR_ANY;

    if(bind(s->sock,(struct sockaddr *)&s->addr, sizeof(s->addr)) < 0){
        perror("error bind failed");
        close(s->sock);
        s->sock = -1;
        return -1;
    }

    if(listen(s->sock, 10) < 0){
        perror("error listen failed");
        close(s->sock);
        s->sock = -1;
    }

    pthread_create(&s->th, NULL, rsimServerTh, (void *)s);

    return 0;
}

int rsimServerStop(rsim_server_t *s)
{
    pthread_cancel(s->th);
    pthread_join(s->th, NULL);
    close(s->sock);
    s->sock = -1;

    return 0;
}

int rsimServerRegister(rsim_server_t *s, uint16_t id, rsim_session_cb cb,
                       void *data)
{
    rsim_session_cb_t *cbs;

    if (rsimServerCBIsRegistered(s, id))
        return -1;


    cbs = SIM_ALLOC(rsim_session_cb_t);
    cbs->id = id;
    cbs->cb = cb;
    cbs->data = data;
    cbs->occupied = 0;

    pthread_mutex_lock(&s->lock);
    simListAppend(&s->callbacks, &cbs->list);
    pthread_mutex_unlock(&s->lock);

    return 0;
}

static void *rsimServerTh(void *_s)
{
    rsim_server_t *s = (rsim_server_t *)_s;
    int connfd;
    size_t addrsize;
    rsim_session_t *sess;

    for(;;){
        addrsize = sizeof(s->addr);
        connfd = accept(s->sock, (struct sockaddr *)&s->addr, &addrsize);

        if(connfd < 0){
            perror("error accept failed");
            continue;
        }

        DBG("New connection %d", connfd);

        sess = rsimSessionNew(s, connfd);

        pthread_mutex_lock(&s->lock);
        simListAppend(&s->sessions, &sess->list);
        pthread_mutex_unlock(&s->lock);

        rsimSessionStart(sess);

        rsimServerDelDeadSessions(s);
    }

    return NULL;
}

static int rsimServerCBIsOccupied(rsim_server_t *s, uint16_t id)
{
    sim_list_t *item;
    rsim_session_cb_t *cb;
    int ret = 0;

    pthread_mutex_lock(&s->lock);
    simListForEach(&s->callbacks, item){
        cb = simListEntry(item, rsim_session_cb_t, list);
        if (cb->id == id && cb->occupied){
            ret = 1;
            break;
        }
    }
    pthread_mutex_unlock(&s->lock);

    return ret;
}

static int rsimServerCBIsRegistered(rsim_server_t *s, uint16_t id)
{
    sim_list_t *item;
    rsim_session_cb_t *cb;
    int ret = 0;

    pthread_mutex_lock(&s->lock);
    simListForEach(&s->callbacks, item){
        cb = simListEntry(item, rsim_session_cb_t, list);
        if (cb->id == id){
            ret = 1;
            break;
        }
    }
    pthread_mutex_unlock(&s->lock);

    return ret;
}

static rsim_session_cb_t *rsimServerCB(rsim_server_t *s, uint16_t id)
{
    sim_list_t *item;
    rsim_session_cb_t *cb;
    rsim_session_cb_t *ret = NULL;

    pthread_mutex_lock(&s->lock);
    simListForEach(&s->callbacks, item){
        cb = simListEntry(item, rsim_session_cb_t, list);
        if (cb->id == id){
            ret = cb;
            break;
        }
    }
    pthread_mutex_unlock(&s->lock);

    return ret;
}

static void rsimServerDelDeadSessions(rsim_server_t *s)
{
    sim_list_t *item;
    rsim_session_t *sess;

    pthread_mutex_lock(&s->lock);

    while (!simListEmpty(&s->sessions_del)){
        item = simListNext(&s->sessions_del);
        sess = simListEntry(item, rsim_session_t, list);
        rsimSessionDel(s, sess);
    }

    pthread_mutex_unlock(&s->lock);
}


static rsim_session_t *rsimSessionNew(rsim_server_t *s, int sock)
{
    rsim_session_t *sess;

    sess = SIM_ALLOC(rsim_session_t);
    sess->server = s;
    sess->cb = NULL;

    sess->sock = sock;
    rsimMsgReaderInit(&sess->reader, sess->sock);

    return sess;
}

static void rsimSessionDel(rsim_server_t *s, rsim_session_t *sess)
{
    pthread_join(sess->th, NULL);

    if (sess->sock)
        close(sess->sock);

    rsimMsgReaderDestroy(&sess->reader);

    simListDel(&sess->list);
    free(sess);
}

static void rsimSessionStart(rsim_session_t *sess)
{
    pthread_create(&sess->th, NULL, rsimSessionTh, sess);
}

static void __rsimSessionRemove(rsim_session_t *sess)
{
    pthread_mutex_lock(&sess->server->lock);
    if (sess->cb)
        sess->cb->occupied = 0;
    simListDel(&sess->list);
    simListAppend(&sess->server->sessions_del, &sess->list);
    pthread_mutex_unlock(&sess->server->lock);
}

static void *rsimSessionTh(void *_sess)
{
    rsim_session_t *sess = (rsim_session_t *)_sess;
    const rsim_msg_t *msg;
    rsim_msg_init_t *msg_init;
    rsim_session_cb_t *cb;

    DBG2("1");
    msg = rsimMsgReaderNext(&sess->reader);
    if (msg == NULL || msg->type != RSIM_MSG_INIT){
        fprintf(stderr, "Invalid initial message.\n");
        __rsimSessionRemove(sess);
        return NULL;
    }

    DBG2("2");
    msg_init = (rsim_msg_init_t *)msg;
    fprintf(stdout, "id: %d, type: %d\n", (int)msg_init->id, (int)msg->type);
    fflush(stdout);

    cb = rsimServerCB(sess->server, msg_init->id);
    if (!cb || cb->occupied){
        fprintf(stderr, "Callback not registered or already occupied.\n");
        __rsimSessionRemove(sess);
        return NULL;
    }

    pthread_mutex_lock(&sess->server->lock);
    cb->occupied = 1;
    pthread_mutex_unlock(&sess->server->lock);

    sess->id = msg_init->id;
    sess->cb = cb;
    rsimMsgSendInit(sess->sock, sess->id);

    while ((msg = rsimMsgReaderNext(&sess->reader)) != NULL){
        sess->cb->cb(msg, sess->sock, sess->cb->data);
    }

    __rsimSessionRemove(sess);

    return NULL;
}

