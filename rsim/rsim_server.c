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

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "rsim_server.h"
#include "alloc.h"
#include "dbg.h"

#define RSIM_BUFSIZE 4096

struct _rsim_session_t {
    pthread_t th;
    int sock; /*!< Socket for reading and writing data into stream */
    char buf[RSIM_BUFSIZE];
    char *bufstart, *bufend;

    sim_list_t list;
};
typedef struct _rsim_session_t rsim_session_t;

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

    return s;
}

void rsimServerDel(rsim_server_t *s)
{
    sim_list_t *item;
    rsim_session_t *sess;

    while (!simListEmpty(&s->sessions)){
        item = simListNext(&s->sessions);
        sess = simListEntry(item, rsim_session_t, list);
        rsimSessionDel(s, sess);
    }

    if (s->sock > 0)
        close(s->sock);

    free(s);
}

int rsimServerStart(rsim_server_t *s, int port)
{
    size_t addrsize;
    int connfd;
    rsim_session_t *sess;

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

    for(;;){
        addrsize = sizeof(s->addr);
        connfd = accept(s->sock, (struct sockaddr *)&s->addr, &addrsize);

        if(connfd < 0){
            perror("error accept failed");
            continue;
        }

        DBG("New connection %d", connfd);

        sess = rsimSessionNew(s, connfd);
        rsimSessionStart(sess);

        /*
        sprintf(buf, "Conn %02d: ", connfd);
        write(1, buf, 9);
        while ((readsize = read(connfd, buf, BUFSIZE)) > 0){
            write(1, buf, readsize);

            if (buf[readsize - 1] == '\n')
                break;
        }

        write(connfd, "Hello client", 12);

        shutdown(connfd, SHUT_RDWR);
        */

        close(connfd);
    }

    close(s->sock);
    s->sock = -1;

    return 0;
}



static rsim_session_t *rsimSessionNew(rsim_server_t *s, int sock)
{
    rsim_session_t *sess;

    sess = SIM_ALLOC(rsim_session_t);
    simListAppend(&s->sessions, &sess->list);

    sess->sock = sock;
    sess->bufstart = sess->bufend = 0;

    return sess;
}

static void rsimSessionDel(rsim_server_t *s, rsim_session_t *sess)
{
    if (sess->sock)
        close(sess->sock);

    simListDel(&sess->list);
    free(sess);
}

static void rsimSessionStart(rsim_session_t *sess)
{
    int res;

    res = pthread_create(&sess->th, NULL, rsimSessionTh, sess);
    if (res == 0)
        pthread_join(sess->th, NULL);

}

static void *rsimSessionTh(void *_sess)
{
    rsim_session_t *sess = (rsim_session_t *)_sess;
    size_t readsize;

    readsize = read(sess->sock, sess->buf, RSIM_BUFSIZE);
    if (readsize <= 0){
        return NULL;
    }

    sess->bufstart = sess->buf;
    sess->bufend   = sess->buf + readsize;
    write(1, sess->buf, readsize);
    write(1, "\n", 1);

    return NULL;
}
