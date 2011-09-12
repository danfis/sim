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

#include <netinet/in.h>
#include <string.h>
#include <stdio.h>
#include <sys/select.h>
#include "comp/rserver.hpp"
#include "msg.hpp"
#include "sim.hpp"

namespace sim {
namespace comp {

void *RServerSession::thread(void *_s)
{
    RServerSession *s = (RServerSession *)_s;
    uint16_t id;
    char type;

    while (true){
        // first read ID
        if (s->_readID(&id) != 0)
            break;

        // then read type
        if (s->_readType(&type) != 0)
            break;

        DBG("id: " << id << " type: " << (int)type);

        if (type == RMessage::MSG_PING){
            s->_server->__addMessage(new RMessageInPing(id));
        }else if (type == RMessage::MSG_PONG){
            s->_server->__addMessage(new RMessageInPong(id));
        }else{
            s->_server->__addMessage(new RMessageIn(id, type));
        }
    }


    s->_server->__addSessionToJoin(s);
    return NULL;
}


RServerSession::RServerSession(RServer *s, int sock)
    : _server(s), _sock(sock), _bufstart(0), _bufend(0)
{
}

RServerSession::~RServerSession()
{
    DBG(this);
    pthread_join(_th, NULL);
    shutdown(_sock, SHUT_RDWR);
    close(_sock);
}

void RServerSession::run()
{
    pthread_create(&_th, NULL, thread, (void *)this);
}

void RServerSession::cancel()
{
    DBG(this);
    pthread_cancel(_th);
}

void RServerSession::sendMessage(const RMessageOut &msg)
{
    uint16_t id;
    char type;

    id = msg.msgID();
    id = htons(id);
    type = msg.msgType();

    write(_sock, (void *)&id, sizeof(uint16_t));
    write(_sock, (void *)&type, sizeof(char));
}

int RServerSession::_readByte(char *b)
{
    ssize_t readsize;

    //DBG("%d %d", (int)r->bufstart, (int)r->bufend);

    if (_bufstart == _bufend){
        readsize = read(_sock, _buf, SIM_RSERVER_BUFSIZE);
        if (readsize <= 0)
            return -1;

        _bufstart = _buf;
        _bufend = _buf + readsize;
    }

    *b = *_bufstart;
    ++_bufstart;

    return 0;
}

int RServerSession::_readID(uint16_t *id)
{
    char *cid = (char *)id;

    if (_readByte(cid + 0) != 0)
        return -1;
    if (_readByte(cid + 1) != 0)
        return -1;

    *id = ntohs(*id);
    return 0;
}

int RServerSession::_readType(char *type)
{
    return _readByte(type);
}



RServer::RServer(const char *addr, uint16_t port)
    : sim::Component(Component::PRIO_HIGHEST),
      _sim(0), _addr(addr), _port(port), _sock(-1)
{
    pthread_mutex_init(&_lock_sess, NULL);
    pthread_mutex_init(&_lock_to_join, NULL);
}

RServer::~RServer()
{
    pthread_mutex_destroy(&_lock_sess);
    pthread_mutex_destroy(&_lock_to_join);
}

void RServer::init(Sim *sim)
{
    struct sockaddr_in addr;

    // create tcp socket
    _sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(_sock < 0){
        ERR("RServer: Can not create socket.");
        return;
    }

    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(_port);
    addr.sin_addr.s_addr = INADDR_ANY; // TODO

    if(bind(_sock,(struct sockaddr *)&addr, sizeof(addr)) < 0){
        perror("RServer: ");
        close(_sock);
        _sock = -1;
        return;
    }

    if(listen(_sock, 10) < 0){
        perror("error listen failed");
        close(_sock);
        _sock = -1;
        return;
    }

    sim->regPreStep(this);
    sim->regMessage(this, RMessageOut::Type);
    sim->regMessage(this, RMessageOutPing::Type);
    sim->regMessage(this, RMessageOutPong::Type);
    _sim = sim;
}

void RServer::finish()
{
    std::list<RServerSession *>::iterator it, it_end;

    DBG("");

    pthread_mutex_lock(&_lock_sess);
    pthread_mutex_lock(&_lock_to_join);
    it = _sessions.begin();
    it_end = _sessions.end();
    for (; it != it_end; ++it){
        DBG(*it);
        (*it)->cancel();
        _sessions_to_join.push_back(*it);
    }
    _sessions.clear();
    pthread_mutex_unlock(&_lock_to_join);
    pthread_mutex_unlock(&_lock_sess);

    _joinSessions();

    if (_sock > 0){
        shutdown(_sock, SHUT_RDWR);
        close(_sock);
    }
}

void RServer::cbPreStep()
{
    // check for new connections
    _newConnections();

    // deliver all messages
    _deliverMsgs();

    // join ended sessions
    _joinSessions();
}

void RServer::processMessage(const sim::Message &msg)
{
    RMessageOut *omsg;

    if (msg.type() == RMessageOut::Type
            || msg.type() == RMessageOutPing::Type
            || msg.type() == RMessageOutPong::Type){
        omsg = (RMessageOut *)&msg;
        _sendRMessage((const RMessageOut &)msg);
    }
}

void RServer::_newConnections()
{
    fd_set fds;
    int ready, maxfd, connfd;
    struct timeval timeout;

    FD_ZERO(&fds);
    FD_SET(_sock, &fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 1;


    maxfd = _sock + 1;
    ready = select(maxfd, &fds, NULL, NULL, &timeout);
    if (ready > 0){
        connfd = accept(_sock, NULL, NULL);
        if (connfd < 0){
            perror("accept() error");
            return;
        }

        DBG("New connection " << connfd);
        _addSession(connfd);
    }
}

void RServer::_deliverMsgs()
{
    std::list<RMessage *>::iterator it, it_end;

    pthread_mutex_lock(&_lock_msgs);

    it = _msgs_to_deliver.begin();
    it_end = _msgs_to_deliver.end();
    for (; it != it_end; ++it){
        DBG("");
        _sim->sendMessage(*it);
    }
    _msgs_to_deliver.clear();

    pthread_mutex_unlock(&_lock_msgs);
}

void RServer::_joinSessions()
{
    std::list<RServerSession *>::iterator it, it_end;

    pthread_mutex_lock(&_lock_to_join);

    it = _sessions_to_join.begin();
    it_end = _sessions_to_join.end();
    for (; it != it_end; ++it){
        DBG(*it);
        delete *it;
    }
    _sessions_to_join.clear();

    pthread_mutex_unlock(&_lock_to_join);
}

void RServer::_addSession(int sock)
{
    RServerSession *sess;

    sess = new RServerSession(this, sock);

    pthread_mutex_lock(&_lock_sess);
    _sessions.push_back(sess);
    pthread_mutex_unlock(&_lock_sess);

    sess->run();
}

void RServer::__addSessionToJoin(RServerSession *sess)
{
    pthread_mutex_lock(&_lock_sess);
    _sessions.remove(sess);
    pthread_mutex_unlock(&_lock_sess);

    pthread_mutex_lock(&_lock_to_join);
    if (std::find(_sessions_to_join.begin(), _sessions_to_join.end(), sess)
            == _sessions_to_join.end()){
        _sessions_to_join.push_back(sess);
    }
    pthread_mutex_unlock(&_lock_to_join);
}

void RServer::__addMessage(RMessage *msg)
{
    pthread_mutex_lock(&_lock_msgs);
    _msgs_to_deliver.push_back(msg);
    pthread_mutex_unlock(&_lock_msgs);
}

void RServer::_sendRMessage(const RMessageOut &msg)
{
    std::list<RServerSession *>::iterator it, it_end;

    pthread_mutex_lock(&_lock_sess);

    it = _sessions.begin();
    it_end = _sessions.end();
    for (; it != it_end; ++it){
        (*it)->sendMessage(msg);
    }

    pthread_mutex_unlock(&_lock_sess);
}

}
}
