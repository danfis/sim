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

#include <netinet/in.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include "comp/rserver.hpp"
#include "msg.hpp"
#include "sim.hpp"
#include <osgDB/WriteFile>

namespace sim {
namespace comp {

RMessageOutRF::RMessageOutRF(uint16_t id, const sim::sensor::RangeFinder &rf)
    : RMessageOut(id, RMessage::MSG_RF), _dist(rf.numBeams())
{
    size_t i, len;
    const Scalar *dist = rf.distance();
    len = rf.numBeams();

    DBG("");
    for (i = 0; i < len; i++){
        _dist[i] = dist[i];
    }
}

RMessageOutImg::RMessageOutImg(uint16_t id, const sim::sensor::Camera &cam)
    : RMessageOut(id, RMessage::MSG_IMG), _data(NULL)
{
    const osg::Image *img;
    osg::Vec4 rgb;
    size_t i, j, k, w, h;

    img = cam.image();

    w = img->s();
    h = img->t();

    _width  = w;
    _height = h;
    _data   = new unsigned char[3 * w * h];

    k = 0;
    for (j = 0; j < h; j++){
        for (i = 0; i < w; i++){
            rgb = img->getColor(i, j);
            _data[k++] = (unsigned char)(rgb.b() * 255);
            _data[k++] = (unsigned char)(rgb.g() * 255);
            _data[k++] = (unsigned char)(rgb.r() * 255);
        }
    }
}

RMessageOutImg::~RMessageOutImg()
{
    if (_data)
        delete [] _data;
}


void *RServerSession::thread(void *_s)
{
    RServerSession *s = (RServerSession *)_s;
    uint16_t id;
    char type;
    float f;

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

        }else if (type == RMessage::MSG_GET_POS){
            s->_server->__addMessage(new RMessageInGetPos(id));

        }else if (type == RMessage::MSG_GET_ROT){
            s->_server->__addMessage(new RMessageInGetRot(id));

        }else if (type == RMessage::MSG_SET_VEL_LEFT){
            if (s->_readFloat(&f) != 0)
                break;
            s->_server->__addMessage(new RMessageInSetVelLeft(id, f));

        }else if (type == RMessage::MSG_SET_VEL_RIGHT){
            if (s->_readFloat(&f) != 0)
                break;
            s->_server->__addMessage(new RMessageInSetVelRight(id, f));

        }else if (type == RMessage::MSG_GET_RF){
            s->_server->__addMessage(new RMessageInGetRF(id));

        }else if (type == RMessage::MSG_GET_IMG){
            s->_server->__addMessage(new RMessageInGetImg(id));

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
    _writeID(msg.msgID());
    _writeType(msg.msgType());

    DBG(msg.msgID() << " " << msg.msgType());
    if (msg.msgType() == RMessage::MSG_POS){
        const sim::Vec3 &v = ((const RMessageOutPos *)&msg)->pos();
        _writeFloat(v.x());
        _writeFloat(v.y());
        _writeFloat(v.z());

    }else if (msg.msgType() == RMessage::MSG_ROT){
        const sim::Quat &v = ((const RMessageOutRot *)&msg)->rot();
        _writeFloat(v.x());
        _writeFloat(v.y());
        _writeFloat(v.z());
        _writeFloat(v.w());

    }else if (msg.msgType() == RMessage::MSG_RF){
        DBG("");
        size_t i, len;
        const std::vector<Scalar> &dist = ((const RMessageOutRF *)&msg)->distance();

        len = dist.size();
        _writeUInt16(len);
        for (i = 0; i < len; i++){
            _writeFloat(dist[i]);
        }

    }else if (msg.msgType() == RMessage::MSG_IMG){
        size_t i, len;
        size_t width = ((const RMessageOutImg *)&msg)->width();
        size_t height = ((const RMessageOutImg *)&msg)->height();
        const unsigned char *data = ((const RMessageOutImg *)&msg)->data();

        DBG("Sending MSG_IMG " << width << "x" << height);

        _writeUInt16(width);
        _writeUInt16(height);

        len = width * height * 3;
        for (i = 0; i < len; i++){
            _writeByte(data[i]);
        }
    }
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

int RServerSession::_readFloat(float *f)
{
    uint32_t i;
    char *c, *cf;

    c = (char *)&i;
    if (_readByte(c + 0) != 0)
        return -1;
    if (_readByte(c + 1) != 0)
        return -1;
    if (_readByte(c + 2) != 0)
        return -1;
    if (_readByte(c + 3) != 0)
        return -1;

    i = ntohl(i);
    cf = (char *)f;
    cf[0] = c[0];
    cf[1] = c[1];
    cf[2] = c[2];
    cf[3] = c[3];

    return 0;
}


int RServerSession::_writeID(uint16_t _id)
{
    return _writeUInt16(_id);
}

int RServerSession::_writeUInt16(uint16_t _i)
{
    ssize_t size;
    uint16_t i;

    i = htons(_i);

    size = write(_sock, (void *)&i, sizeof(uint16_t));
    if (size != sizeof(uint16_t))
        return -1;
    return 0;
}

int RServerSession::_writeType(char type)
{
    return _writeByte(type);
}

int RServerSession::_writeByte(char type)
{
    ssize_t size;

    size = write(_sock, &type, sizeof(char));
    if (size != 1)
        return -1;
    return 0;
}

int RServerSession::_writeFloat(float f)
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

    size = write(_sock, (void *)&i, 4);
    if (size != 4)
        return -1;
    return 0;
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

    if (msg.type() == RMessageOut::Type){
        omsg = (RMessageOut *)&msg;
        _sendRMessage((const RMessageOut &)omsg);
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
