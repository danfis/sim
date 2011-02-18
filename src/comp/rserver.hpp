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

#ifndef _SIM_RSERVER_HPP_
#define _SIM_RSERVER_HPP_

#include <string>
#include <stdint.h>
#include <sim/config.hpp>
#include "sim/component.hpp"

#define SIM_RSERVER_BUFSIZE 1

namespace sim {

namespace comp {

class RServer;

class RMessage : public sim::Message {
  private:
    uint16_t _id;
    char _type;

  public:
    RMessage(uint16_t id, char type)
        : Message(Message::PRIO_HIGHEST), _id(id), _type(type) {}

    uint16_t msgID() const { return _id; }
    char msgType() const { return _type; }

    // Note that this must be synchronized with /rsim/rsim.h!!
    enum {
        MSG_PING = 1,
        MSG_PONG = 2
    };
};

class RMessageIn : public RMessage {
    SIM_MESSAGE_INIT2(1, 100)
  public:
    RMessageIn(uint16_t id, char type)
        : RMessage(id, type) {}
};

class RMessageInPing : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 101)
  public:
    RMessageInPing(uint16_t id)
        : RMessageIn(id, RMessage::MSG_PING) {}
};

class RMessageInPong : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 102)
  public:
    RMessageInPong(uint16_t id)
        : RMessageIn(id, RMessage::MSG_PONG) {}
};



class RMessageOut : public RMessage {
    SIM_MESSAGE_INIT2(1, 0)
  public:
    RMessageOut(uint16_t id, char type)
        : RMessage(id, type) {}
};

class RMessageOutPing : public RMessageOut {
    SIM_MESSAGE_INIT2(1, 1)
  public:
    RMessageOutPing(uint16_t id)
        : RMessageOut(id, RMessage::MSG_PING) {}
};

class RMessageOutPong : public RMessageOut {
    SIM_MESSAGE_INIT2(1, 2)
  public:
    RMessageOutPong(uint16_t id)
        : RMessageOut(id, RMessage::MSG_PONG) {}
};


class RServerSession {
  protected:
    RServer *_server;
    int _sock;
    pthread_t _th;
    char _buf[SIM_RSERVER_BUFSIZE];
    char *_bufstart, *_bufend;

  public:
    RServerSession(RServer *server, int sock);
    ~RServerSession();

    void run();
    void cancel();

    void sendMessage(const RMessageOut &msg);

  private:
    static void *thread(void *);

    int _readByte(char *c);
    int _readID(uint16_t *id);
    int _readType(char *type);
};

class RServer : public sim::Component {
  protected:
    sim::Sim *_sim;

    std::string _addr;
    uint16_t _port;
    int _sock;

    std::list<RServerSession *> _sessions; /*!< List of active sessions */
    std::list<RServerSession *> _sessions_to_join; /*!< List of sessions that
                                                        should be joined and
                                                        closed */
    std::list<RMessage *> _msgs_to_deliver; /*!< List of messages to
                                                 deliver */

    pthread_mutex_t _lock_sess;
    pthread_mutex_t _lock_to_join; /*!< Lock for _sessions_to_join */
    pthread_mutex_t _lock_msgs;

  public:
    RServer(const char *addr, uint16_t port);
    ~RServer();

    virtual void init(Sim *sim);
    virtual void finish();
    virtual void cbPreStep();
    virtual void processMessage(const sim::Message &msg);

    void __addSessionToJoin(RServerSession *sess);
    void __addMessage(RMessage *msg);

  private:
    void _newConnections();
    void _deliverMsgs();
    void _joinSessions();

    void _addSession(int sock);

    void _sendRMessage(const RMessageOut &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_RSERVER_HPP_ */

