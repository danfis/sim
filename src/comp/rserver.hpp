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
    SIM_MESSAGE_INIT2(10, 0)

  private:
    uint16_t _id;
    char _type;

  public:
    RMessage(uint16_t id, char type) : Message(Message::PRIO_HIGHEST), _id(id), _type(type) {}

    uint16_t msgID() const { return _id; }
    char msgType() const { return _type; }
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

    void __addSessionToJoin(RServerSession *sess);
    void __addMessage(RMessage *msg);

  private:
    void _newConnections();
    void _deliverMsgs();
    void _joinSessions();

    void _addSession(int sock);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_RSERVER_HPP_ */

