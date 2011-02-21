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
#include <vector>
#include <sim/config.hpp>
#include <sim/component.hpp>
#include <sim/math.hpp>
#include <sim/sensor/rangefinder.hpp>

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
        MSG_PING          = 1,
        MSG_PONG          = 2,
        MSG_GET_POS       = 3,
        MSG_POS           = 4,
        MSG_GET_ROT       = 5,
        MSG_ROT           = 6,
        MSG_SET_VEL_LEFT  = 7,
        MSG_SET_VEL_RIGHT = 8,
        MSG_GET_RF        = 9,
        MSG_RF            = 10
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

class RMessageInGetPos : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 103)
  public:
    RMessageInGetPos(uint16_t id)
        : RMessageIn(id, RMessage::MSG_GET_POS) {}
};

class RMessageInGetRot : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 104)
  public:
    RMessageInGetRot(uint16_t id)
        : RMessageIn(id, RMessage::MSG_GET_ROT) {}
};

class RMessageInSetVelLeft : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 105)

  private:
    float _vel;

  public:
    RMessageInSetVelLeft(uint16_t id, float vel)
        : RMessageIn(id, RMessage::MSG_SET_VEL_LEFT), _vel(vel) {}

    float msgVel() const { return _vel; }
};

class RMessageInSetVelRight : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 106)

  private:
    float _vel;

  public:
    RMessageInSetVelRight(uint16_t id, float vel)
        : RMessageIn(id, RMessage::MSG_SET_VEL_RIGHT), _vel(vel) {}

    float msgVel() const { return _vel; }
};

class RMessageInGetRF : public RMessageIn {
    SIM_MESSAGE_INIT2(1, 107)
  public:
    RMessageInGetRF(uint16_t id)
        : RMessageIn(id, RMessage::MSG_GET_RF) {}
};




class RMessageOut : public RMessage {
    SIM_MESSAGE_INIT2(1, 0)
  public:
    RMessageOut(uint16_t id, char type)
        : RMessage(id, type) {}
};

class RMessageOutPing : public RMessageOut {
  public:
    RMessageOutPing(uint16_t id)
        : RMessageOut(id, RMessage::MSG_PING) {}
};

class RMessageOutPong : public RMessageOut {
  public:
    RMessageOutPong(uint16_t id)
        : RMessageOut(id, RMessage::MSG_PONG) {}
};

class RMessageOutPos : public RMessageOut {
  private:
    sim::Vec3 _v;

  public:
    RMessageOutPos(uint16_t id, const sim::Vec3 &v)
        : RMessageOut(id, RMessage::MSG_POS), _v(v) {}

    const sim::Vec3 &pos() const { return _v; }
};

class RMessageOutRot : public RMessageOut {
  private:
    sim::Quat _q;

  public:
    RMessageOutRot(uint16_t id, const sim::Quat &q)
        : RMessageOut(id, RMessage::MSG_ROT), _q(q) {}

    const sim::Quat &rot() const { return _q; }
};

class RMessageOutRF : public RMessageOut {
    std::vector<Scalar> _dist;

  public:
    RMessageOutRF(uint16_t id, const sim::sensor::RangeFinder &rf);
    const std::vector<Scalar> &distance() const { return _dist; }
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
    int _readFloat(float *f);

    int _writeID(uint16_t id);
    int _writeType(char type);
    int _writeUInt16(uint16_t i);
    int _writeFloat(float f);
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

