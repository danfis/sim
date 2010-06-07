/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

#ifndef _SIM_MESSAGE_HPP_
#define _SIM_MESSAGE_HPP_

#include <typeinfo>

namespace sim {

// long has at least 32 bits, so lowest 16 is minor number and higest 16 is
// major 

#define SIM_MESSAGE_MAKE_MAJOR(N) \
    (((unsigned long)(N) & 0xffffUL) << 16UL)
#define SIM_MESSAGE_MAKE_MINOR(N) \
    ((unsigned long)(N) & 0xffffUL)

/**
 * Composes ID from MAJOR and MINOR number. Resulting number has type of
 * unsigned long.
 */
#define SIM_MESSAGE_MAKE_ID(MAJOR, MINOR) \
    (SIM_MESSAGE_MAKE_MAJOR(MAJOR) \
        | SIM_MESSAGE_MAKE_MINOR(MINOR))

/**
 * Rips MAJOR ID from ID.
 */
#define SIM_MESSAGE_MAJOR(ID) \
    (((unsigned long)(ID) >> 16UL) & 0xffffUL)

/**
 * Rips MINOR ID from ID.
 */
#define SIM_MESSAGE_MINOR(ID) \
    ((unsigned long)(ID) & 0xffffUL)

/**
 * Initializes class inherited from Message class with major ID of class.
 */
#define SIM_MESSAGE_INIT(MAJOR) \
    SIM_MESSAGE_INIT2(MAJOR, 0)

#define SIM_MESSAGE_INIT2(MAJOR, MINOR) \
    public: \
        virtual unsigned long type() const \
            { return SIM_MESSAGE_MAKE_ID(MAJOR, MINOR); } \
        \
        static const unsigned long Type = SIM_MESSAGE_MAKE_ID(MAJOR, MINOR); \
        static const unsigned long TypeMajor = SIM_MESSAGE_MAKE_MAJOR(MAJOR); \
        static const unsigned long TypeMinor = SIM_MESSAGE_MAKE_MINOR(MINOR);


/**
 * Class representing Message that can be sent by Sim::sendMessage() method
 * to registered Components.
 * See \ref man_message for more info.
 */
class Message {
    SIM_MESSAGE_INIT2(0, 0)

  public:
    enum Priority {
        PRIO_LOWEST = 0,
        PRIO_LOWER,
        PRIO_NORMAL,
        PRIO_HIGHER,
        PRIO_HIGHEST,
        PRIO_MAX
    };

  protected:
    Priority _prio;

  public:
    Message(Priority prio = PRIO_NORMAL) : _prio(prio) {}
    virtual ~Message();

    Priority prio() const { return _prio; }
    void setPrio(Message::Priority p) { _prio = p; }

    /* final */ unsigned long typeMajor() const { return type() >> 16UL; }
    /* final */ unsigned long typeMinor() const { return type() & 0xffffUL; }

    /**
     * Casts Message msg to specified type.
     * Expensive dynamic_cast function is used so type checking is
     * performed and if msg is not convertible to To, 0 is returned.
     */
    template <typename To>
    static To *cast(Message *msg) { return dynamic_cast<To *>(msg); }
};

class MessageKeyPressed : public Message {
    SIM_MESSAGE_INIT2(0, 1)

  private:
    int _key;

  public:
    MessageKeyPressed(int key) : Message(), _key(key) {}

    int key() const { return _key; }
};


} /* namespace sim */

#endif /* _SIM_MESSAGE_HPP_ */
