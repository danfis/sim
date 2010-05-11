/*!
\page man_message How to define new Message

Message is one method Components can comunicate with each other or how to
send any information to Components from outside.

Every new Message must be declared as child of
\ref sim::Message class.

Every new Message must have assigned type ID that distinguish messages from
each other. Message has also assigned priority that controls order of
processing Messages.

\section man_message_new Creating new Message
Let start with simple example:
\code
#include <sim/message.hpp>

class MBase : public sim::Message {
    SIM_MESSAGE_INIT(1)
  public:
    MBase(sim::Message::Priority prio)
        : sim::Message(prio) {}
};

class M1 : public MBase {
    SIM_MESSAGE_INIT(2)
  public:
    M1() : MBase(sim::Message::PRIO_HIGHER) {}
};

class M2 : public MBase {
    SIM_MESSAGE_INIT(3)

  private:
    std::string _msg;

  public:
    M2(const std::string &msg)
        : MBase(sim::Message::PRIO_NORMAL),
          _msg(msg)
        {}

    const std::string &msg() const { return _msg; }
};

\endcode

Example above defines three Messages with different type ID and priorities.
ID of Message is defined using SIM_MESSAGE_INIT() macro (ID 0 is reserved
for system messages, so don't use it) and priority is set
using sim::Message contructor. Type of Message can be lately obtained by
method .type() or by class's static member ::Type and you can use them to
register Component or to dispatch Messages in Component's
\ref sim::Component::processMessage "processMessage()" method:
\code
// create new simulator
Sim *sim = new Sim();

// create new Component
Comp *c = new Comp();

// register component to M1 and M2 message
sim->regMessage(c, M1::Type);
sim->regMessage(c, M2::Type);
\endcode

\code
// Dispatching messages in processMessage() method:
void Comp::processMessage(const sim::Message &msg)
{
    if (msg.type() == M1::Type){
        std::cout << "Hooray, I got M1 message!" << std::endl;
    }else if (msg.type() == M2::Type){
        std::cout << "Oh no, M2 again..." << std::endl;

        // lets look at M2's message:
        const M2 &m2 = (const M2 &)msg;
        // you can also use dynamic_cast through ::cast() static method:
        // const M2 *m2 = msg.cast<const M2>(&msg);
        std::cout << "  M2's msg is " << M2.msg() << std::endl;
    }
}
\endcode

Messages can be sent to registered Components:
\code
sim->sendMessage(new M1());
sim->sendMessage(new M2("Some message"));
\endcode

You can also change priority using sendMessage() method:
\code
sim->sendMessage(new M1(), sim::Message::PRIO_HIGHEST);
\endcode
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
    SIM_MESSAGE_INIT2(1, 0)

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


} /* namespace sim */

#endif /* _SIM_MESSAGE_HPP_ */
