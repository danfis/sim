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


class Message {
    SIM_MESSAGE_INIT2(1, 0)

  public:
    Message() {}
    virtual ~Message();

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
