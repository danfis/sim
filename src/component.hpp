#ifndef _SIM_COMPONENT_HPP_
#define _SIM_COMPONENT_HPP_

#include <list>

#include "sim/message.hpp"


namespace sim {

// forward declaration
class Sim;
class SimComponentMessageRegistry;

class Component {
  private:
    friend class SimComponentMessageRegistry;

    /** _only_ SimComponentMessageRegistry should touch this! */
    std::list<const Message *> __msgs_to_deliver[Message::PRIO_MAX];

  public:
    enum Priority {
        PRIO_LOWEST = 0,
        PRIO_LOWER,
        PRIO_NORMAL,
        PRIO_HIGH,
        PRIO_HIGHEST,
        PRIO_MAX
    };

  protected:
    Sim *_sim;
    Priority _prio;

  public:
    Component(Priority prio = PRIO_NORMAL) : _sim(0), _prio(prio) {}
    virtual ~Component();

    Priority prio() const { return _prio; }

    Sim *sim() { return _sim; }
    const Sim *sim() const { return _sim; }
    void setSim(Sim *s) { _sim = s; }

    virtual void init(Sim *sim) {}
    virtual void finish() {}

    /**
     * Callback called from Sim before each step (if component is registered to
     * this callback).
     */
    virtual void cbPreStep() {}

    /**
     * Callback called from Sim after each step (if component is registered to
     * this callback).
     */
    virtual void cbPostStep() {}

    virtual void processMessage(const Message &msg) {}
};

} /* namespace sim */

#endif /* _SIM_COMPONENT_HPP_ */
