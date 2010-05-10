#ifndef _SIM_COMPONENT_HPP_
#define _SIM_COMPONENT_HPP_


namespace sim {

// forward declaration
class Sim;

class Component {
  protected:
    Sim *_sim;

  public:
    Component() : _sim(0) {}
    virtual ~Component();

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
};

} /* namespace sim */

#endif /* _SIM_COMPONENT_HPP_ */
