#ifndef _SIM_SIM_HPP_
#define _SIM_SIM_HPP_

#include "world.hpp"
#include "visworld.hpp"
#include "component.hpp"

namespace sim {

/**
 * Simulator.
 */
class Sim {
    World *_world;
    VisWorld *_visworld;

    std::list<Component *> _cs; //!< List of all components

    /**
     * List of components registered for preStep callback
     * (see Component::cbPreStep()).
     */
    std::list<Component *> _cs_pre;

    /**
     * List of components registered for postStep callback
     * (see Component::cbPostStep()).
     */
    std::list<Component *> _cs_post;

  protected:
    typedef std::list<Component *>::iterator cit_t; //!< Component list iterator
    typedef std::list<Component *>::const_iterator const_cit_t;

  public:
    enum WorldTypes {
        WorldTypeBullet
    };

  public:
    Sim(World *world = 0, VisWorld *visworld = 0);
    virtual ~Sim();

    World *world() { return _world; }
    const World *world() const { return _world; }
    VisWorld *visWorld() { return _visworld; }
    const VisWorld *visWorld() const { return _visworld; }

    void setWorld(World *w);
    void setVisWorld(VisWorld *w);

    virtual void init();
    virtual void step();
    virtual void finish();
    virtual bool done();

    /**
     * Run simulation.
     */
    virtual void run();

    /**
     * Is called when key is pressed.
     * Overload this method if you need to.
     */
    virtual bool pressedKey(int key) { return false; }

    /**
     * Adds component to simulator if it wasn't already added.
     *
     * Component's init() function is called.
     */
    void addComponent(Component *c);

    /**
     * Removes component from simulator if it was added before.
     *
     * Component's finish() function is called.
     */
    void rmComponent(Component *c);

    /**
     * Registers Component for cbPreStep() callback.
     * Component is registered only if it is already added using
     * addComponent() method.
     */
    void regPreStep(Component *c);

    /**
     * Register Component for cbPostStep() callback.
     */
    void regPostStep(Component *c);

  protected:
    /**
     * Returns true if Component was added.
     */
    bool _hasComponent(const Component *c) const;

    /**
     * Returns true if Component is registered for cbPreStep() callback.
     */
    bool _hasPreStep(const Component *c) const;

    /**
     * Returns true if Component is registered for cbPreStep() callback.
     */
    bool _hasPostStep(const Component *c) const;

    /**
     * Returns true if Component is in list.
     */
    bool _isComponentInList(const Component *c,
                            const std::list<Component *> &list) const;

    /**
     * Calls all Component::cbPreStep() methods on all registered
     * components.
     */
    void _cbPreStep();

    /**
     * Calls all Component::cbPostStep() methods on all registered
     * components.
     */
    void _cbPostStep();
};

}

#endif /* _SIM_SIM_HPP_ */
