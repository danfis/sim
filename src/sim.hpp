#ifndef _SIM_SIM_HPP_
#define _SIM_SIM_HPP_

#include "world.hpp"
#include "visworld.hpp"

namespace sim {

/**
 * Simulator.
 */
class Sim {
    World *_world;
    VisWorld *_visworld;

  public:
    Sim();
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
};

}

#endif /* _SIM_SIM_HPP_ */
