#ifndef _SIM_SIM_HPP_
#define _SIM_SIM_HPP_

#include <osgViewer/Viewer>
#include "object.hpp"

namespace sim {

/**
 * Callback called from Sim::run() function during ODE simulation.
 */
void __ode_near_collision(void *data, dGeomID o1, dGeomID o2);

/**
 * Simulator.
 */
class Sim {
    double _sim_time;
    dContact _default_contact; /*! default setting of contact */

    osgViewer::Viewer *_viewer;
    osg::Group *_root; /*! root of scene graph */

    dWorldID _world;
    dSpaceID _space;
    dJointGroupID _coll_contacts;

    std::list<Object *> _objs; /*! list of objects */

    friend void __ode_near_collision(void *data, dGeomID o1, dGeomID o2);

  public:
    Sim();
    virtual ~Sim();

    /**
     * Adds object into internal list of objects.
     * This method steals pointer to given object! It means that destructor
     * is called from Sim and _can't_ be called outside.
     */
    virtual void addObject(Object *o);

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
