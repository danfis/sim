#ifndef _SIM_WORLD_HPP_
#define _SIM_WORLD_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "body.hpp"
#include "joint.hpp"
#include "actor.hpp"
#include "visworld.hpp"

namespace sim {


/**
 * Physical world.
 */
class World {
  protected:
    btCollisionConfiguration *_coll_conf;
    btCollisionDispatcher *_dispatch;
    btBroadphaseInterface *_broadphase;
    btConstraintSolver *_solver;
    btDynamicsWorld *_world;
    VisWorld *_vis;

    std::list<Actor *> _actors;
    std::list<Actor *> _pre_step;
    std::list<Actor *> _post_step;

    typedef std::list<Actor *>::iterator _act_it;

  public:
    World();
    virtual ~World();

    /**
     * Initializes world.
     */
    void init();

    /**
     * Destroys world.
     */
    void destroy();

    /**
     * Performes one step.
     */
    void step(bool phys = true, bool vis = true);

    bool done();


    void addJoint(Joint *j);
    void addBody(Body *obj);
    void addActor(Actor *a);

    void regActorPreStep(Actor *a);
    void regActorPostStep(Actor *a);

  protected:
    void _actorsRunPreStep();
    void _actorsRunPostStep();
};

}

#endif /* _SIM_WORLD_HPP_ */

