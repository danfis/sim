/***
 * Here should be all functions and classes which modifies collision
 * detection phase.
 */

#ifndef _SIM_COLLISION_DETECTION_HPP_
#define _SIM_COLLISION_DETECTION_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>

namespace sim {

class CollisionDispatcher : public btCollisionDispatcher {
  public:
    CollisionDispatcher(btCollisionConfiguration *conf);
    virtual ~CollisionDispatcher() {}

    bool needsCollision(btCollisionObject *b0, btCollisionObject *b1);
};

} /* namespace sim */

#endif /* _SIM_COLLISION_DETECTION_HPP_ */
