/***
 * Here should be all functions and classes which modifies collision
 * detection phase.
 */

#ifndef _SIM_BULLET_COLLISION_DETECTION_HPP_
#define _SIM_BULLET_COLLISION_DETECTION_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>

namespace sim {

namespace bullet {

class CollisionDispatcher : public btCollisionDispatcher {
  public:
    CollisionDispatcher(btCollisionConfiguration *conf);
    virtual ~CollisionDispatcher() {}

    bool needsCollision(btCollisionObject *b0, btCollisionObject *b1);
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_COLLISION_DETECTION_HPP_ */
