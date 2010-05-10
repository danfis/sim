#include "sim/bullet/collision_detection.hpp"
#include "sim/bullet/body.hpp"


namespace sim {

namespace bullet {

CollisionDispatcher::CollisionDispatcher(btCollisionConfiguration *conf)
    : btCollisionDispatcher(conf)
{
}

bool CollisionDispatcher::needsCollision(btCollisionObject *c0, btCollisionObject *c1)
{
    Body *b0, *b1;
    b0 = (Body *)c0->getUserPointer();
    b1 = (Body *)c1->getUserPointer();

    if (b0 && b1){
        if (b0->collInfo().dont_collide_id == b1->collInfo().dont_collide_id
                && b0->collInfo().dont_collide_id != 0){
            return false;
        }
    }

    return btCollisionDispatcher::needsCollision(c0, c1);
}

} /* namespace bullet */

} /* namespace sim */
