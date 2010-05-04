#include "collision_detection.hpp"
#include "body.hpp"


namespace sim {

CollisionDispatcher::CollisionDispatcher(btCollisionConfiguration *conf)
    : btCollisionDispatcher(conf)
{
}

bool CollisionDispatcher::needsCollision(btCollisionObject *c0, btCollisionObject *c1)
{
    Body *b0, *b1;
    b0 = reinterpret_cast<Body *>(c0->getUserPointer());
    b1 = reinterpret_cast<Body *>(c1->getUserPointer());

    if (b0 && b1){
        if (b0->collInfo().dont_collide_id == b1->collInfo().dont_collide_id
                && b0->collInfo().dont_collide_id != 0){
            return false;
        }
    }

    return btCollisionDispatcher::needsCollision(c0, c1);
}

} /* namespace sim */
