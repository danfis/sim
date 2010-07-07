/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sim/bullet/collision_detection.hpp"
#include "sim/bullet/body.hpp"
#include "sim/msg.hpp"


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
