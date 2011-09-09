/***
 * sim
 * ---------------------------------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
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


#include <sim/world.hpp>

namespace sim {
#ifndef SIM_HAVE_ODE
WorldODE *ODE()
{
    return NULL;
}
#endif /* SIM_HAVE_ODE */

#ifndef SIM_HAVE_BULLET
WorldBullet *Bullet()
{
    return NULL;
}
#endif /* SIM_HAVE_BULLET */

} /* namespace sim */
