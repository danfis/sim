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

#include <sim/comp/sssa.hpp>

namespace sim {
namespace comp {

SSSA::SSSA(const Vec3 &pos, const Quat &rot)
    : sim::Component(),
      _sim(0), _robot(0),
      _with_wheels(true), _with_boxes(false),
      _pos(pos), _rot(rot)
{
}

SSSA::SSSA(bool with_wheels, bool with_boxes,
           const sim::Vec3 &pos, const sim::Quat &rot)
    : sim::Component(),
      _sim(0), _robot(0),
      _with_wheels(with_wheels), _with_boxes(with_boxes),
      _pos(pos), _rot(rot)
{
}

SSSA::~SSSA()
{
    if (_robot)
        delete _robot;
}

void SSSA::init(sim::Sim *sim)
{
    _sim = sim;

    _robot = new sim::robot::SSSA(_sim->world(),
                                  _with_wheels, _with_boxes,
                                  _pos, _rot);
    _robot->activate();
}

}
}

