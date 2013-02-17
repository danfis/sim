/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *                   Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_COMP_SNAKE_BODIES_HPP_
#define _SIM_COMP_SNAKE_BODIES_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>
#include <sim/joint.hpp>
#include <sim/robot/sssa.hpp>

namespace sim {

namespace comp {


/** Simple keybord 'controller' for changing velocities of joints
  *
  * You can select desired joint by 'n' and 'm' keys (they loop between all joints)
  *
  * We assume that all joints are Hinge2 type, so there are two axis: axis1 (controller by Up/Down
  * keys) and axis2 (controlled by Left/Right  keys)
  */

class SnakeBody : public sim::Component {
	sim::Sim *_sim;
	std::vector<sim::robot::SSSA *> _snakeBodies;
	std::vector<double> vel1,vel2;
	osg::Vec4 oldColor;
	int selectedRobot;

	public:
	SnakeBody(const std::vector<sim::robot::SSSA *> &snakeBodies);
	~SnakeBody();


	void init(sim::Sim *sim);
	void finish();
	void cbPreStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
