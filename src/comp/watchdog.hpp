/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
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

#ifndef _SIM_COMP_WATCHDOG_HPP_
#define _SIM_COMP_WATCHDOG_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>
#include <sim/joint.hpp>

namespace sim {

namespace comp {

/**Simple watchdog:
  After predefined time, position of the given body is printed to a file and simulation is terminated.
  If simulatedTime is true, then sim->timeSimulated is watched, otherwise sim->timeReal is watched
  */
class Watchdog : public sim::Component {
	sim::Sim *_sim;
	sim::Body *_body;
    double _timeout;
	const char *_paramFile;
    bool _simulatedTime;
	public:

	Watchdog(sim::Body *body, const double timeout, const char *paramFile, const bool simulatedTime);
	~Watchdog();


	void init(sim::Sim *sim);
	void finish();
	void cbPostStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
