/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
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

#ifndef _SIM_COMP_BLENDER_HPP_
#define _SIM_COMP_BLENDER_HPP_

#include <sim/sim.hpp>
#include <sim/time.hpp>
#include <vector>
#include <string>


namespace sim {

namespace comp {

    
class Blender : public sim::Component {
	sim::Sim *_sim;
	int _frame;
    const char *_prefix;
    double _frameTime;
    sim::Time _lastFrameTime;

	public:
    /** prefix specifies directory for output puictures. if the directory does not exist, no pictures wil
      * be created. FrameTime specifies time of one frame. this can be used .e.g for porducing movie, where one frame
      * is assumed to be 1/24 s long. if frameTime==-1, all simulation frames are outputted */
	Blender(const char *prefix = "", const double frameTime = 1.0/24.0);
	~Blender();

	void init(sim::Sim *sim);
	void finish();
	void cbPostStep();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_POVRAY_HPP_ */
