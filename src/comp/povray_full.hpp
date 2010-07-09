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

#ifndef _SIM_COMP_POVRAY_FULL_HPP_
#define _SIM_COMP_POVRAY_FULL_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>


namespace sim {

namespace comp {

    
 /**
  * Povray component
  */
class PovrayFull : public sim::Component {
    sim::Sim *_sim;
    unsigned long _frame;
    const char *_dir;

  public:
    PovrayFull(const char *dir = "povray");
    ~PovrayFull();

    void init(sim::Sim *sim);
    void finish();
    void cbPostStep();

  protected:
    void _cameraFile();
    void _lightsFile();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_POVRAY_FULL_HPP_ */

