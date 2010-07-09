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
  * Component for exporting whole scene into povray format (for later
  * rendering of high quality images).
  *
  * This class generates one povray file after each simulation step this
  * _one_ file contains _whole_ scene. Using this component significantly
  * slows down a simulation but provides accurate description of scene.
  *
  * For rendering use program povray(1) directly or use processPovray.pl
  * script shipped with this library.
  */
class PovrayFull : public sim::Component {
    sim::Sim *_sim;
    unsigned long _frame;
    const char *_dir;

  public:
    /**
     * Constructor. Takes path to directory where files will be put as
     * first argument.
     */
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

