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

#ifndef _SIM_COMP_POVRAY_STEP_HPP_
#define _SIM_COMP_POVRAY_STEP_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>


namespace sim {

namespace comp {

    
 /**
  * Component for exporting whole scene into povray format (for later
  * rendering of high quality images).
  *
  * This class generates one .inc file per VisBody and is able to detect
  * that some VisBody was added to scene. During a simulation is then
  * generated one .pov file per frame where objects are referenced trough
  * .inc files. This method is fast (as opposite to PovrayFull) but can be
  * inaccurate because:
  *     1) no change of color and shape (or dimension) of VisBody during
  *        simulation detected
  *     2) no deactivation of VisBody is detected
  * but for most proposes is this method good enough.
  *
  * For rendering use program povray(1) directly or use processPovray.pl
  * script shipped with this library.
  */
class PovrayStep : public sim::Component {
  protected:
    sim::Sim *_sim;
    unsigned long _frame;
    const char *_dir;
    unsigned long _last_id;

  public:
    /**
     * Constructor. Takes path to directory where files will be put as
     * first argument.
     */
    PovrayStep(const char *dir = "povray");
    ~PovrayStep();

    void init(sim::Sim *sim);
    void finish();
    void cbPostStep();

  protected:
    void _cameraFile();
    void _lightsFile();
    void _updateObjects();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_POVRAY_STEP_HPP_ */


