/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

#ifndef _SIM_COMPONENT_HPP_
#define _SIM_COMPONENT_HPP_

#include <list>

#include "sim/message.hpp"

namespace sim {

// forward declaration
class Sim;
class SimComponentMessageRegistry;

class Component {
  private:
    friend class SimComponentMessageRegistry;

    /** _only_ SimComponentMessageRegistry should touch this! */
    std::list<const Message *> __msgs_to_deliver[2][Message::PRIO_MAXIMUM];

  public:
    enum Priority {
        PRIO_LOWEST = 0,
        PRIO_LOWER,
        PRIO_NORMAL,
        PRIO_HIGH,
        PRIO_HIGHEST,
        PRIO_MAXIMUM
    };

  protected:
    Sim *_sim;
    Priority _prio;

  public:
    Component(Priority prio = PRIO_NORMAL) : _sim(0), _prio(prio) {}
    virtual ~Component();

    Priority prio() const { return _prio; }

    Sim *sim() { return _sim; }
    const Sim *sim() const { return _sim; }
    void setSim(Sim *s) { _sim = s; }

    virtual void init(Sim *sim) {}
    virtual void finish() {}

    /**
     * Callback called from Sim before each step (if component is registered to
     * this callback).
     */
    virtual void cbPreStep() {}

    /**
     * Callback called from Sim after each step (if component is registered to
     * this callback).
     */
    virtual void cbPostStep() {}

    virtual void processMessage(const Message &msg) {}
};

} /* namespace sim */

#endif /* _SIM_COMPONENT_HPP_ */
