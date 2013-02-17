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

#ifndef _SIM_COMP_JOYSTICK_HPP_
#define _SIM_COMP_JOYSTICK_HPP_

#include <sim/config.hpp>

#ifdef SIM_HAVE_SDL
# include <SDL/SDL.h>
#endif /* SIM_HAVE_SDL */

#include <sim/sim.hpp>
#include <sim/component.hpp>
#include <sim/message.hpp>
#include <sim/time.hpp>

namespace sim {

namespace comp {

class Joystick : public sim::Component {
  protected:
    sim::Sim *_sim;
    int _num;
    sim::Time _delay;
    sim::Timer _timer;
#ifdef SIM_HAVE_SDL
    SDL_Joystick *_joystick;
#else /* SIM_HAVE_SDL */
    void *_joystick;
#endif /* SIM_HAVE_SDL */

    size_t _buttons;

  public:
    Joystick(int num = 0, const sim::Time &delay = sim::Time(0, 200000000UL));
    ~Joystick();

    const Time &joystickDelay() const { return _delay; }
    void setJoystickDelay(const Time &t) { _delay = t; }

    void init(sim::Sim *sim);
    void finish();
    void cbPostStep();
};

class JoystickMessage : public sim::Message {
    SIM_MESSAGE_INIT2(0, 10)

  protected:
    int _num;
    std::vector<bool> _buttons;
    
  public:
    JoystickMessage(int num, size_t num_buttons)
        : sim::Message(),
          _num(num), _buttons(num_buttons) { }

    int joystickNum() const { return _num; }
    size_t numButtons() const { return _buttons.size(); }
    bool button(size_t num) const { return _buttons[num]; }
    void setButton(size_t num, bool val) { _buttons[num] = val; }
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_JOYSTICK_HPP_ */
