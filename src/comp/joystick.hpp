#ifdef HAVE_SDL

#ifndef _SIM_COMP_JOYSTICK_HPP_
#define _SIM_COMP_JOYSTICK_HPP_

#include <SDL/SDL.h>
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
    SDL_Joystick *_joystick;

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

#endif /* HAVE_SDL */
