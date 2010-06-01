#include "joystick.hpp"
#include <sim/msg.hpp>

namespace sim {
namespace comp {

Joystick::Joystick(int num, const sim::Time &delay)
    : sim::Component(),
      _num(num), _delay(delay),
      _joystick(0),
      _buttons(0)
{
}

Joystick::~Joystick()
{
}

void Joystick::init(sim::Sim *sim)
{
#ifndef HAVE_SDL
    bool init = false;

    _sim = sim;

    _joystick = 0;

    if (SDL_WasInit(SDL_INIT_JOYSTICK) == 0){
        if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0){
            ERR("sim::comp::Joystick: Can't initialize Joystick!");
        }else{
            init = true;
        }
    }else{
        init = true;
    }

    if (init){
        if (SDL_NumJoysticks() <= _num){
            ERR("sim::comp::Joystick: Joystick " << _num << " is not available");
        }else if (SDL_JoystickOpened(_num)){
            ERR("sim::comp::Joystick: Joystick " << _num << " is already opened");
        }else{
            _joystick = SDL_JoystickOpen(_num);
            SDL_JoystickEventState(SDL_IGNORE);

            MSG("sim::comp::Joystick: Joystick " << _num << " found: " << SDL_JoystickName(_num));

            _buttons = SDL_JoystickNumButtons(_joystick);
            MSG("sim::comp::Joystick: Joystick " << _num << " has " << _buttons << " buttons");
        }
    }

    if (_joystick){
        _timer.start();

        _sim->regPostStep(this);
    }
#endif /* HAVE_SDL */
}

void Joystick::finish()
{
#ifdef HAVE_SDL
    if (_joystick){
        SDL_JoystickClose(_joystick);
        _joystick = 0;
    }
#endif /* HAVE_SDL */
}

void Joystick::cbPostStep()
{
#ifdef HAVE_SDL
    if (_timer.stop() < _delay)
        return;

    SDL_JoystickUpdate();

    JoystickMessage *msg = new JoystickMessage(_num, _buttons);

    for (size_t i = 0; i < _buttons; i++){
        msg->setButton(i, (bool)SDL_JoystickGetButton(_joystick, i));
    }

    _timer.start();

    _sim->sendMessage(msg);
#endif /* HAVE_SDL */
}

}
}
